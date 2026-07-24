# -*- coding: utf-8 -*-
"""
功能说明: 手眼标定验证工具

本脚本用于验证手眼标定结果 ( T_end_cam ) 的准确性。核心思路是:
  1. 通过相机定位标定板, 获取 T_cam_board ( 相机到标定板的变换矩阵 )
  2. 利用手眼标定矩阵 T_end_cam 和 T_cam_board, 计算机械臂应到达的目标位姿
  3. 驱动机械臂移动到目标位姿, 观察末端是否准确抵达标定板上的预期位置

支持两种运行模式:
  - 录制模式 ( --record ): 在当前相机视角下定位标定板, 将 T_cam_board 保存为模板文件, 作为后续验证的目标值
  - 验证模式 ( 默认 ): 加载已录制的模板 T_cam_board, 循环执行 "回到初始位姿 -> 定位标定板 -> 计算并移动
    到预备位姿 -> 可选迭代细化 -> 移动到最终位姿" 的流程, 以验证手眼标定精度

关键流程 ( 验证模式 ):
  step [1] 初次定位标定板
  step [2] 计算预备位姿 ( 预备位姿在最终位姿上方 2cm, 避免碰撞 )
  step [3] 移动到预备位置
  step [4-N] ( 可选 ) 迭代细化: 重新定位标定板并修正预备位姿, 重复 refine_num 次
  step [6] 从预备位姿沿直线移动到最终位姿

依赖:
  - ROS2 ( rclpy )
  - apriltag2 ( 标定板检测与定位 )
  - 本工程 core 模块 ( arm_wrapper, cam_ros_utils, arm_ros_utils, vision_utils 等 )
"""

import rclpy

import logging
import argparse
import os
import sys
import time
import json
import threading
from typing_extensions import List

import numpy as np

import apriltag2

# 导入本工程的模块

code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    wait_key,
    read_cam_params, read_calib_handeye, inv_tf
)

from core.arm_wrapper import ArmWrapper

from core.arm_ros_utils import TargetArmNode

from core.cam_ros_utils import (
    CamNode,
)

from core.vision_utils import (
    compute_locate_error
)


######################################################### 类定义 #########################################################

class Locator:
    """标定板定位器"""

    def __init__(self,
                 intrinsic: np.ndarray,
                 distortion: np.ndarray,
                 calib_board_info: List[float]):
        self.K = np.array([
            [intrinsic[0], 0.0, intrinsic[2]],
            [0.0, intrinsic[1], intrinsic[3]],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        """内参矩阵 3*3"""

        self.D = None
        """畸变参数, None 表示无畸变, 否则为 numpy 数组, 格式与 OpenCV 的畸变参数格式相同"""
        if distortion is not None and len(distortion) > 0:
            self.D = np.array(distortion, dtype=np.float64)
        # end if

        self.tag3d_list = apriltag2.create_calib_board_3d(
            tag_size=calib_board_info[0],
            space_size=calib_board_info[1],
            rows=calib_board_info[2],
            cols=calib_board_info[3]
        )

        self.detector = apriltag2.Detector(tag_family='tag36h11')
    # end def __init__

    def locate(self, color_img: np.ndarray) -> np.ndarray:
        """
        定位标定板
        Args:
            color_img (np.ndarray): 彩色图像
        Returns:
            T_cam_board (np.ndarray): 相机到标定板的变换矩阵, 4*4 , 如果定位失败返回 None
        """

        tag2d_list = self.detector.detect(color_img, -1.0)
        if len(tag2d_list) == 0:
            logging.warning(f'{YELLOW}no tag detected in the image{RESET}')
            return None
        # end if

        T_cam_board = apriltag2.locate_calib_board(
            tag2d_list=tag2d_list,
            tag3d_list=self.tag3d_list,
            K=self.K,
            D=self.D
        )
        if T_cam_board is None:
            logging.error(f"{RED}Failed to locate calibration board.{RESET}")
            return None
        # end if

        return T_cam_board
    # end def locate

    def locate_via_ros(self, cam_node: CamNode, timeout_sec: float = 5.0) -> np.ndarray:
        """
        通过 ROS2 图像话题定位标定板
        Args:
            cam_node (CamNode): 相机节点
            timeout_sec (float): 超时时间, 单位秒
        Returns:
            T_cam_board (np.ndarray): 相机到标定板的变换矩阵, 4*4 , 如果定位失败返回 None
        """

        # 获取图像
        frame_list = cam_node.get_frames(timeout_sec=timeout_sec)
        if frame_list is None:
            logging.error(f'{RED}failed to get images{RESET}')
            return None
        # end if

        rgb_img = frame_list[0][0]  # 取第一帧第一摄像头图像

        return self.locate(color_img=rgb_img)
    # end def locate_via_ros
# end class Locator


######################################################### 函数定义 #########################################################


def compute_next_pose(T_end_cam: np.ndarray,
                      tmpl_T_cam_board: np.ndarray,
                      cur_T_base_end: np.ndarray,
                      cur_T_cam_board: np.ndarray) -> np.ndarray:
    """
    计算机械臂位姿,使得机械臂从当前位姿移动到设定位姿时, cur_T_cam_board = tmpl_T_cam_board    
    Args:
        T_end_cam (np.ndarray): 从相机到机械臂末端的变换矩阵, 4*4
        tmpl_T_cam_board (np.ndarray): 模板:从标定板到相机的变换矩阵, 4*4
        cur_T_base_end (np.ndarray): 当前从机械臂末端到基座的变换矩阵, 4*4
        cur_T_cam_board (np.ndarray): 当前从标定板到相机的变换矩阵, 4*4
    Returns:
        (np.ndarray): 计算得到的机械臂位姿 T_base_end
    """

    tmpl_T_board_cam = inv_tf(tmpl_T_cam_board)
    T_cam_end = inv_tf(T_end_cam)

    target_T_base_end = cur_T_base_end @ T_end_cam @ cur_T_cam_board @ tmpl_T_board_cam @ T_cam_end

    return target_T_base_end
# end def compute_next_pose


# def verify(T_end_cam: np.ndarray,
#            tmpl_T_cam_board: np.ndarray,
#            locator: Locator,
#            arm: ArmWrapper,
#            cam_node: CamNode,
#            arm_node: TargetArmNode,
#            refine_num: int,
#            debug: bool = False) -> bool:
#     """
#     验证机械臂是否能够从当前位姿移动到目标位姿, 并在移动过程中进行物体跟踪和位姿细化
#     Args:
#         T_end_cam (np.ndarray): 从相机到机械臂末端的变换矩阵, 4*4
#         tmpl_T_cam_board (np.ndarray): 目标值: 从相机到标定板的变换矩阵, 4*4
#         locator (Locator): 标定板定位器
#         arm (ArmWrapper): 机械臂
#         cam_node (CamNode): 相机节点
#         arm_node (TargetArmNode): 机械臂目标节点
#         refine_num (int): 位姿细化次数, 每次细化都会重新定位物体并计算预备位姿
#         debug (bool): 是否启用调试模式, 启用后会在每个步骤等待用户按键确认, 并显示更多日志信息
#     Returns:
#         (bool): 执行结果, False: 任务失败; True: 任务成功
#     """

#     final_T_end_board = T_end_cam @ tmpl_T_cam_board
#     ready_T_end_board = final_T_end_board.copy()
#     ready_T_end_board[2, 3] += 0.02  # 预备位姿比抓取位姿远 2 cm, 避免碰撞

#     T_cam_end = inv_tf(T_end_cam)
#     tmpl_T_cam_board = T_cam_end @ ready_T_end_board

#     # 计算从 ready 位姿到 target 位姿的增量
#     delta_T_end = inv_tf(ready_T_end_board) @ final_T_end_board  # 末端坐标系下的位姿增量
#     logging.info(f'delta_T_end: \n{GREEN}{delta_T_end}{RESET}')

#     show_locate_err = False  # 是否显示定位误差

#     ######## 1. 初次定位标定板 ########
#     print()
#     logging.info(f'step [1], {BLUE}locate calibration board{RESET}')
#     if not wait_key(debug):
#         return False
#     # end if

#     cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)
#     if cur_T_cam_board is None:
#         return False
#     # end if

#     logging.info(f'T_cam_board: \n{GREEN}{cur_T_cam_board}{RESET}')

#     ######## 2. 计算预备位姿 ########
#     print()
#     logging.info(f'step [2], {BLUE}compute ready pose{RESET}')

#     cur_T_base_end = arm.get_pose()
#     target_T_base_end = compute_next_pose(T_end_cam=T_end_cam,
#                                            tmpl_T_cam_board=tmpl_T_cam_board,
#                                            cur_T_base_end=cur_T_base_end,
#                                            cur_T_cam_board=cur_T_cam_board)
#     logging.info(f'ready T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

#     # 发布将要到达的位姿( 用于可视化 )
#     arm_node.publish_pose(target_T_base_end)

#     ######## 3. 移动到预备位置 ########
#     print()
#     logging.info(f'step [3], {BLUE}move to ready pose{RESET}')
#     if not wait_key(debug):
#         return False
#     # end if

#     is_ok = arm.set_pose(target_T_base_end)
#     if not is_ok:
#         logging.error(f"{RED}move arm to ready pose failed.{RESET}")
#         return False
#     # end if

#     # 计算定位偏差
#     if show_locate_err:
#         cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)  # 定位标定板
#         if cur_T_cam_board is None:
#             return False
#         # end if

#         pos_err, rot_err = compute_locate_error(tmpl_T_cam_board, cur_T_cam_board)
#         logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
#     # end if

#     ######## 迭代细化预备位姿 ########
#     refine_cnt = 0
#     while refine_cnt < refine_num:
#         refine_cnt += 1

#         ######## 4. 跟踪物体并计算预备位姿 ########
#         print()
#         logging.info(f'step [4-{refine_cnt}] , {BLUE}track model{RESET}')
#         if not wait_key(debug):
#             return False
#         # end if

#         cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)  # 定位标定板
#         if cur_T_cam_board is None:
#             return False
#         # end if
#         logging.info(f'T_cam_board: \n{GREEN}{cur_T_cam_board}{RESET}')

#         # 计算定位偏差
#         if show_locate_err:
#             pos_err, rot_err = compute_locate_error(tmpl_T_cam_board, cur_T_cam_board)
#             logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
#         # end if

#         # 计算预备位姿
#         cur_T_base_end = arm.get_pose()
#         target_T_base_end = compute_next_pose(T_end_cam=T_end_cam,
#                                                tmpl_T_cam_board=tmpl_T_cam_board,
#                                                cur_T_base_end=cur_T_base_end,
#                                                cur_T_cam_board=cur_T_cam_board)
#         logging.info(f'ready T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

#         # 发布将要到达的位姿( 用于可视化 )
#         arm_node.publish_pose(target_T_base_end)

#         ######## 5. 再次移动到预备位置并计算抓取位姿 ########
#         print()
#         logging.info(f'step [5-{refine_cnt}] , {BLUE}move to ready pose again{RESET}')
#         if not wait_key(debug):
#             return False
#         # end if

#         # 移动到预备位置
#         is_ok = arm.set_pose(target_T_base_end)
#         if not is_ok:
#             logging.error(f"{RED}move arm to ready pose failed.{RESET}")
#             return False
#         # end if

#         logging.info(f'refine count: {refine_cnt}/{refine_num}')
#     # end while   停止迭代细化

#     # 计算定位偏差
#     if show_locate_err and debug:
#         cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)  # 定位标定板
#         if cur_T_cam_board is None:
#             return False
#         # end if
#         logging.info(f'track T_cam_board: \n{GREEN}{cur_T_cam_board}{RESET}')

#         # 计算定位偏差
#         pos_err, rot_err = compute_locate_error(tmpl_T_cam_board, cur_T_cam_board)
#         logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
#     # end if

#     # 计算最终位姿
#     cur_T_base_end = arm.get_pose()
#     target_T_base_end = cur_T_base_end @ delta_T_end
#     logging.info(f'final T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

#     # 发布将要到达的位姿( 用于可视化 )
#     arm_node.publish_pose(target_T_base_end)

#     ######## 6. 抵达 ########
#     print()
#     logging.info(f'step [6] , {BLUE}move to final pose{RESET}')
#     if not wait_key(debug):
#         return False
#     # end if

#     # 移动到抓取位姿
#     is_ok = arm.set_pose(target_T_base_end, move_line=True)
#     if not is_ok:
#         logging.error(f"{RED}move arm to final pose failed.{RESET}")
#         return False
#     # end if

#     return True
# # end def verify


def verify(T_end_cam: np.ndarray,
           tmpl_T_cam_board: np.ndarray,
           locator: Locator,
           arm: ArmWrapper,
           cam_node: CamNode,
           arm_node: TargetArmNode,
           refine_num: int,
           debug: bool = False) -> bool:
    """
    验证机械臂是否能够从当前位姿移动到目标位姿, 并在移动过程中进行物体跟踪和位姿细化
    Args:
        T_end_cam (np.ndarray): 从相机到机械臂末端的变换矩阵, 4*4
        tmpl_T_cam_board (np.ndarray): 模板: 从相机到标定板的变换矩阵, 4*4
        locator (Locator): 标定板定位器
        arm (ArmWrapper): 机械臂
        cam_node (CamNode): 相机节点
        arm_node (TargetArmNode): 机械臂目标节点
        refine_num (int): 位姿细化次数, 每次细化都会重新定位物体并计算预备位姿
        debug (bool): 是否启用调试模式, 启用后会在每个步骤等待用户按键确认, 并显示更多日志信息
    Returns:
        (bool): 执行结果, False: 任务失败; True: 任务成功
    """

    show_locate_err = True  # 是否显示定位误差

    ######## 1. 初次定位标定板 ########
    print()
    logging.info(f'step [1], {BLUE}locate calibration board{RESET}')
    if not wait_key(debug):
        return False
    # end if

    cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)
    if cur_T_cam_board is None:
        return False
    # end if

    logging.info(f'T_cam_board: \n{GREEN}{cur_T_cam_board}{RESET}')

    ######## 2. 计算最终位姿 ########
    print()
    logging.info(f'step [2], {BLUE}compute final pose{RESET}')

    cur_T_base_end = arm.get_pose()
    target_T_base_end = compute_next_pose(T_end_cam=T_end_cam,
                                          tmpl_T_cam_board=tmpl_T_cam_board,
                                          cur_T_base_end=cur_T_base_end,
                                          cur_T_cam_board=cur_T_cam_board)
    logging.info(f'final T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

    # 发布将要到达的位姿( 用于可视化 )
    arm_node.publish_pose(target_T_base_end)

    ######## 3. 移动到最终位置 ########
    print()
    logging.info(f'step [3], {BLUE}move to final pose{RESET}')
    if not wait_key(debug):
        return False
    # end if

    is_ok = arm.set_pose(target_T_base_end)
    if not is_ok:
        logging.error(f"{RED}move arm to final pose failed.{RESET}")
        return False
    # end if

    # 计算定位偏差
    if show_locate_err:
        cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)  # 定位标定板
        if cur_T_cam_board is None:
            return False
        # end if

        pos_err, rot_err = compute_locate_error(tmpl_T_cam_board, cur_T_cam_board)
        logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
    # end if

    ######## 迭代细化最终位姿 ########
    refine_cnt = 0
    while refine_cnt < refine_num:
        refine_cnt += 1

        ######## 4. 定位标定板并计算最终位姿 ########
        print()
        logging.info(f'step [4-{refine_cnt}] , {BLUE}locate calibration board{RESET}')
        if not wait_key(debug):
            return False
        # end if

        cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)  # 定位标定板
        if cur_T_cam_board is None:
            return False
        # end if
        logging.info(f'T_cam_board: \n{GREEN}{cur_T_cam_board}{RESET}')

        # 计算定位偏差
        if show_locate_err:
            pos_err, rot_err = compute_locate_error(tmpl_T_cam_board, cur_T_cam_board)
            logging.info(f'locate error at final pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
        # end if

        # 计算预备位姿
        cur_T_base_end = arm.get_pose()
        target_T_base_end = compute_next_pose(T_end_cam=T_end_cam,
                                              tmpl_T_cam_board=tmpl_T_cam_board,
                                              cur_T_base_end=cur_T_base_end,
                                              cur_T_cam_board=cur_T_cam_board)
        logging.info(f'final T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

        # 发布将要到达的位姿( 用于可视化 )
        arm_node.publish_pose(target_T_base_end)

        ######## 5. 再次移动到最终位姿 ########
        print()
        logging.info(f'step [5-{refine_cnt}] , {BLUE}move to ready pose again{RESET}')
        if not wait_key(debug):
            return False
        # end if

        is_ok = arm.set_pose(target_T_base_end)
        if not is_ok:
            logging.error(f"{RED}move arm to ready pose failed.{RESET}")
            return False
        # end if

        logging.info(f'refine count: {refine_cnt}/{refine_num}')
    # end while

    # 迭代细化后计算定位偏差
    if show_locate_err and debug and refine_cnt > 0:
        cur_T_cam_board = locator.locate_via_ros(cam_node=cam_node)  # 定位标定板
        if cur_T_cam_board is None:
            return False
        # end if
        logging.info(f'track T_cam_board: \n{GREEN}{cur_T_cam_board}{RESET}')

        # 计算定位偏差
        pos_err, rot_err = compute_locate_error(tmpl_T_cam_board, cur_T_cam_board)
        logging.info(f'locate error at final pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
    # end if

    ######## 6. 抵达 ########

    # 沿着末端坐标系 Z 轴方向移动 2 cm 到达最终位姿
    delta_T_end = np.eye(4, dtype=np.float32)
    delta_T_end[2, 3] = 0.023
    cur_T_base_end = arm.get_pose()
    target_T_base_end = cur_T_base_end @ delta_T_end
    logging.info(f'final T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

    # 发布将要到达的位姿( 用于可视化 )
    arm_node.publish_pose(target_T_base_end)

    print()
    logging.info(f'step [6] , {BLUE}move to final pose{RESET}')
    if not wait_key(debug):
        return False
    # end if

    # 移动到抓取位姿
    is_ok = arm.set_pose(target_T_base_end, move_line=False)
    if not is_ok:
        logging.error(f"{RED}move arm to final pose failed.{RESET}")
        return False
    # end if

    return True
# end def verify


def run(init_T_base_end: np.ndarray,
        T_end_cam: np.ndarray,
        tmpl_T_cam_board: np.ndarray,
        locator: Locator,
        arm: ArmWrapper,
        cam_node: CamNode,
        arm_node: TargetArmNode,
        refine_num: int,
        debug: bool = False,
        stop_event: threading.Event = None):
    """
    循环执行验证
    Args:
        T_end_cam (np.ndarray): 从相机到机械臂末端的变换矩阵, 4*4
        tmpl_T_cam_board (np.ndarray): 最终从相机到标定板的变换矩阵, 4*4
        locator (Locator): 定位器
        arm (ArmWrapper): 机械臂包装器
        cam_node (CamNode): 相机节点
        arm_node (TargetArmNode): 机械臂节点
        refine_num (int): 位姿细化次数
        debug (bool): 是否开启调试模式
        stop_event (threading.Event): 停止事件, 正常结束时设置此事件以通知主线程退出 spin 循环
    """

    while rclpy.ok():

        print(f"\n{GREEN}start loop {RESET}")

        logging.info(f"{GREEN}try move arm to init pose...{RESET}")
        if not wait_key(True):
            return False
        # end if

        # is_ok = arm.set_pose(init_T_base_end)
        # if not is_ok:
        #     logging.error(f"{RED}move arm to init pose failed, try again.{RESET}")
        #     break
        # # end if

        is_ok = verify(T_end_cam=T_end_cam,
                       tmpl_T_cam_board=tmpl_T_cam_board,
                       locator=locator,
                       arm=arm,
                       cam_node=cam_node,
                       arm_node=arm_node,
                       refine_num=refine_num,
                       debug=debug)

        if not is_ok:
            logging.error(f"{RED}grasp failed at current loop.{RESET}")
            break
        # end if

        if not wait_key(True):
            break
        # end if

    # end while

    # 机械臂回到零点
    # arm.set_joints(arm.init_joints)

    # 通知主线程停止 spin 循环
    if stop_event is not None:
        stop_event.set()

    logging.info('run finished.')

# end def run


def do_record(locator: Locator,
              cam_node: CamNode,
              tmpl_path: str,
              stop_event: threading.Event = None):
    """
    录制模板
    Args:
        locator (Locator): 定位器
        cam_node (CamNode): 相机节点
        tmpl_path (str): 模板保存路径
        stop_event (threading.Event): 停止事件, 正常结束时设置此事件以通知主线程退出 spin 循环
    """

    time.sleep(5.0)  # 等待机械臂稳定

    # 定位标定板
    T_cam_board = locator.locate_via_ros(cam_node=cam_node)
    if T_cam_board is None:
        logging.error(f"{RED}failed to locate calibration board, cannot record template.{RESET}")
        return
    # end if

    data_dict = {
        'T_cam_board': T_cam_board.tolist()
    }

    tmpl_dir = os.path.dirname(tmpl_path)
    if not os.path.exists(tmpl_dir):
        os.makedirs(tmpl_dir, exist_ok=True)
    # end if

    # 保存模板数据
    with open(tmpl_path, 'w') as f:
        json.dump(data_dict, f, indent=4)
    logging.info(f'saved template to: {GREEN}{tmpl_path}{RESET}')

    # 通知主线程停止 spin 循环
    if stop_event is not None:
        stop_event.set()
    # end if
# end def do_record


######################################################### 主函数 #########################################################


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--arm_index",
                        type=int,
                        default=0,
                        help="机械臂索引, 用于区分多机械臂系统")

    parser.add_argument("--cam_params_path",
                        type=str,
                        required=True,
                        help="相机参数文件的路径, 包含内参和畸变参数")

    parser.add_argument("--calib_handeye_path",
                        type=str,
                        required=True,
                        help="手眼标定文件的路径, 包含相机与机械臂的位姿关系")

    parser.add_argument("--tmpl_path",
                        type=str,
                        required=True,
                        help="保存模板 T_cam_board 的文件路径")

    parser.add_argument("--calib_board_info",
                        type=str,
                        required=True,
                        help="标定板信息 [tag_size, space_size, tag_rows, tag_cols]")

    parser.add_argument("--color_img_topic",
                        type=str,
                        required=True,
                        help="彩色图像的 ROS2 话题名称")

    parser.add_argument("--init_pose",
                        type=str,
                        required=True,
                        help="初始化状态下的末端位姿, 格式[tx,ty,tz,qx,qy,qz,qw], 其中 t 是位移, q 是旋转四元数")

    parser.add_argument("--refine_num",
                        type=int,
                        default=0,
                        help="细化次数, 用于控制优化的迭代次数")

    parser.add_argument("--record",
                        action='store_true',
                        help="是否开启录制模板,此时不会执行验证")

    parser.add_argument("--debug",
                        action='store_true',
                        help="是否开启调试模式")

    args = parser.parse_args()

    arm_index = args.arm_index

    cam_params_path = args.cam_params_path
    calib_handeye_path = args.calib_handeye_path
    tmpl_path = args.tmpl_path
    calib_board_info = json.loads(args.calib_board_info)

    color_img_topic = args.color_img_topic
    if color_img_topic is None:
        logging.error("color_img_topic is not provided.")
        exit(0)
    # end if

    init_pose = json.loads(args.init_pose)
    init_T_base_end = ArmWrapper.array_to_matrix(init_pose)
    refine_num = args.refine_num

    record = args.record
    debug = args.debug

    print()
    print(f'arm index: {BLUE}{arm_index}{RESET}')
    print(f"Camera parameters file: {BLUE}{cam_params_path}{RESET}")
    print(f"handeye calib file: {BLUE}{calib_handeye_path}{RESET}")
    print(f"template file: {BLUE}{tmpl_path}{RESET}")
    print(f"calibration board info: {BLUE}{calib_board_info}{RESET}")
    print(f"color image topic: {BLUE}{color_img_topic}{RESET}")
    print(f"init pose: {BLUE}{init_pose}{RESET}")
    print(f"refine num: {BLUE}{refine_num}{RESET}")
    print(f"record: {BLUE}{record}{RESET}")
    print(f"debug: {BLUE}{debug}{RESET}")
    print()

    # 读取相机参数
    intrinsic, distortion = read_cam_params(cam_params_path)
    if intrinsic is None:
        exit(1)
    # end if

    # 读取手眼标定矩阵
    print()
    T_end_cam, _ = read_calib_handeye(calib_handeye_path)
    if T_end_cam is None:
        exit(1)
    # end if

    # 读取模板 T_cam_board
    if not record:
        try:
            with open(tmpl_path, 'r') as f:
                data = json.load(f)
                T_cam_board = np.array(data['T_cam_board'], dtype=np.float32)
        except Exception as e:
            logging.error(f"failed to load template from {tmpl_path}, error: {e}")
            exit(1)
        logging.info(f"T_cam_board: \n{GREEN}{T_cam_board}{RESET}")
    else:
        T_cam_board = None
    # end if

    print()

    # 创建标定板定位器
    locator = Locator(intrinsic=intrinsic,
                      distortion=distortion,
                      calib_board_info=calib_board_info)

    # 创建机械臂对象
    arm = ArmWrapper(arm_index=arm_index)
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting{RESET}')   # 红色打印
        sys.exit(-1)
    # end if

    # 初始化 ROS2 节点
    rclpy.init(args=None)

    cam_node = CamNode(img_topic_list=[color_img_topic])
    arm_node = TargetArmNode()

    # 线程停止事件: 子线程正常结束时设置此事件, 通知主线程退出 spin 循环
    stop_event = threading.Event()

    # 运行
    if not record:
        thd_run = threading.Thread(target=run,
                                   args=(init_T_base_end,
                                         T_end_cam,
                                         T_cam_board,
                                         locator,
                                         arm,
                                         cam_node,
                                         arm_node,
                                         refine_num,
                                         debug,
                                         stop_event
                                         )  # 传入停止事件
                                   )
    else:
        thd_run = threading.Thread(target=do_record,
                                   args=(locator,
                                         cam_node,
                                         tmpl_path)
                                   )
    # end if
    thd_run.start()

    try:
        # 使用 spin_once 循环代替 rclpy.spin, 以便周期性检查子线程是否结束
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(cam_node, timeout_sec=0.1)
        # end while

    except KeyboardInterrupt:
        logging.warning('interrupted by user (Ctrl+C)')
    finally:
        logging.info('shutting down...')

        # # 1. 机械臂回到初始位置
        # try:
        #     arm.set_joints(arm.init_joints)
        # except Exception as e:
        #     logging.warning(f'arm set_joints to init failed: {e}')

        # 2. 等待抓取子线程结束
        thd_run.join(timeout=10.0)
        if thd_run.is_alive():
            logging.warning('run thread still alive after timeout, force proceeding.')

        # 3. 恢复机械臂速度等级并断开连接( 从 __del__ 提前到显式调用, 确保确定性清理 )
        try:
            arm.set_speed_level(arm.init_speed_level)
        except Exception:
            pass
        try:
            arm.arm.disconnect()
            logging.info('Arm disconnected.')
        except Exception:
            pass

        # 4. 销毁 ROS2 节点
        cam_node.destroy_node()
        arm_node.destroy_node()

        # 5. 关闭 rclpy( 加保护避免 SIGINT handler 已抢先 shutdown 导致重复调用报错 )
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        logging.info('shutdown complete.')

    # end try

# end if __name__ == '__main__'
