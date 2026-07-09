# -*- coding: utf-8 -*-
"""
功能说明: 生成自动采集用的机械臂行动模板.

生成规则:
    1. 相机光心位于以 target_point 为球心、一个或多个 radius 为半径的半球面上;
    2. 相机光心位于机械臂基座坐标系 view_axis 方向的半球面上;
    3. 相机坐标系 Z 轴始终指向 target_point;
    4. 相机光心方向与 view_axis 的夹角不超过 max_angle_deg;
    5. 相机 O-X/O-Y 轴在基座 XOY 平面上的投影方向为相机光心投影点指向 target_point 投影点;
       当相机正好在 target_point 正上方时,该投影方向为 target_point 投影点指向基座坐标系原点;
    6. 在基座 XOY 平面中,若角 O-T-C 小于 min_otc_angle_deg,则剔除该相机位姿;
    7. 将相机位姿 T_base_cam 通过手眼标定 T_end_cam 转换为末端位姿 T_base_end;
    8. 使用 ArmWrapper.inverse_kinematics 剔除不可达位姿,并保存为 action 模板 JSON.
    9. 可选使用 Open3D 可视化所有可达末端位姿和对应相机视线.
"""

import argparse
import json
import logging
import os
import sys

from typing_extensions import List, Tuple

import numpy as np
import open3d


# 导入本工程的模块
code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    inv_tf, read_calib_handeye,
)
from core.arm_wrapper import ArmWrapper


######################################################### 函数定义 #########################################################

def normalize_vector(vector: np.ndarray,
                     name: str) -> np.ndarray:
    """
    归一化三维向量.
    Args:
        vector (np.ndarray): 输入向量,形状为 (3,)
        name (str): 向量名称,用于错误提示
    Returns:
        (np.ndarray): 单位向量
    """

    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    vector_norm = np.linalg.norm(vector)
    if vector_norm < 1e-9:
        raise ValueError(f'{name} norm is too small')
    # end if

    return vector / vector_norm
# end def normalize_vector


def get_orthogonal_unit_vector(axis: np.ndarray) -> np.ndarray:
    """
    获取与 axis 垂直的单位向量.
    Args:
        axis (np.ndarray): 输入单位轴,形状为 (3,)
    Returns:
        (np.ndarray): 与 axis 垂直的单位向量
    """

    axis = normalize_vector(axis, 'axis')
    candidate_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(np.dot(axis, candidate_axis)) > 0.9:
        candidate_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    # end if

    orthogonal_axis = candidate_axis - np.dot(candidate_axis, axis) * axis
    return normalize_vector(orthogonal_axis, 'orthogonal_axis')
# end def get_orthogonal_unit_vector


def get_project_axis_dir(target_point: np.ndarray,
                         camera_position: np.ndarray) -> np.ndarray:
    """
    获取相机指定轴在基座 XOY 平面上的期望投影方向.

    Args:
        target_point (np.ndarray): 基座坐标系下的目标点,形状为 (3,)
        camera_position (np.ndarray): 基座坐标系下的相机光心,形状为 (3,)
    Returns:
        (np.ndarray): XOY 平面内的单位方向向量,形状为 (3,)
    """

    target_point = np.asarray(target_point, dtype=np.float64).reshape(3)
    camera_position = np.asarray(camera_position, dtype=np.float64).reshape(3)

    project_axis_dir = target_point - camera_position
    project_axis_dir[2] = 0.0
    if np.linalg.norm(project_axis_dir) < 1e-9:
        project_axis_dir = -target_point
        project_axis_dir[2] = 0.0
    # end if
    if np.linalg.norm(project_axis_dir) < 1e-9:
        logging.warning(f'{YELLOW}skip camera pose because projection direction is undefined, target_point: {target_point}{RESET}')
        return None
    # end if

    return normalize_vector(project_axis_dir, 'project_axis_dir')
# end def get_project_axis_dir


def build_axis_with_xoy_projection(camera_z_axis: np.ndarray,
                                   project_axis_dir: np.ndarray,
                                   project_axis: str) -> np.ndarray:
    """
    构造与 camera_z_axis 正交、且 XOY 投影方向为 project_axis_dir 的相机 X/Y 轴.

    Args:
        camera_z_axis (np.ndarray): 相机 Z 轴在基座坐标系下的方向,形状为 (3,)
        project_axis_dir (np.ndarray): XOY 平面内的目标投影方向,形状为 (3,)
        project_axis (str): 被约束的相机轴,可选 'x' 或 'y'
    Returns:
        (np.ndarray): 被约束的相机轴在基座坐标系下的单位方向. 不可满足时返回 None
    """

    camera_z_axis = normalize_vector(camera_z_axis, 'camera_z_axis')
    project_axis_dir = normalize_vector(project_axis_dir, 'project_axis_dir')

    if abs(camera_z_axis[2]) < 1e-9:
        logging.warning(
            f'{YELLOW}skip camera pose because camera Z axis is parallel to XOY plane and camera {project_axis.upper()} projection constraint is unsolvable{RESET}')
        return None
    # end if

    axis = project_axis_dir.copy()
    axis[2] = -np.dot(project_axis_dir, camera_z_axis) / camera_z_axis[2]
    return normalize_vector(axis, f'camera_{project_axis}_axis')
# end def build_axis_with_xoy_projection


def build_camera_lookat_pose(target_point: np.ndarray,
                             camera_position: np.ndarray,
                             project_axis: str = 'x') -> np.ndarray:
    """
    根据相机光心和观察方向构造相机位姿.

    Args:
        target_point (np.ndarray): 基座坐标系下的目标点,形状为 (3,)
        camera_position (np.ndarray): 基座坐标系下的相机光心,形状为 (3,)
        project_axis (str): 将 XOY 平面投影方向对齐到相机的哪个轴,可选 'x' 或 'y'
    Returns:
        (np.ndarray): 从相机坐标系到基座坐标系的位姿 T_base_cam,形状为 (4,4). 不可满足投影约束时返回 None
    """

    target_point = np.asarray(target_point, dtype=np.float64).reshape(3)
    camera_position = np.asarray(camera_position, dtype=np.float64).reshape(3)
    view_dir_base = normalize_vector(target_point - camera_position, 'view_dir_base')

    camera_z_axis = view_dir_base

    project_axis = project_axis.lower()
    if project_axis not in ['x', 'y']:
        raise ValueError("project_axis must be 'x' or 'y'")
    # end if

    project_axis_dir = get_project_axis_dir(target_point=target_point,
                                            camera_position=camera_position)
    if project_axis_dir is None:
        return None
    # end if

    if project_axis == 'x':
        camera_x_axis = build_axis_with_xoy_projection(camera_z_axis=camera_z_axis,
                                                       project_axis_dir=project_axis_dir,
                                                       project_axis=project_axis)
        if camera_x_axis is None:
            return None
        # end if
        camera_y_axis = normalize_vector(np.cross(camera_z_axis, camera_x_axis), 'camera_y_axis')
    else:
        camera_y_axis = build_axis_with_xoy_projection(camera_z_axis=camera_z_axis,
                                                       project_axis_dir=project_axis_dir,
                                                       project_axis=project_axis)
        if camera_y_axis is None:
            return None
        # end if
        camera_x_axis = normalize_vector(np.cross(camera_y_axis, camera_z_axis), 'camera_x_axis')
    # end if

    T_base_cam = np.eye(4, dtype=np.float64)
    T_base_cam[:3, :3] = np.column_stack((camera_x_axis, camera_y_axis, camera_z_axis))
    T_base_cam[:3, 3] = camera_position

    check_view_dir = normalize_vector(target_point - camera_position, 'check_view_dir')
    if np.dot(check_view_dir, camera_z_axis) < 1.0 - 1e-6:
        raise ValueError('camera Z axis does not point to target_point')
    # end if

    check_axis = camera_x_axis if project_axis == 'x' else camera_y_axis
    check_axis_proj = check_axis.copy()
    check_axis_proj[2] = 0.0
    check_axis_proj = normalize_vector(check_axis_proj, 'check_axis_proj')
    if np.dot(check_axis_proj, project_axis_dir) < 1.0 - 1e-6:
        raise ValueError(f'camera {project_axis.upper()} axis projection does not match target projection direction')
    # end if

    return T_base_cam
# end def build_camera_lookat_pose


def compute_xoy_angle_deg(vertex_point: np.ndarray,
                          point_a: np.ndarray,
                          point_b: np.ndarray) -> float:
    """
    计算基座 XOY 平面中的夹角 A-vertex-B.

    Args:
        vertex_point (np.ndarray): 夹角顶点,形状为 (3,)
        point_a (np.ndarray): 夹角第一条边上的点,形状为 (3,)
        point_b (np.ndarray): 夹角第二条边上的点,形状为 (3,)
    Returns:
        (float): 夹角,单位: 度. 如果夹角无法定义则返回 None
    """

    vertex_xy = np.asarray(vertex_point, dtype=np.float64).reshape(3)[:2]
    point_a_xy = np.asarray(point_a, dtype=np.float64).reshape(3)[:2]
    point_b_xy = np.asarray(point_b, dtype=np.float64).reshape(3)[:2]

    vec_a = point_a_xy - vertex_xy
    vec_b = point_b_xy - vertex_xy
    norm_a = np.linalg.norm(vec_a)
    norm_b = np.linalg.norm(vec_b)
    if norm_a < 1e-9 or norm_b < 1e-9:
        return None
    # end if

    cosine = np.dot(vec_a, vec_b) / (norm_a * norm_b)
    cosine = np.clip(cosine, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))
# end def compute_xoy_angle_deg


def should_skip_by_otc_angle(target_point: np.ndarray,
                             camera_position: np.ndarray,
                             min_otc_angle_deg: float) -> bool:
    """
    判断相机光心是否应按角 O-T-C 过小的规则剔除.

    Args:
        target_point (np.ndarray): 基座坐标系下目标点 T,形状为 (3,)
        camera_position (np.ndarray): 基座坐标系下相机光心,形状为 (3,)
        min_otc_angle_deg (float): 最小允许角 O-T-C,单位: 度
    Returns:
        (bool): True 表示需要剔除该位姿
    """

    base_origin = np.zeros(3, dtype=np.float64)
    angle_deg = compute_xoy_angle_deg(vertex_point=target_point,
                                      point_a=base_origin,
                                      point_b=camera_position)
    if angle_deg is None:
        return False
    # end if

    return angle_deg < min_otc_angle_deg
# end def should_skip_by_otc_angle


def generate_camera_lookat_poses(target_point: np.ndarray,
                                 radius: float,
                                 max_angle_deg: float,
                                 num_polar: int,
                                 num_azimuth: int,
                                 view_axis: np.ndarray,
                                 project_axis: str = 'x',
                                 min_otc_angle_deg: float = 40.0) -> List[np.ndarray]:
    """
    在 view_axis 方向的球面帽上采样相机位姿,使相机 Z 轴始终指向 target_point.

    Args:
            target_point (np.ndarray): 基座坐标系下的目标点,形状为 (3,)
            radius (float): 相机光心到 target_point 的距离,单位: 米
            max_angle_deg (float): 相机光心方向与 view_axis 的最大夹角,单位: 度
            num_polar (int): 极角采样数量,包含 0 和 max_angle_deg
            num_azimuth (int): 每个非零极角上的方位角采样数量
            view_axis (np.ndarray): 基座坐标系下用于约束相机光心半球方向的参考轴,默认基座 +Z 轴
            project_axis (str): 将 XOY 平面投影方向对齐到相机的哪个轴,可选 'x' 或 'y'
                min_otc_angle_deg (float): 基座 XOY 平面中角 O-T-C 的最小允许值,单位: 度
    Returns:
            (List[np.ndarray]): 相机候选位姿列表 T_base_cam
    """

    if radius <= 0:
        raise ValueError('radius must be positive')
    # end if
    if num_polar < 1:
        raise ValueError('num_polar must be >= 1')
    # end if
    if num_azimuth < 1:
        raise ValueError('num_azimuth must be >= 1')
    # end if
    if max_angle_deg < 0 or max_angle_deg > 90:
        raise ValueError('max_angle_deg must be in [0, 90] for hemisphere sampling')
    # end if
    if min_otc_angle_deg < 0 or min_otc_angle_deg > 180:
        raise ValueError('min_otc_angle_deg must be in [0, 180]')
    # end if

    target_point = np.asarray(target_point, dtype=np.float64).reshape(3)
    view_axis = normalize_vector(view_axis, 'view_axis')
    tangent_x_axis = get_orthogonal_unit_vector(view_axis)
    tangent_y_axis = normalize_vector(np.cross(view_axis, tangent_x_axis), 'tangent_y_axis')

    max_angle = np.deg2rad(max_angle_deg)
    polar_angles = np.linspace(0.0, max_angle, num_polar)

    T_base_cam_list = []
    for polar_angle in polar_angles:
        if abs(polar_angle) < 1e-9:
            azimuth_angles = [0.0]
        else:
            azimuth_angles = np.linspace(0.0, 2.0 * np.pi, num_azimuth, endpoint=False)
        # end if

        for azimuth_angle in azimuth_angles:
            tangent_dir = (np.cos(azimuth_angle) * tangent_x_axis +
                           np.sin(azimuth_angle) * tangent_y_axis)
            camera_dir_base = (np.cos(polar_angle) * view_axis +
                               np.sin(polar_angle) * tangent_dir)
            camera_dir_base = normalize_vector(camera_dir_base, 'camera_dir_base')
            camera_position = target_point + radius * camera_dir_base
            if should_skip_by_otc_angle(target_point=target_point,
                                        camera_position=camera_position,
                                        min_otc_angle_deg=min_otc_angle_deg):
                continue
            # end if

            T_base_cam = build_camera_lookat_pose(target_point=target_point,
                                                  camera_position=camera_position,
                                                  project_axis=project_axis)
            if T_base_cam is None:
                continue
            # end if
            T_base_cam_list.append(T_base_cam)
        # end for
    # end for

    return T_base_cam_list
# end def generate_camera_lookat_poses


def convert_camera_to_arm_poses(T_base_cam_list: List[np.ndarray],
                                T_end_cam: np.ndarray) -> List[np.ndarray]:
    """
    将相机位姿转换为机械臂末端位姿.

    已知 T_base_cam = T_base_end @ T_end_cam,
    因此 T_base_end = T_base_cam @ inv(T_end_cam).

    Args:
            T_base_cam_list (List[np.ndarray]): 相机候选位姿列表
            T_end_cam (np.ndarray): 从相机坐标系到机械臂末端坐标系的手眼标定矩阵,形状为 (4,4)
    Returns:
            (List[np.ndarray]): 机械臂末端候选位姿列表
    """

    assert T_end_cam.shape == (4, 4), 'T_end_cam must be 4x4'

    T_cam_end = inv_tf(T_end_cam)
    T_base_end_list = []
    for T_base_cam in T_base_cam_list:
        T_base_end = T_base_cam @ T_cam_end
        T_base_end_list.append(T_base_end)
    # end for

    return T_base_end_list
# end def convert_camera_to_arm_poses


def filter_reachable_poses(arm: ArmWrapper,
                           T_base_end_list: List[np.ndarray],
                           T_base_cam_list: List[np.ndarray]) -> Tuple[List[np.ndarray], List[np.ndarray], List[List[float]], List[int]]:
    """
    使用 ArmWrapper.inverse_kinematics 过滤不可达末端位姿.

    Args:
            arm (ArmWrapper): 机械臂对象
            T_base_end_list (List[np.ndarray]): 机械臂末端候选位姿列表
            T_base_cam_list (List[np.ndarray]): 相机候选位姿列表
    Returns:
            (Tuple[List[np.ndarray], List[np.ndarray], List[List[float]], List[int]]):
                    可达末端位姿、可达相机位姿、对应关节角、原始候选索引
    """

    assert len(T_base_end_list) == len(T_base_cam_list), 'pose list length mismatch'

    joints_list = arm.inverse_kinematics(T_base_end_list)

    reachable_T_base_end_list = []
    reachable_T_base_cam_list = []
    reachable_joints_list = []
    reachable_idx_list = []
    for pose_idx, joints in enumerate(joints_list):
        if joints is None or len(joints) == 0:
            continue
        # end if

        joints = [float(v) for v in joints]
        reachable_T_base_end_list.append(T_base_end_list[pose_idx])
        reachable_T_base_cam_list.append(T_base_cam_list[pose_idx])
        reachable_joints_list.append(joints)
        reachable_idx_list.append(pose_idx)
    # end for

    return reachable_T_base_end_list, reachable_T_base_cam_list, reachable_joints_list, reachable_idx_list
# end def filter_reachable_poses


def save_action_templates(tmpl_dir: str,
                          T_base_end_list: List[np.ndarray],
                          T_base_cam_list: List[np.ndarray],
                          joints_list: List[List[float]],
                          src_idx_list: List[int],
                          gripper_dist: float,
                          overwrite: bool = False) -> None:
    """
    保存可达行动模板.

    Args:
        tmpl_dir (str): 模板输出目录
        T_base_end_list (List[np.ndarray]): 可达末端位姿列表
        T_base_cam_list (List[np.ndarray]): 可达相机位姿列表
        joints_list (List[List[float]]): 可达关节角列表
        src_idx_list (List[int]): 原始候选索引列表
        gripper_dist (float): 夹爪距离
        overwrite (bool): 是否覆盖已有 JSON 文件
    """

    os.makedirs(tmpl_dir, exist_ok=True)

    for action_idx, (T_base_end, T_base_cam, joints, src_idx) in enumerate(zip(T_base_end_list, T_base_cam_list, joints_list, src_idx_list)):
        file_path = os.path.join(tmpl_dir, f'{action_idx}.json')
        if os.path.exists(file_path) and not overwrite:
            raise FileExistsError(f'action file already exists: {file_path}, use --overwrite to replace it')
        # end if

        action_dict = {
            'T_base_end': T_base_end.tolist(),
            'joints': joints,
            'gripper_dist': float(gripper_dist),
            'T_base_cam': T_base_cam.tolist(),
            'src_candidate_idx': int(src_idx),
        }
        with open(file_path, 'w') as f:
            json.dump(action_dict, f, indent=4)
        # end with

        logging.info(f'saved reachable action [{action_idx}] from candidate [{src_idx}] to: {GREEN}{file_path}{RESET}')
    # end for
# end def save_action_templates


def visualize_cam_poses(pose_list: List[np.ndarray],
                        target_point: np.ndarray,
                        frame_size: float = 0.04) -> None:
    """
    使用 Open3D 可视化所有相机位姿和对应相机视线.

    Args:
        pose_list (List[np.ndarray]): 位姿列表
        target_point (np.ndarray): 基座坐标系下的目标点,形状为 (3,)
        frame_size (float): 坐标系显示尺寸,单位: 米
    """

    assert len(pose_list) > 0, 'pose list is empty'

    target_point = np.asarray(target_point, dtype=np.float64).reshape(3)
    geometry_list = []

    base_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 2.0)
    geometry_list.append(base_frame)

    target_sphere = open3d.geometry.TriangleMesh.create_sphere(radius=frame_size * 0.35)
    target_sphere.paint_uniform_color([1.0, 0.0, 0.0])
    target_sphere.translate(target_point)
    geometry_list.append(target_sphere)

    line_points = [target_point.tolist()]
    line_indices = []
    line_colors = []
    cam_center_points = []

    for pose_idx, T_base_cam in enumerate(pose_list):
        cam_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 0.75)
        cam_frame.transform(T_base_cam)
        geometry_list.append(cam_frame)

        cam_center = T_base_cam[:3, 3]
        cam_center_points.append(cam_center.tolist())

        line_points.append(cam_center.tolist())
        line_indices.append([0, pose_idx + 1])
        line_colors.append([0.0, 0.7, 1.0])
    # end for

    if len(cam_center_points) > 0:
        cam_center_pc = open3d.geometry.PointCloud()
        cam_center_pc.points = open3d.utility.Vector3dVector(np.array(cam_center_points, dtype=np.float64))
        cam_center_pc.paint_uniform_color([0.0, 0.35, 1.0])
        geometry_list.append(cam_center_pc)

        view_line_set = open3d.geometry.LineSet()
        view_line_set.points = open3d.utility.Vector3dVector(np.array(line_points, dtype=np.float64))
        view_line_set.lines = open3d.utility.Vector2iVector(np.array(line_indices, dtype=np.int32))
        view_line_set.colors = open3d.utility.Vector3dVector(np.array(line_colors, dtype=np.float64))
        geometry_list.append(view_line_set)
    # end if

    logging.info(f'Open3D visualize poses: {GREEN}{len(pose_list)}{RESET}')
    open3d.visualization.draw_geometries(geometry_list,
                                         window_name='camera poses',
                                         width=1280,
                                         height=720)
# end def visualize_cam_poses


def visualize_arm_poses(pose_list: List[np.ndarray],
                        frame_size: float = 0.04) -> None:
    """
    使用 Open3D 可视化所有机械臂位姿和对应机械臂视线.

    Args:
        pose_list (List[np.ndarray]): 位姿列表
        frame_size (float): 坐标系显示尺寸,单位: 米
    """

    assert len(pose_list) > 0, 'pose list is empty'

    geometry_list = []

    base_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 2.0)
    geometry_list.append(base_frame)

    for pose_idx, T in enumerate(pose_list):
        frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 0.75)
        frame.transform(T)
        geometry_list.append(frame)
    # end for

    logging.info(f'Open3D visualize poses: {GREEN}{len(pose_list)}{RESET}')
    open3d.visualization.draw_geometries(geometry_list,
                                         window_name='arm poses',
                                         width=1280,
                                         height=720)
# end def visualize_arm_poses


def run(args: argparse.Namespace) -> None:
    """
    生成并保存行动模板.
    Args:
        args (argparse.Namespace): 命令行参数
    """

    T_end_cam, eye_in_hand = read_calib_handeye(args.calib_handeye_path)
    if T_end_cam is None or not eye_in_hand:
        logging.warning(f'{RED}hand-eye calibration must contain T_armend_cam for eye-in-hand camera{RESET}')
        T_end_cam = np.eye(4, dtype=np.float64)
    # end if

    target_point = np.array(args.target_point, dtype=np.float64)
    view_axis = np.array(args.view_axis, dtype=np.float64)
    radius_list = [float(radius) for radius in args.radius]

    T_base_cam_list = []
    for radius in radius_list:
        radius_T_base_cam_list = generate_camera_lookat_poses(target_point=target_point,
                                                              radius=radius,
                                                              max_angle_deg=args.max_angle_deg,
                                                              num_polar=args.num_polar,
                                                              num_azimuth=args.num_azimuth,
                                                              view_axis=view_axis,
                                                              project_axis=args.project_axis,
                                                              min_otc_angle_deg=args.min_otc_angle_deg)
        logging.info(f'generated candidates with radius {GREEN}{radius:.3f}{RESET}: {GREEN}{len(radius_T_base_cam_list)}{RESET}')
        T_base_cam_list.extend(radius_T_base_cam_list)
    # end for

    if len(T_base_cam_list) == 0:
        logging.warning(f'{YELLOW}no camera pose candidate generated, nothing saved{RESET}')
        return
    # end if

    T_base_end_list = convert_camera_to_arm_poses(T_base_cam_list=T_base_cam_list,
                                                  T_end_cam=T_end_cam)

    logging.info(f'generated candidates: {GREEN}{len(T_base_end_list)}{RESET}')

    if args.visualize:
        visualize_cam_poses(pose_list=T_base_cam_list,
                            target_point=target_point,
                            frame_size=args.vis_frame_size)
    # end if

    arm = ArmWrapper(ip=args.arm_ip, speed_level=args.speed_level)
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting{RESET}')
        sys.exit(1)
    # end if

    reachable_T_base_end_list, reachable_T_base_cam_list, reachable_joints_list, reachable_idx_list = filter_reachable_poses(
        arm=arm,
        T_base_end_list=T_base_end_list,
        T_base_cam_list=T_base_cam_list,
    )
    logging.info(f'reachable candidates: {GREEN}{len(reachable_T_base_end_list)}{RESET}/{len(T_base_end_list)}')

    if len(reachable_T_base_end_list) == 0:
        logging.warning(f'{YELLOW}no reachable pose found, nothing saved{RESET}')
        return
    # end if

    if args.visualize:
        visualize_arm_poses(pose_list=reachable_T_base_end_list,
                            frame_size=args.vis_frame_size)
    # end if

    save_action_templates(tmpl_dir=args.tmpl_dir,
                          T_base_end_list=reachable_T_base_end_list,
                          T_base_cam_list=reachable_T_base_cam_list,
                          joints_list=reachable_joints_list,
                          src_idx_list=reachable_idx_list,
                          gripper_dist=args.gripper_dist,
                          overwrite=args.overwrite)
# end def run


######################################################### 主函数 #########################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('--calib_handeye_path', type=str, required=True,
                        help='手眼标定文件路径,必须包含 T_armend_cam')

    parser.add_argument('--tmpl_dir', type=str, required=True,
                        help='保存 action 模板 JSON 的目录')

    parser.add_argument('--target_point', type=float, nargs=3, default=[0.3, 0.0, 0.0],
                        help='基座坐标系下相机需要指向的点,默认: 0.3 0 0')

    parser.add_argument('--radius', type=float, nargs='+', default=[0.35],
                        help='相机光心到 target_point 的球面半径,单位: 米,可设置多个值')

    parser.add_argument('--max_angle_deg', type=float, default=45.0,
                        help='相机光心方向与 view_axis 的最大夹角,单位: 度')

    parser.add_argument('--num_polar', type=int, default=9,
                        help='极角方向采样数量,包含中心方向和最大夹角边界')

    parser.add_argument('--num_azimuth', type=int, default=8,
                        help='每个非零极角圆环上的方位采样数量')

    parser.add_argument('--view_axis', type=float, nargs=3, default=[0.0, 0.0, 1.0],
                        help='基座坐标系下相机光心所在半球的参考轴,默认基座 +Z')

    parser.add_argument('--project_axis', type=str, choices=['x', 'y'], default='x',
                        help='选择相机 O-X 或 O-Y 轴的 XOY 平面投影方向对齐到目标投影方向')

    parser.add_argument('--min_otc_angle_deg', type=float, default=40.0,
                        help='基座 XOY 平面中角 O-T-C 的最小允许值,小于该角度的相机位姿会被剔除')

    parser.add_argument('--gripper_dist', type=float, default=0.08,
                        help='写入 action 模板的夹爪距离,单位: 米')

    parser.add_argument('--arm_ip', type=str, default='10.42.0.101',
                        help='机械臂 IP 地址')

    parser.add_argument('--speed_level', type=int, default=50,
                        help='机械臂速度等级')

    parser.add_argument('--overwrite', action='store_true',
                        help='允许覆盖 tmpl_dir 下已有的同名 JSON 文件')

    parser.add_argument('--visualize', action='store_true',
                        help='使用 Open3D 可视化所有逆解可达的末端位姿和相机视线')

    parser.add_argument('--vis_frame_size', type=float, default=0.04,
                        help='Open3D 可视化中的坐标系尺寸,单位: 米')

    args = parser.parse_args()

    print()
    print(f'calib_handeye_path: {BLUE}{args.calib_handeye_path}{RESET}')
    print(f'tmpl_dir: {BLUE}{args.tmpl_dir}{RESET}')
    print(f'target_point: {BLUE}{args.target_point}{RESET}')
    print(f'radius: {BLUE}{args.radius}{RESET}')
    print(f'max_angle_deg: {BLUE}{args.max_angle_deg}{RESET}')
    print(f'num_polar: {BLUE}{args.num_polar}{RESET}')
    print(f'num_azimuth: {BLUE}{args.num_azimuth}{RESET}')
    print(f'view_axis: {BLUE}{args.view_axis}{RESET}')
    print(f'project_axis: {BLUE}{args.project_axis}{RESET}')
    print(f'min_otc_angle_deg: {BLUE}{args.min_otc_angle_deg}{RESET}')
    print(f'gripper_dist: {BLUE}{args.gripper_dist}{RESET}')
    print(f'visualize: {BLUE}{args.visualize}{RESET}')
    print()

    run(args)
# end if __name__ == '__main__'
