#!/bin/bash

source /opt/ros/foxy/setup.bash     # Ubuntu 20.04 使用 foxy
source /opt/ros/humble/setup.bash   # Ubuntu 22.04 使用 humble
export ROS_DOMAIN_ID=1              # 设置 ROS_DOMAIN_ID,确保与其他设备不冲突

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"

root_dir="$(realpath "${script_dir}/../../../")"
echo "项目根目录: $root_dir"
echo


############################################## 参数配置 ##############################################

# 机械臂索引, 用于区分多机械臂系统
arm_index=1

# 机械臂手眼标定结果文件路径
# calib_handeye_path="${root_dir}/demo/data/calib/g305/calib_handeye.json"

# 夹爪参数文件路径
# gripper_path="${root_dir}/demo/data/calib/g305/gripper_body.json"

# 机械臂所在的坐标系名称
frame_id="base_link"
# frame_id="odom_frame"

# 相机点云所在的坐标系名称
# pc_frame_id="d405_depth_optical_frame"


############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/arm_node.py \
    --arm_index ${arm_index} \
    --calib_handeye_path "${calib_handeye_path}" \
    --gripper_path "${gripper_path}" \
    --frame_id "${frame_id}" \
    --pc_frame_id "${pc_frame_id}"
