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
arm_index=0

calib_handeye_path="${root_dir}/demo/data/calib/d405/calib_handeye.json"

tmpl_dir="${root_dir}/data/action/collect_dataset"


############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/create_collect_actions.py \
  --arm_index ${arm_index} \
  --calib_handeye_path "${calib_handeye_path}" \
  --tmpl_dir "${tmpl_dir}" \
  --target_point 0.3 0 0 \
  --radius 0.35 0.25 \
  --max_angle_deg 30 \
  --num_polar 3 \
  --num_azimuth 8 \
  --overwrite \
  --visualize
