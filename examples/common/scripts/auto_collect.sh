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

# 机械臂动作模板文件夹路径
tmpl_dir="${root_dir}/data/common/calib_handeye"

# 相机图像话题列表
img_topic_list=(
    "/realsense/d405/color/image_rect_raw"
    "/realsense/d405/aligned_depth_to_color/image_raw"
)

# 数据保存路径
data_dir="${root_dir}/results/auto_collect/calib_handeye"


############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/auto_collect.py \
    --tmpl_dir "${tmpl_dir}" \
    --img_topic_list "${img_topic_list[@]}" \
    --data_dir "${data_dir}" \
    # --debug
