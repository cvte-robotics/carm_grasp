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

# RGB-D 相机参数文件路径
cam_params_path="${root_dir}/demo/data/calib/g305/cam_params.json"
# cam_params_path="${root_dir}/data/calib/cam_params.json" 

# 手眼标定文件路径
calib_handeye_path="${root_dir}/demo/data/calib/g305/calib_handeye.json"
# calib_handeye_path="${root_dir}/data/calib/calib_handeye.json"

# RGB-D 图像话题名称
color_img_topic="/gemini305/color/image_raw"
depth_img_topic="/gemini305/depth/image_raw"
# color_img_topic="/realsense/d405/color/image_rect_raw"
# depth_img_topic="/realsense/d405/aligned_depth_to_color/image_raw"

# 模板文件的目录
tmpl_dir="${root_dir}/demo/data/benchmark/g305/grasp_3d"
# tmpl_dir="${root_dir}/data/benchmark/tmpl/grasp_3d"  


############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/create_tmpl_grasp_3d.py \
    --arm_index ${arm_index} \
    --cam_params_path ${cam_params_path} \
    --calib_handeye_path ${calib_handeye_path} \
    --color_img_topic ${color_img_topic} \
    --depth_img_topic ${depth_img_topic} \
    --tmpl_dir ${tmpl_dir}
