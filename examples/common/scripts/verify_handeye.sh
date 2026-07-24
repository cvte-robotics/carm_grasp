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

# RGB-D 相机参数文件路径
# cam_params_path="${root_dir}/demo/data/calib/g305/cam_params.json"
# cam_params_path="${root_dir}/data/calib/cam_params.json"
cam_params_path="/home/user/Work/Public/carm_grasp/tmp/auto_collect/d405/calib_handeye/0000/cam_params.json"

# 手眼标定文件路径
# calib_handeye_path="${root_dir}/demo/data/calib/g305/calib_handeye.json"
# calib_handeye_path="${root_dir}/data/calib/calib_handeye.json"
calib_handeye_path="/home/user/Work/Public/carm_grasp/tmp/auto_collect/d405/calib_handeye/0000/calib_handeye.json"

# 模板文件路径 ( 保存或加载 T_cam_board )
# tmpl_path="${root_dir}/data/calib/verify_handeye_tmpl.json"
tmpl_path="/home/user/Work/Public/carm_grasp/tmp/data/calib/verify_handeye_tmpl.json"

# 标定板信息: [tag_size( m ), space_size( m ), tag_rows, tag_cols]
calib_board_info='[0.0352, 0.01056, 6, 6]'

# 彩色图像话题名称
color_img_topic="/realsense/d405/color/image_rect_raw"
# color_img_topic="/gemini305/color/image_raw"

# 初始化状态下的末端位姿 [tx, ty, tz, qx, qy, qz, qw]
init_pose="[0.11283640, -0.37894820, -0.33809095, -0.94943114, -0.26543730, -0.01121556, 0.16732533]"

# 细化次数, 每次细化都会重新定位物体并计算预备位姿
refine_num=0


############################################## 可执行程序 ##############################################

# --record 
# --debug

python3 ${script_dir}/../src/verify_handeye.py \
    --arm_index ${arm_index} \
    --cam_params_path "${cam_params_path}" \
    --calib_handeye_path "${calib_handeye_path}" \
    --tmpl_path "${tmpl_path}" \
    --calib_board_info "${calib_board_info}" \
    --color_img_topic "${color_img_topic}" \
    --init_pose "${init_pose}" \
    --refine_num ${refine_num} \
    --debug
