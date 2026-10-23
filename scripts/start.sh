#!/bin/bash

set -e
cd "$(dirname "$0")/.."
WORKSPACE_ROOT=$(pwd)
VISION_CONFIG_PATH="${QYBOT_VISION_CONFIG_PATH:-/opt/booster}"

echo "[STOP EXISTING QYBOT NODES]"
./scripts/stop.sh
# Performance tuning is opt-in and never masks system updates or kills Python.
if [ "${QYBOT_TUNE_SYSTEM:-0}" = "1" ]; then
    sudo jetson_clocks
fi

echo "[START ROBOCUP NODES]"
source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
# export RMW_FASTRTPS_USE_SHM=0


echo "[START VISION]"
# 如果是用的zed相机，https://booster.feishu.cn/wiki/XodtwX56AiCtZtkewo3cPgbrn8d#share-MDrvdyWa2o87qixU3TccE72NnNc 文档中下载安装包，可以自启动zed
# 注意: 如果是realsense相机，千万不要改，否则realsense摄像头会起不来，这个时候只能cd ~/Documents/recovery/ 重装daemon-perception恢复
# source ~/ThirdParty/zed-ros/install/setup.bash
# nohup ros2 launch zed_wrapper zed_camera.launch.py camera_model:="zed2i" > zed.log 2>&1 &
# nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
nohup ros2 launch vision launch.py vision_config_path:="${VISION_CONFIG_PATH}" save_data:=true > vision.log 2>&1 &
# nohup ros2 run ros2_sync_package sync_node > sync_node.log 2>&1 &
# nohup sh src/vision_segmentation/run.sh > vision_segmentation.log 2>&1 &
echo "[START BRAIN]"
nohup ros2 launch brain launch.py vision_config_path:="${VISION_CONFIG_PATH}" "$@" > brain.log 2>&1 &
# nohup ros2 launch brain launch.py "$@"  > brain.log 2>&1 &
echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
echo "[START WEBUI_CLIENT]"
nohup ros2 launch k1_robot_webui_client webui_client.launch.py log_directory:="${WORKSPACE_ROOT}" > webui_client.log 2>&1 &
#echo "[START SOUND]"
#nohup ros2 run sound_play sound_play_node > sound.log 2>&1 &
echo "[DONE]"
