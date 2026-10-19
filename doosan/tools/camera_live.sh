#!/bin/bash
# 비전 노드(shelf_vision)의 실시간 화면(YOLO 박스 포함)을 띄운다. 창을 닫거나 q 로 종료하면 실시간 발행도 끔.
# 비전 노드가 카메라를 쥐고 있으므로 카메라를 직접 열지 않고 /couping/vision/image/compressed 를 받아 본다.
source /opt/ros/jazzy/setup.bash && source ~/couping_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
RATE=${1:-10.0}
ros2 param set /shelf_vision preview_rate "$RATE"
trap 'ros2 param set /shelf_vision preview_rate 0.0 >/dev/null 2>&1' EXIT
~/venv/yolo/bin/python ~/couping_ws/tools/vision_view.py 2>&1 | grep -v -e QFontDatabase -e "Qt no longer ships"
