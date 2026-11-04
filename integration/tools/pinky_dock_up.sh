# 핑키에서 실행: 정밀 도킹 노드(~/couping_dock_ws, amr/pinky_precision_docking 복사본) + 부저 노드.
# 카메라 영상이 Wi-Fi 를 타지 않게 도킹 노드를 핑키 안에서 돌린다 (10/23). pinky_up.sh 를 먼저 실행.
#   bash ~/pinky_dock_up.sh
source /opt/ros/jazzy/setup.bash
source ~/couping_dock_ws/install/setup.bash
export ROS_DOMAIN_ID=30 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/pinky_fastdds_wlan0.xml
unset RMW_IMPLEMENTATION
ros2 launch pinky_precision_docking precision_docking.launch.py > /tmp/docking.log 2>&1 &
ros2 run pinky_precision_docking pinky_buzzer_node > /tmp/buzzer.log 2>&1 &
wait
