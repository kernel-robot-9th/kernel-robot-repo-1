# 핑키에서 실행: Nav2 (지도 + AMCL + 주행). 라이다가 Wi-Fi 를 안 타게 핑키 안에서 돌린다 (10/27). pinky_up.sh 를 먼저 실행.
#   bash ~/pinky_nav_up.sh
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/local_setup.bash
source ~/couping_dock_ws/install/setup.bash
export ROS_DOMAIN_ID=30 FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/pinky_fastdds_wlan0.xml
unset RMW_IMPLEMENTATION
exec ros2 launch pinky_navigation bringup_arena.launch.xml > /tmp/nav2.log 2>&1
