# 핑키(pinky-26116)에서 실행: bringup + 도킹용 카메라. 이 파일과 pinky_fastdds_wlan0.xml 을 핑키 홈(~/)에 복사해 bash ~/pinky_up.sh
# FastDDS 를 wlan0(192.168.111.102)만 쓰게 고정: 핑키 자체 AP(ap0) 주소로 알려져 노트북에 데이터가 안 오는 문제 방지 (10/22)
source /opt/ros/jazzy/setup.bash
source /home/pinky/pinky_pro/install/local_setup.bash
export ROS_DOMAIN_ID=30
unset RMW_IMPLEMENTATION
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pinky/pinky_fastdds_wlan0.xml
ros2 launch pinky_bringup bringup_robot.launch.xml > /tmp/pinky_bringup.log 2>&1 &
sleep 3
python3 /home/pinky/pinky_camera_node.py > /tmp/pinky_camera.log 2>&1 &
wait
