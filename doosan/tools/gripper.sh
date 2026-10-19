#!/usr/bin/env bash
# 그리퍼 열기/닫기 (RH-P12-RN). gripper_service 가 다른 터미널에서 실행 중이어야 함.
#   ~/couping_ws/tools/gripper.sh open          # 열기
#   ~/couping_ws/tools/gripper.sh close [힘]    # 닫기 (힘 기본 200, 단단히 잡을 땐 300~400)
#   ~/couping_ws/tools/gripper.sh 375           # 원하는 위치 (0=닫힘 ~ 750=열림)
# 그리퍼 제어는 auto 모드에서만 동작해서, 잠깐 auto 로 바꿨다가 원래 모드(수동이면 수동)로 되돌린다.
set -e
source /opt/ros/jazzy/setup.bash
source ~/couping_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
SYS=/dsr01/dsr_controller2/system

case "$1" in
  open)  POS=750 ;;
  close) POS=0 ;;
  ''|*[!0-9]*) echo "사용법: $0 open | close [힘] | <0~750>"; exit 1 ;;
  *)     POS=$1 ;;
esac
CUR=${2:-200}

mode=$(ros2 service call $SYS/get_robot_mode dsr_msgs2/srv/GetRobotMode | grep -o 'robot_mode=[0-9]' | cut -d= -f2)
[ "$mode" = "0" ] && ros2 service call $SYS/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}" >/dev/null && sleep 0.5
# 서비스 재시작 직후 첫 명령은 그리퍼 루프(DRL)가 뜨기 전에 들어가 무시될 수 있어 한 번 더 보낸다
for i in 1 2; do
  ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd "{position: $POS, current: $CUR}" | grep -o 'success=[A-Za-z]*'
  sleep 1.0
done
sleep 1.5   # 그리퍼가 다 움직일 때까지 auto 유지
[ "$mode" = "0" ] && ros2 service call $SYS/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 0}" >/dev/null && echo "수동 모드로 복귀"
echo "그리퍼 position=$POS current=$CUR"
