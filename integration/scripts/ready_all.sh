#!/usr/bin/env bash
# ==============================================================================
# 🚀 Couping One-Click Automated System Bringup & Self-Check Script
# ==============================================================================
# 사용법:
#   bash ~/couping_ws/ready_all.sh [핑키번호]
#   bash ~/couping_ws/ready_all.sh 3       # 기본값: 3호기 (도메인 3)
# ==============================================================================

set -eo pipefail

P="${1:-3}"
DOOSAN_IP="110.120.1.13"
PINKY_IP="192.168.111.10${P}"
REPO="/home/omen/team/kernel-robot-repo-1"
WS="/home/omen/couping_ws"
LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

# ANSI 색상
GREEN="\033[1;32m"
RED="\033[1;31m"
YELLOW="\033[1;33m"
CYAN="\033[1;36m"
BOLD="\033[1m"
NC="\033[0m"

log_info() { echo -e "${CYAN}[INFO]${NC} $*"; }
log_ok()   { echo -e "  ${GREEN}[OK]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_err()  { echo -e "${RED}[FAIL]${NC} $*"; }

T_START=$(date +%s)
echo -e "\n${BOLD}================================================================${NC}"
echo -e "${BOLD} 🤖 Couping 통합 시스템 원클릭 기동 (핑키 ${P}호기 | 도메인 ${P})${NC}"
echo -e "${BOLD}================================================================${NC}\n"

# ------------------------------------------------------------------------------
# 1. ROS 2 및 DDS 환경 로드
# ------------------------------------------------------------------------------
log_info "1. 로컬 환경 변수 로드 (ROS 2 Jazzy + CycloneDDS 도메인 $P)..."
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
source "$WS/dds_env.sh" "$P" > /dev/null

# ------------------------------------------------------------------------------
# 2. 로컬 잔여 프로세스 정리 (Clean Slate)
# ------------------------------------------------------------------------------
log_info "2. 로컬 잔여 ROS 프로세스 정리 중..."
pkill -9 -f "ros2_control_node|gripper_service_a|scan_self_filter|bringup_arena|precision_docking|shelf_vision|doosan_picker|omx_packer|web_ui|web_order_run|order_run" 2>/dev/null || true
sleep 1
log_ok "로컬 프로세스 정리 완료"

# ------------------------------------------------------------------------------
# 3. 네트워크 및 하드웨어 연결 사전 점검
# ------------------------------------------------------------------------------
log_info "3. 하드웨어 네트워크 핑 점검..."
if ping -c 1 -W 1 "$DOOSAN_IP" >/dev/null 2>&1; then
    log_ok "두산 제어기 응답 ($DOOSAN_IP)"
else
    log_err "두산 로봇팔 제어기($DOOSAN_IP) 핑 무응답! 랜선 및 제어기 전원을 확인하세요."
    exit 1
fi

if ping -c 1 -W 1 "$PINKY_IP" >/dev/null 2>&1; then
    log_ok "핑키 ${P}호기 응답 ($PINKY_IP)"
else
    log_err "핑키 ${P}호기($PINKY_IP) 핑 무응답! Wi-Fi 연결을 확인하세요."
    exit 1
fi

# ------------------------------------------------------------------------------
# 4. 핑키 로봇 하드웨어 브링업 원격 확인 & 자동 기동
# ------------------------------------------------------------------------------
log_info "4. 핑키 ${P}호기 온보드 브링업 점검..."
if ! ssh -o ConnectTimeout=2 -o BatchMode=yes "pinky@$PINKY_IP" "pgrep -f pinky_bringup" >/dev/null 2>&1; then
    log_warn "핑키 온보드 브링업이 꺼져 있습니다. 원격 실행(pinky_up.sh) 시작..."
    ssh -o ConnectTimeout=3 "pinky@$PINKY_IP" "nohup bash ~/pinky_up.sh > /tmp/pinky_up_run.log 2>&1 &"
    sleep 3
fi
log_ok "핑키 ${P}호기 모터 및 카메라 브링업 가동 중"

# ------------------------------------------------------------------------------
# 5. 서브시스템 병렬 백그라운드 런치
# ------------------------------------------------------------------------------
log_info "5. 서브시스템 병렬 백그라운드 기동 시작..."

# 5-1. 두산 로봇팔 제어기
log_info "  → [두산] dsr_bringup2 구동..."
nohup ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real host:="$DOOSAN_IP" model:=e0509 remap_tf:=true gui:=false \
    > "$LOG_DIR/dsr_bringup.log" 2>&1 &
disown

# 5-2. 두산 그리퍼 서비스
log_info "  → [그리퍼] gripper_service_a 구동..."
nohup ros2 run dsr_gripper gripper_service_a \
    > "$LOG_DIR/gripper_service.log" 2>&1 &
disown

# 5-3. 핑키 라이다 필터 + Nav2 아레나 내비게이션
log_info "  → [Nav2] 라이다 셀프필터 + bringup_arena 구동..."
nohup python3 "$REPO/integration/tools/scan_self_filter.py" > "$LOG_DIR/scan_self_filter.log" 2>&1 &
disown
nohup ros2 launch pinky_navigation bringup_arena.launch.xml \
    > "$LOG_DIR/nav2_arena.log" 2>&1 &
disown

# 5-4. 핑키 ArUco 정밀 도킹 노드
log_info "  → [도킹] precision_docking 구동..."
nohup ros2 launch pinky_precision_docking precision_docking.launch.py \
    > "$LOG_DIR/precision_docking.log" 2>&1 &
disown

# 5-5. 통합 연동 (비전 + 두산 피커 + OMX 패커 + Web UI)
log_info "  → [통합] shelf_vision + doosan_picker + omx_packer + web_ui 구동..."
nohup bash -c "OMX_SIM_UDP=127.0.0.1:5005 ros2 launch couping_integration integration.launch.py \
    dsr_bringup:=false pinky:=false docking:=false \
    vision:=true doosan:=true omx:=true web:=true orchestrator:=false sim_view:=true" \
    > "$LOG_DIR/integration_system.log" 2>&1 &
disown

# ------------------------------------------------------------------------------
# 6. 실시간 헬스체크 및 자동 초기화
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}================================================================${NC}"
echo -e "${BOLD} 🔍 서브시스템 활성화 자동 점검 및 자율 초기화 대기${NC}"
echo -e "${BOLD}================================================================${NC}"

# 6-1. 두산 로봇팔 STANDBY 상태 대기 (최대 25초)
log_info "두산 로봇팔 STANDBY 상태 진입 대기..."
DSR_READY=0
for i in {1..25}; do
    if ros2 service call /dsr01/dsr_controller2/system/get_robot_state dsr_msgs2/srv/GetRobotState {} 2>/dev/null | grep -q "robot_state=1"; then
        DSR_READY=1
        break
    fi
    sleep 1
done
if [ $DSR_READY -eq 1 ]; then
    log_ok "두산 로봇팔 STANDBY (제어 준비 완료)"
else
    log_err "두산 로봇팔 활성화 시간 초과! 로그($LOG_DIR/dsr_bringup.log)를 확인하세요."
    exit 1
fi

# 6-2. Nav2 Lifecycle 대기 (최대 25초)
log_info "Nav2 내비게이션 활성화 대기..."
NAV2_READY=0
for i in {1..25}; do
    if ros2 action list 2>/dev/null | grep -q "navigate_to_pose" || ros2 topic list 2>/dev/null | grep -q "/global_costmap/costmap"; then
        NAV2_READY=1
        break
    fi
    sleep 1
done
if [ $NAV2_READY -eq 1 ]; then
    log_ok "Nav2 내비게이션 활성화 완료"
else
    log_err "Nav2 활성화 시간 초과! 로그($LOG_DIR/nav2_arena.log)를 확인하세요."
    exit 1
fi

# 6-3. 핑키 초기 위치(Home Parking) 설정 및 검증
log_info "핑키 ${P}호기 초기 위치(Home Parking) 자동 부여..."
python3 "$REPO/integration/tools/set_home_pose.py" > /dev/null 2>&1
sleep 1.5
POSE_STR=$(python3 "$REPO/integration/tools/get_pose.py" 2>/dev/null | grep "Tuple:" || echo "")
log_ok "핑키 홈 위치 초기화 완료: ${POSE_STR}"

# 6-4. 비전 및 피커/패커 서비스 준비 확인 (최대 15초)
log_info "진열대 비전(RealSense) 및 피커·패커 노드 준비 대기..."
INT_READY=0
for i in {1..15}; do
    TOPICS=$(ros2 topic list 2>/dev/null || true)
    if echo "$TOPICS" | grep -q "/couping/picker/cmd" && echo "$TOPICS" | grep -q "/couping/packer/cmd"; then
        INT_READY=1
        break
    fi
    sleep 1
done
if [ $INT_READY -eq 1 ]; then
    log_ok "진열대 비전(D455F) 및 피커/패커 노드 준비 완료"
else
    log_warn "통합 노드 확인 지연. 로그($LOG_DIR/integration_system.log) 확인 필요"
fi

# ------------------------------------------------------------------------------
# 7. Web UI 주문 수신 러너 가동
# ------------------------------------------------------------------------------
log_info "7. 웹 주문 자동 실행기(web_order_run.py) 백그라운드 기동..."
pkill -f "web_order_run.py" 2>/dev/null || true
nohup bash -c "PINKY=\"$P\" python3 \"$REPO/integration/tools/web_order_run.py\"" > "$LOG_DIR/web_order_run.log" 2>&1 &
disown
sleep 1
log_ok "웹 주문 리스너 준비 완료"

T_END=$(date +%s)
ELAPSED=$((T_END - T_START))

echo -e "\n${BOLD}================================================================${NC}"
echo -e "${GREEN}${BOLD} 🎉 모든 준비 완료! (총 소요 시간: ${ELAPSED}초)${NC}"
echo -e "${BOLD}================================================================${NC}"
echo -e " 🌐 ${BOLD}고객 웹 화면:${NC} http://localhost:8080/"
echo -e " 🛠️  ${BOLD}관리자 화면:${NC}  http://localhost:8080/admin"
echo -e " 📦 ${BOLD}터미널 테스트:${NC} bash $REPO/integration/scripts/order_run.sh [품목1,품목2]"
echo -e "================================================================\n"
