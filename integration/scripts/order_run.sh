#!/usr/bin/env bash
# 주문 한 건: P → [생필품 있으면] 선반1 마커 도킹·두산 피킹 → [식자재 있으면] 선반2 마커 도킹·두산 피킹 → 포장대(5) 도킹·OMX
#
#   bash integration/scripts/order_run.sh tide,carrot            # 회차 이름은 날짜_시각
#   bash integration/scripts/order_run.sh tide,carrot 3          # 로그 이름 order_3_*.log
#   SHELF1=9 SHELF2=2 bash integration/scripts/order_run.sh ...  # 도킹 마커 바꾸기 (기본 7, 4)
#   RETURN_HOME=1 bash ...                                        # 포장 뒤 후진해서 home(P) 복귀
#   HOME_FIT=1 bash ...                                           # P 정위치가 아닐 때: 라이다로 초기 위치 맞춤 (기본은 고정 좌표)
#   SHELF_DIST=0.03 PACK_DIST=0.07 bash ...                      # 도킹 목표 거리(m, 마커 벽까지). 기본 선반 0.03, 포장대 0.07
#
# 주행·도킹·후진은 팀원 run_arena_mission(--step <마커>) 을 그대로 쓰고, 포장대는 pack_run.py 를 쓴다.
# 두산·비전·OMX packer·Nav2·핑키 bringup/도킹 노드는 미리 띄워 둔다 (integration/README.md 10/26).
# 구역별 품목은 mission_bridge.py 의 ZONE_ITEMS (선반1 = tide·paper_towel·bath_tissue, 선반2 = carrot·tomato·salad).
ORDER="${1:?주문 품목을 쉼표로 (예: tide,carrot)}"
N="${2:-$(date +%m%d_%H%M%S)}"
SHELF1="${SHELF1:-7}"
SHELF2="${SHELF2:-4}"
SHELF_DIST="${SHELF_DIST:-0.03}"
PACK_DIST="${PACK_DIST:-0.07}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
L=~/couping_ws/logs
source /opt/ros/jazzy/setup.bash && source ~/couping_ws/install/setup.bash && source ~/couping_ws/dds_env.sh
set -u   # ROS setup.bash 는 정의 안 된 변수를 써서 source 뒤에 켬

NEED1=0; NEED2=0
for it in ${ORDER//,/ }; do
    case "$it" in
        tide|paper_towel|bath_tissue) NEED1=1 ;;
        carrot|tomato|salad) NEED2=1 ;;
        *) echo "알 수 없는 품목: $it (tomato carrot salad bath_tissue tide paper_towel)"; exit 1 ;;
    esac
done
echo "주문 $N: $ORDER  →  선반1(마커 $SHELF1) $([ $NEED1 = 1 ] && echo 들름 || echo 건너뜀), 선반2(마커 $SHELF2) $([ $NEED2 = 1 ] && echo 들름 || echo 건너뜀), 포장대(5)"

# 연결 노드: 이 주문 품목으로 새로 띄운다 (남아 있는 것은 끔. 두 개면 두산에 피킹이 두 번 감)
# 10/27: 출발 지연 최소화 - 기다리지 않고 띄움 (두산 START 는 첫 도킹 뒤라 그 전에 준비됨)
pkill -f "integration/tools/mission_bridge.py"
python3 "$REPO/integration/tools/mission_bridge.py" --order "$ORDER" > "$L/order_${N}_bridge.log" 2>&1 &
BRIDGE=$!
trap 'kill $BRIDGE 2>/dev/null' EXIT

dock_dist() {   # 핑키 도킹 노드 목표 거리를 뒤에서 바꿈 (주행하는 동안 끝남, 기다리지 않음). 결과는 order_N_dockdist.log
    ( for try in 1 2 3; do
          timeout 10 ros2 param set /precision_docking_node target_distance "$1" > /dev/null 2>&1
          timeout 10 ros2 param get /precision_docking_node target_distance 2>/dev/null | grep -q "value is: $1\$" \
              && { echo "$(date +%T) $1 m 설정됨"; break; }
          echo "$(date +%T) $1 m 설정 실패 ($try/3)"
      done >> "$L/order_${N}_dockdist.log" 2>&1 ) &
}

T0=$(date +%s)
dock_dist "$SHELF_DIST"
# P 에 정해진 방향으로 두고 시작하므로 고정 좌표로 초기 위치 (약 1초). 다른 곳에서 시작하면 HOME_FIT=1 (라이다 맞춤)
if [ "${HOME_FIT:-0}" = 1 ]; then python3 "$REPO/integration/tools/home_pose.py" || exit 1
else python3 "$REPO/integration/tools/set_home_pose.py"; fi

shelf() {   # $1 = 마커 번호. 터미널 입력은 막아 둠 (빈 줄로 두산보다 먼저 떠나지 않게)
    ros2 run pinky_precision_docking run_arena_mission --step "$1" --no-home < <(sleep infinity) 2>&1 | tee "$L/order_${N}_m$1.log"
    grep -q "SUCCESS\] ENTIRE" "$L/order_${N}_m$1.log" || { echo "중단: 마커 $1 구역 실패 (로그 $L/order_${N}_m$1.log)"; exit 1; }
}
[ $NEED1 = 1 ] && shelf "$SHELF1"
[ $NEED2 = 1 ] && shelf "$SHELF2"

dock_dist "$PACK_DIST"   # 포장대까지 주행하는 동안 뒤에서 설정
python3 "$REPO/integration/scripts/pack_run.py" --after-pick --dock-node laptop $([ "${RETURN_HOME:-0}" = 1 ] || echo --no-return) 2>&1 | tee "$L/order_${N}_pack.log"
grep -q "전체 완료" "$L/order_${N}_pack.log" || { echo "중단: 포장대 단계 실패"; exit 1; }
echo "주문 $N 완료: $(( $(date +%s) - T0 ))초"
