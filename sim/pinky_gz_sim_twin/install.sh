#!/usr/bin/env bash
# Gazebo 디지털 트윈 설치: 핑키 공식 pinky_pro 의 pinky_gz_sim 위에 이 폴더(트윈 추가분)를 덮어쓰고 빌드한다.
#
#   bash sim/pinky_gz_sim_twin/install.sh             # ~/pinky_pro 에 설치
#   PINKY_PRO=~/other/pinky_pro bash sim/pinky_gz_sim_twin/install.sh
#
# 필요: ~/pinky_pro (https://github.com/pinklab-art/pinky_pro, 10/19 기준 커밋 75f76e8),
#       ~/doosan_ws (dsr_description2: 핑키 위 축소 e0509 모양),
#       sudo apt install ros-jazzy-domain-bridge ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
# 덮어쓰기 전 원래 파일은 <pinky_pro>/backup/pinky_gz_sim.before_twin_<시각>/ 에 남긴다.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PINKY_PRO="${PINKY_PRO:-$HOME/pinky_pro}"
PKG="$PINKY_PRO/src/pinky_pro/pinky_gz_sim"
[ -d "$PKG" ] || { echo "pinky_gz_sim 이 없음: $PKG (git clone https://github.com/pinklab-art/pinky_pro.git $PINKY_PRO/src/pinky_pro)"; exit 1; }

BACKUP="$PINKY_PRO/backup/pinky_gz_sim.before_twin_$(date +%m%d_%H%M%S)"
mkdir -p "$PINKY_PRO/backup" && touch "$PINKY_PRO/backup/COLCON_IGNORE"   # 빌드 때 백업을 같은 이름 패키지로 보지 않게
cp -r "$PKG" "$BACKUP"
echo "원래 파일 백업: $BACKUP"

(cd "$HERE" && find . -type f ! -name install.sh ! -name README.md) | while read -r f; do
    mkdir -p "$PKG/$(dirname "$f")"
    cp "$HERE/$f" "$PKG/$f"
done
# 바닥 무늬 그림은 절대 경로로만 읽혀서, 설치한 자리로 바꿔 넣는다
sed -i "s#file://__TWIN_FLOOR__#file://$PKG/models/img/floor_concrete.png#" "$PKG/worlds/pinky_factory.world"
chmod +x "$PKG"/scripts/*.py

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/doosan_ws/install/setup.bash" ] && source "$HOME/doosan_ws/install/setup.bash"
cd "$PINKY_PRO" && colcon build --packages-select pinky_gz_sim
echo "설치 끝. 실행: source $PINKY_PRO/install/setup.bash && ros2 launch couping_integration twin.launch.py real_domain:=<핑키 번호>"
