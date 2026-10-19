# 🔗 연동 (`couping_integration` 패키지)

두산 피킹 · 핑키 운반 · OMX 포장을 **하나의 주문 흐름으로 잇는 코드**와 **주문자 / 관리자 웹 UI**.
두산 피킹 노드·비전은 `../doosan/couping` 패키지에 있고, 이 패키지는 그것을 함께 띄운다.

## 구성
| 파일 | 역할 |
|---|---|
| `couping_integration/orchestrator.py` | 주문 대기열. `/couping/order` 를 받아 `[MOVING_TO_PICK → PICKING] × 구역 수 → MOVING_TO_PACK → PACKING → RETURNING → DONE`. 주문 품목이 속한 피킹 구역을 `pick_zones` 순서로 들르고, 핑키가 구역에 도착한 뒤 두산이 그 구역 품목만 집는다. 두산은 `/couping/picker/cmd`(`pick:tomato,salad`, 두 번째 구역부터 `pick@<첫 자리>:carrot`), 핑키는 Nav2 `/navigate_to_pose`, OMX 는 `/couping/packer/cmd`. 상태 `/couping/state`, 주문별 `/couping/order/status`, 서비스 `/couping/stop`·`/couping/resume`·`/couping/start` |
| `couping_integration/web_ui.py` | 웹 서버 겸 ROS 노드 (추가 설치 없음). 주문자 `/`, 관리자 `/admin`, 실시간 상태 `/events`(SSE), 주문 `POST /api/order`, 정지·재개 `POST /api/stop`·`/api/resume`, 카메라 `/camera.mjpg` |
| `web/customer.html`, `web/admin.html` | 주문자 화면(상품 고르기 → 주문 → 주문 확인·상품 담는 중·포장 시작·배송 시작·배송 완료), 관리자 화면(들어온 주문·품목별 상태·진행 단계·카메라·로봇 상태·신호 로그·정지/재개) |
| `couping_integration/order_terminal.py` | 터미널 번호 주문 (`ros2 run couping_integration order` → `1, 3`) |
| `couping_integration/omx_packer.py` | `/couping/packer/cmd` 를 받아 양팔 OMX ACT 정책(`omx/scripts/run_act.sh`) 실행. `fake:=true` 면 5초 뒤 done |
| `couping_integration/pinky_signal.py`, `monitor.py`, `omx_check.py` | 핑키 신호 시험, 신호 모니터, OMX 연결 확인 |
| `couping_integration/fake_pinky.py` | 가짜 핑키(Nav2 `NavigateToPose` 흉내). 거리 / `speed` 초 뒤 도착. `ros2 param set /fake_pinky fail true` 면 도착 실패. 이동 중 `/amcl_pose`·`/odom` 을 내서 Gazebo 디지털 트윈의 가상 핑키도 움직임 |
| `couping_integration/fake_twin.py` | 가짜 두산·OMX 동작을 디지털 트윈에 보냄. 두산 '집는 중' 마다 `/dsr01/joint_states`, OMX 포장 중에는 LeRobot 과 같은 UDP(127.0.0.1:5005) |
| `couping_integration/sim_camera.py` | 디지털 트윈(도메인 `sim_domain`, 기본 10)의 위에서 본 카메라 `/camera` → JPEG → 연동 도메인 `/couping/sim/image/compressed`. 관리자 화면 카메라 칸의 '시뮬레이션' 버튼(`/sim.mjpg`) |
| `launch/twin.launch.py` | Gazebo 디지털 트윈(`~/pinky_pro` 의 `pinky_gz_sim`)을 arena_map 보정값으로 실행 |
| `launch/integration.launch.py` | 두산 bringup(선택) · 두산 피킹 · 비전(선택) · OMX · 오케스트레이터 · 웹 UI 를 한 번에 |
| `config/integration.yaml` | 오케스트레이터(단계 on/off, 타임아웃, 핑키 맵 좌표) · OMX 정책 설정 |

## 토픽 약속 (다른 로봇 팀이 맞출 것)
| 토픽 | 방향 | 내용 |
|---|---|---|
| `/couping/<unit>/cmd` (`std_msgs/String`) | 오케스트레이터 → 로봇 | `start` (두산은 `pick:<품목,...>`), `stop` |
| `/couping/<unit>/status` (`std_msgs/String`) | 로봇 → 오케스트레이터 | `running` → (`progress:<문구>`) → `done` 또는 `fail:<이유>` |
| `/couping/order` (`std_msgs/String`, JSON) | 주문 UI → 오케스트레이터 | `{"id": "001", "items": ["tomato", "salad"]}` |

`<unit>` = `picker`(두산), `packer`(OMX), `pinky`(신호 모드). 핑키 실제 주행은 Nav2 `NavigateToPose` 액션.

## 실행
```bash
colcon build --packages-select couping couping_integration --symlink-install && source install/setup.bash
# 실제 (두산 bringup·그리퍼 서비스는 따로)
ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false omx:=false vision:=true
# 로봇·카메라 없이 신호와 UI 만
ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false omx:=false fake:=true
# 로봇 없이 전체 흐름 (두산·OMX fake + 가짜 핑키). 실제 로봇과 섞이지 않게 이 PC 안에서만 통신
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch couping_integration integration.launch.py dsr_bringup:=false fake:=true fake_pinky:=true
```
**Gazebo 디지털 트윈에서 보기** (`~/pinky_pro` 의 `pinky_gz_sim`, 터미널 2개. 둘 다 같은 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`)
```bash
# 1) 디지털 트윈: 연동 도메인(87)의 핑키·두산 신호를 시뮬 도메인(10)으로 받아 따라 움직임
#    twin.launch.py = pinky_gz_sim digital_twin.launch.xml + arena_map 기준 보정값 (기본값은 다른 맵 기준이라 위치가 틀어짐)
source ~/doosan_ws/install/setup.bash && source ~/pinky_pro/install/setup.bash
ros2 launch couping_integration twin.launch.py real_domain:=87
# 2) 가짜 주문 흐름 + 디지털 트윈 연결 + 관리자 화면에 Gazebo 화면
export ROS_DOMAIN_ID=87
ros2 launch couping_integration integration.launch.py dsr_bringup:=false fake:=true fake_pinky:=true fake_twin:=true sim_view:=true
```
관리자 화면 카메라 칸에서 '시뮬레이션' → '화면 켜기'. 실제 로봇일 때는 `fake*` 없이 `sim_view:=true` 만 주고, 디지털 트윈은 `twin.launch.py real_domain:=<실제 로봇 도메인>` 으로 실행.
`integration.yaml` 의 핑키 좌표는 Gazebo 월드의 진열대·OMX 앞에 서도록 정한 arena_map 좌표다 (대기: 왼쪽 위 방, 생활용품: 오른쪽 위 진열대, 식품: 가운데 아래 진열대 옆, 포장: 왼쪽 아래 OMX 앞).
- 주문자 `http://<노트북 IP>:8080/`, 관리자 `http://<노트북 IP>:8080/admin` (`web_port:=` 로 변경, `web:=false` 로 끔)
- **핑키 없이 두산 + OMX** (박스는 사람이 옮김): `pinky:=false omx:=true manual_transfer:=true`
  → 피킹이 끝나면 관리자 화면에 **'박스 옮김 완료'** 버튼이 뜨고, 누르면 OMX 포장 시작 (`/couping/confirm`)
- OMX 는 OMX 팀 `omx/scripts/run_act.sh` 를 실행한다. 정책 경로·카메라는 `config/integration.yaml` 의 `omx_packer` 에서 (비우면 스크립트 기본값)
- 핑키·OMX 단계를 켜려면 `pinky:=true`, `omx:=true` 와 `config/integration.yaml` 의 피킹 구역(`pick_zones`: 생활용품 `daily`, 식품 `food` — 구역마다 `items`와 `pose`), `pack_pose`(OMX 앞), `home_pose`(핑키 대기 위치), OMX 정책 경로를 채운다. 지금 좌표는 맵에서 대략 정한 값(yaw 0)이라 실제 핑키로 확인해야 한다
- **실제 모드와 fake 모드를 같은 `ROS_DOMAIN_ID` 에서 동시에 띄우지 말 것** (두 피킹 노드가 같은 주문을 받아 실제 로봇이 움직임)
