# 🔗 연동 (`couping_integration` 패키지)

두산 피킹 · 핑키 운반 · OMX 포장을 **하나의 주문 흐름으로 잇는 코드**와 **주문자 / 관리자 웹 UI**.
두산 피킹 노드·비전은 `../doosan/couping` 패키지에 있고, 이 패키지는 그것을 함께 띄운다.

## 구성
| 파일 | 역할 |
|---|---|
| `couping_integration/orchestrator.py` | 주문 대기열. `/couping/order` 를 받아 `MOVING_TO_PICK(핑키 → 창고) → PICKING(두산이 핑키 상자에 담음) → MOVING_TO_PACK(핑키 → OMX 포장 구역) → PACKING(OMX) → [RETURNING] → DONE`. 두산은 `/couping/picker/cmd`(`pick:tomato,salad`), 핑키는 Nav2 `/navigate_to_pose`, OMX 는 `/couping/packer/cmd`. 상태 `/couping/state`, 주문별 `/couping/order/status`, 서비스 `/couping/stop`·`/couping/resume`·`/couping/start` |
| `couping_integration/web_ui.py` | 웹 서버 겸 ROS 노드 (추가 설치 없음). 주문자 `/`, 관리자 `/admin`, 실시간 상태 `/events`(SSE), 주문 `POST /api/order`, 정지·재개 `POST /api/stop`·`/api/resume`, 카메라 `/camera.mjpg` |
| `web/customer.html`, `web/admin.html` | 주문자 화면(상품 고르기 → 주문 → 주문 확인·상품 담는 중·포장 시작·배송 시작·배송 완료), 관리자 화면(들어온 주문·품목별 상태·진행 단계·카메라·로봇 상태·신호 로그·정지/재개) |
| `couping_integration/order_terminal.py` | 터미널 번호 주문 (`ros2 run couping_integration order` → `1, 3`) |
| `couping_integration/omx_packer.py` | `/couping/packer/cmd` 를 받아 양팔 OMX ACT 정책(`omx/scripts/run_act.sh`) 실행. `fake:=true` 면 5초 뒤 done |
| `couping_integration/pinky_signal.py`, `monitor.py`, `omx_check.py` | 핑키 신호 시험, 신호 모니터, OMX 연결 확인 |
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
```
- 주문자 `http://<노트북 IP>:8080/`, 관리자 `http://<노트북 IP>:8080/admin` (`web_port:=` 로 변경, `web:=false` 로 끔)
- **핑키 없이 두산 + OMX** (박스는 사람이 옮김): `pinky:=false omx:=true manual_transfer:=true`
  → 피킹이 끝나면 관리자 화면에 **'박스 옮김 완료'** 버튼이 뜨고, 누르면 OMX 포장 시작 (`/couping/confirm`)
- OMX 는 OMX 팀 `omx/scripts/run_act.sh` 를 실행한다. 정책 경로·카메라는 `config/integration.yaml` 의 `omx_packer` 에서 (비우면 스크립트 기본값)
- 핑키·OMX 단계를 켜려면 `pinky:=true`, `omx:=true` 와 `config/integration.yaml` 의 `pick_pose`(창고), `pack_pose`(포장 구역), `home_pose`(주차 구역), OMX 정책 경로를 채운다
- **실제 모드와 fake 모드를 같은 `ROS_DOMAIN_ID` 에서 동시에 띄우지 말 것** (두 피킹 노드가 같은 주문을 받아 실제 로봇이 움직임)

## 10/21 전체 연동 실행 순서 (두산 + 핑키 pinky-26116 + 정밀 도킹)

모든 노트북 터미널에서 먼저 `source integration/tools/dds_env.sh` (도메인 30, FastDDS. 노트북에서는 `~/couping_ws/dds_env.sh` 로 복사해 씀).
핑키 pinky-26116 은 CycloneDDS 가 없어 노트북도 FastDDS 를 써야 데이터가 들어옴.

1. 핑키(SSH): 도메인 30 으로 `pinky_bringup bringup_robot.launch.xml` + `python3 ~/pinky_camera_node.py` (도킹용 `/camera/image_raw/compressed`)
2. 두산: `ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.13 model:=e0509 remap_tf:=true`
   - **remap_tf:=true 필수**: 두산과 핑키가 둘 다 `base_link` 프레임을 써서, 안 붙이면 TF 가 꼬여 Nav2 가 위치를 못 잡음 (두산 TF 는 /dsr01/tf 로 감)
3. 그리퍼: `ros2 run dsr_gripper gripper_service_a`
4. Nav2: `ros2 launch pinky_navigation bringup_arena.launch.xml` + `nav2_view.launch.xml` → RViz 에서 2D Pose Estimate
5. 연동: `ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=true docking:=true vision:=true` (OMX 없으면 `omx:=false`)
   - 정밀 도킹 노드(main 의 v6.0)는 이 launch 가 노트북에서 띄움
   - 구역: 선반1 7번 마커(휴지·세제·키친타월), 선반2 4번 마커(토마토·당근·샐러드), 포장 5번 마커. `config/integration.yaml` 의 `zone_*`

## 10/22 팀원 핑키 미션 코드 그대로 쓰는 방식 (권장)

핑키 주행·도킹·후진은 `amr/pinky_precision_docking` 의 `run_arena_mission` 을 수정 없이 그대로 쓰고,
두산은 미션이 보내는 작업 신호(`/pinky/station_task/trigger` START → `/pinky/station_task/complete`)로만 연결한다.

1. 핑키: `integration/tools/pinky_up.sh` + `pinky_fastdds_wlan0.xml` 을 핑키 `~/` 에 두고 `bash ~/pinky_up.sh`
   - 재부팅 뒤 남은 bringup·sllidar 프로세스가 있으면 모터 포트를 못 잡으니 먼저 정리
2. 두산 bringup(`remap_tf:=true`), 그리퍼, Nav2(`bringup_arena`) + RViz → 2D Pose Estimate (Nav2 는 하나만!)
3. 두산 picker·비전: `ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false docking:=false omx:=false vision:=true web:=false`
4. 도킹 노드(팀원 launch 그대로): `ros2 launch pinky_precision_docking precision_docking.launch.py`
5. 연결: `python3 integration/tools/mission_bridge.py --items tide`
6. 미션: `ros2 run pinky_precision_docking run_arena_mission --step 7 --no-home` → 끝나면 `--step packaging`
   - `--step N` 은 1~6 이면 **N번째 구역**(마커 번호 아님). 마커 5 포장 구역은 `--step packaging` 으로
7. 포장·복귀 (선반 미션이 끝난 뒤, OMX packer 실행 중): `python3 integration/scripts/pack_run.py --after-pick --dock-node laptop`
   - 포장대 주행 → 마커 5 도킹(노트북 v6.0 노드) → OMX `start`/`done` → 후진 0.35m → home(P) 복귀
   - 옵션 없이 실행하면 원래대로 home 확인부터 시작하고 핑키에서 v5 도킹을 띄움
