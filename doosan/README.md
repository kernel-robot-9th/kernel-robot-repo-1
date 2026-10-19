# 🦾 Doosan e0509 오더 피킹 (`couping` 패키지)

주문 → 진열대 비전 인식(RealSense D455F + YOLO11s) → 두산 e0509 **수평 접근 피킹** → 핑키 적재함(박스)에 넣기 → 카메라로 성공 확인·재시도.
주문 대기열·단계 전환(오케스트레이터)과 주문자 / 관리자 웹 UI 는 **`../integration/couping_integration`** 패키지에 있다.

## 폴더 구성
| 경로 | 내용 |
|---|---|
| `couping/` | ROS 2 (Jazzy) ament_python 패키지. `~/couping_ws/src/couping` 에 두고 빌드 |
| `couping/couping/` | 노드·모듈 (아래 표) |
| `couping/config/couping.yaml` | 피킹 속도·그리퍼·재시도, 비전 설정 |
| `couping/launch/vision.launch.py` | 비전 노드만 (전체 실행은 `couping_integration` 의 `integration.launch.py`) |
| `tools/` | 데이터 수집·YOLO 학습, 그리퍼 명령(`gripper.sh`), 준비자세(`go_ready.py`), 카메라 화면 등 |
| `calib/` | 현장 보정값 (아래 참고) |
| `README_workspace.md` | 노트북 전체 설치·단위 테스트·비전·캘리브레이션 상세 문서 |
| `../docs/doosan/` | 작업 기록 (10/14 비전·좌표변환, 10/15 주문 자동 피킹·박스 적재) |

### 주요 노드·모듈
| 파일 | 역할 |
|---|---|
| `doosan_picker.py` | `pick:<품목,...>` 명령을 받아 품목마다 `pick_core` 실행, 진행 상황 발행 |
| `pick_core.py` | 인식 → 동작 계획(도달거리·높이 사전 검사) → 실행 → **카메라로 진열대에서 빠졌는지 확인, 실패 시 재시도** |
| `shelf_vision.py` | RealSense + YOLO 인식, 칸 배정, 카메라→로봇 좌표 변환 (`~/venv/yolo` 파이썬으로 실행) |
| `teach_pose.py`, `box_poses.py`, `poses.py` | 관찰·front·박스 자세 저장 |
| `handeye.py`, `handeye_calib.py` | 핸드-아이 계산 도구 (현재는 강의 GUI 결과 사용) |
| `catalog.py` | 품목 번호·이름 (연동 패키지도 이 목록을 씀) |

품목 번호: 1 tomato, 2 carrot, 3 salad, 4 bath_tissue, 5 tide, 6 paper_towel

## 설치
```bash
cp -r doosan/couping integration/couping_integration ~/couping_ws/src/      # 또는 심볼릭 링크
cd ~/couping_ws && colcon build --packages-select couping couping_integration --symlink-install && source install/setup.bash
```
필요한 것 (이 저장소에 없음)
- `doosan-robot2` (jazzy 브랜치), 강의 `dsr_study` 의 `dsr_gripper`·`dsr_gripper_interfaces` (RH-P12-RN 그리퍼 서비스)
- YOLO 가중치: `~/couping_ws/runs/shelf/yolo11s_1280_v2/weights/best.pt` (`tools/train_yolo.py` 로 학습, 용량 문제로 미포함)
- YOLO 실행용 venv `~/venv/yolo` (ultralytics, pyrealsense2)

### 보정값 (`calib/`) 두는 곳
| 파일 | 위치 | 내용 |
|---|---|---|
| `T_base_camera.npy`, `.yaml` | `~/HamdEyeCal/결과/` (`couping.yaml` 의 `handeye_file`) | 카메라→로봇 베이스 변환 (10/15 재캘리브레이션, 8장, 오차 최대 9.86mm) |
| `grasp_offset.json` | `~/couping_ws/calib/` | 플랜지→잡는 점 오프셋 + 현장 보정(`target_adjust_mm`: +X 70, Z -15) |
| `poses.json` | `~/couping_ws/calib/` | observe, front, 박스(place_above, place, place_2~6) 자세 |
| `box_measure.json` | `~/couping_ws/calib/` | 박스 위치 측정값 (`box_poses.py` 의 `BOX`) |

**카메라·진열대·박스를 옮기면 이 값들은 다시 맞춰야 한다.**

## 실행
```bash
# 1) 두산 bringup
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.13 model:=e0509
# 2) 그리퍼 서비스
ros2 run dsr_gripper gripper_service_a
# 3) 주문 처리 + 비전 + 웹 UI
ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false omx:=false vision:=true
```
- 주문자 화면 `http://<노트북 IP>:8080/`, 관리자 화면 `http://<노트북 IP>:8080/admin`
- 로봇 없이 신호·UI 만 시험: 위 3) 에 `fake:=true` (비전 불필요). **실제 모드와 동시에 띄우지 말 것** (같은 주문을 둘 다 받아 실제 로봇이 움직임)
- 한 품목 단계별 시험: `ros2 run couping auto_pick tomato --step`

## 주의
- 로봇을 움직이는 명령은 비상정지 버튼을 든 사람이 로봇 앞에 있을 때만
- 카메라는 USB 3(USB-C 포트) 필수. USB 2 로 잡히면 비전 노드가 자동 리셋, 계속되면 다시 꽂기
