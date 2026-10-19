# Couping 1차 연동테스트 (두산 → 핑키 → OMX)

이 노트북 한 대에서 세 로봇을 연결하고, 오케스트레이터가 단계를 자동으로 넘깁니다.

```
노트북 (ROS 2 Jazzy, ROS_DOMAIN_ID=30)
 ├─ 이더넷 110.120.1.100 ── 두산 카트 공유기 110.120.1.1 ── E0509 (110.120.1.<시리얼 숫자>)
 ├─ USB /dev/omx_follower ── OMX (LeRobot ACT 정책)
 └─ Wi-Fi ───────────────── 핑키 (RPi, ROS 2 Jazzy, Nav2)

/couping/start → MOVING_TO_PICK(핑키) → PICKING(두산) → MOVING_TO_PACK(핑키) → PACKING(OMX) → RETURNING(핑키) → DONE
```

| 노드 | 역할 | 토픽/서비스 |
|---|---|---|
| `orchestrator` | 상태머신, 핑키 Nav2 목표 전송 | `/couping/start`, `/couping/stop` (Trigger), `/couping/state` |
| `doosan_picker` | 고정 관절 포즈로 픽앤플레이스 | `/couping/picker/cmd` → `/couping/picker/status` |
| `omx_packer` | `~/il_ws/scripts/eval.sh` 실행 | `/couping/packer/cmd` → `/couping/packer/status` |

상태 문자열: `running` / `done` / `fail:<이유>`. 설정은 [src/couping/config/couping.yaml](src/couping/config/couping.yaml)에 있습니다.

---

## ▶ 신호 연동 전체 실행 순서 (요약)
| 터미널 | 위치 | 명령 |
|---|---|---|
| 1 | 노트북 | `ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.13 model:=e0509` |
| 2 | 핑키 (`ssh pinky@192.168.4.1`) | `ros2 launch pinky_bringup bringup_robot.launch.xml` |
| 3 | 핑키 (`ssh pinky@192.168.4.1`) | `python3 ~/pinky_signal.py` |
| 4 | 노트북 | `ros2 launch couping_integration integration.launch.py signal_only:=true dsr_bringup:=false` |
| 5 | 노트북 | `ros2 run couping_integration monitor` ← 신호 송수신 현황만 표시 |
| 6 | 노트북 | `ros2 service call /couping/start std_srvs/srv/Trigger` ← 시작 (반복 가능) |

## 1단계. 노트북 설치 (한 번만)

```bash
bash ~/couping_ws/setup_laptop.sh
```

ROS 2 Jazzy 설치, rosdep, 워크스페이스 빌드를 한 번에 합니다. 두산 컨트롤러가 3.x 버전이면 `DRCF_VER=3`을 붙여 실행하세요.

**두산 이더넷 고정 IP** (랜선을 꽂은 뒤 실행):
```bash
sudo nmcli con add type ethernet ifname enp130s0 con-name doosan ipv4.method manual ipv4.addresses 110.120.1.100/24 && sudo nmcli con up doosan
```

**`.bashrc` 변경 사항** (백업: `~/.bashrc.bak_*`)
- 새 터미널을 열면 ROS 2 Jazzy, `couping_ws`, `ROS_DOMAIN_ID=30`이 자동으로 켜집니다. **핑키도 같은 값이어야 합니다.**
- LeRobot venv는 자동으로 켜지지 않습니다. 필요할 때 `il`을 입력하세요. `~/il_ws/scripts/*.sh`는 스스로 ROS 경로를 비우고 venv를 켜므로 그대로 쓰면 됩니다.
- Isaac Sim을 실행할 때는 ROS 경로가 섞이지 않게 `env -u PYTHONPATH ./isaaclab.sh ...`처럼 실행하세요.


**두산 DSR_ROBOT2 패치 (적용 완료)**: 현재 doosan-robot2 jazzy 최신본의 `dsr_common2/imp/DSR_ROBOT2.py`는 그대로 쓰면 import·서비스 연결이 실패합니다. 그래서 두 곳을 직접 고쳤습니다(원본은 `.orig`로 백업).
- `_srv_name_prefix = 'dsr_controller2/'`: 컨트롤러 서비스와 토픽이 `/dsr01/dsr_controller2/...` 아래에 있습니다.
- `SetSingularityHandlingForce` 클라이언트 비활성화: 메시지 정의가 없어서 import가 실패합니다.

이 파일을 고친 뒤에는 `colcon build --packages-select dsr_common2`를 다시 실행해야 합니다. 설치본은 심볼릭 링크가 아니라 복사본이기 때문입니다.

## 2단계. 현장 값 채우기 (`config/couping.yaml`의 TODO)
- 두산: 모델명, 그리퍼 디지털 출력 번호, 관절 포즈 5개(home, pick_approach, pick, place_approach, place). 티치펜던트에서 읽은 값을 넣습니다.
- 핑키: `pick_pose`(피킹 지정 위치), `pack_pose`(OMX 앞), `home_pose`(핑키 대기 위치)를 맵 좌표 `[x, y, yaw]`로 넣습니다.
- OMX: 박스 덮개 닫기 정책 경로를 넣습니다. 정책을 학습하기 전에는 기존 `jun_pumpkin`으로 흐름만 시험합니다.

`--symlink-install`로 빌드했기 때문에 yaml을 고친 뒤 다시 빌드하지 않아도 됩니다.

## 신호 확인 모드 (로봇 연결 + 작업 없이 신호만)
로봇 3대를 실제로 연결한 상태에서, 각 로봇이 신호를 받고 답하는지만 확인합니다. 로봇은 움직이지 않습니다.

| 로봇 | start 를 받으면 | 확인되는 것 |
|---|---|---|
| 두산 | 실제 컨트롤러에서 현재 관절 각도를 읽음 | 컨트롤러와 통신됨 |
| 핑키 | 핑키에서 도는 `pinky_signal`이 수신 로그를 남기고 done 응답 | 노트북↔핑키 통신됨 (주행 안 함) |
| OMX | 모터 6개의 현재 위치를 읽음 (토크·카메라·정책 사용 안 함) | 노트북↔OMX 통신됨 |

1. 핑키에 노드 파일을 복사한 뒤 핑키에서 실행합니다.
   ```bash
   scp ~/couping_ws/src/couping/couping/pinky_signal.py <핑키계정>@<핑키IP>:~/
   ```
   ```bash
   export ROS_DOMAIN_ID=30 && python3 ~/pinky_signal.py
   ```
2. 노트북에서 실행합니다.
   ```bash
   ros2 launch couping_integration integration.launch.py signal_only:=true host:=<두산IP> model:=<모델>
   ```
   ```bash
   ros2 service call /couping/start std_srvs/srv/Trigger
   ```
3. 성공하면 `MOVING_TO_PICK → PICKING → MOVING_TO_PACK → PACKING → DONE` 순서로 진행됩니다. 두산 관절 각도, 핑키 수신 로그, OMX 모터 위치가 각 터미널에 찍힙니다.

## 3단계. 로봇별 단위 테스트

연결 점검(로봇을 움직이지 않음): `bash ~/couping_ws/check_robots.sh`. 실패 항목마다 확인할 내용이 함께 나옵니다.

**두산** (TP-01, TP-04)
```bash
ros2 launch couping_integration integration.launch.py omx:=false host:=110.120.1.13 model:=e0509 rviz:=true
ros2 topic pub --once /couping/picker/cmd std_msgs/msg/String "{data: start}"
```
첫 시험은 `vel: 10`으로 하고, 비상정지 버튼을 손에 들고 진행하세요.

**핑키** (TP-02, TP-05). 핑키에 SSH로 접속한 터미널 2개에서 실행합니다. 핑키의 `ROS_DOMAIN_ID`도 30이어야 합니다.
```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 launch pinky_navigation bringup_launch.xml map:=<맵 이름>
```
노트북에서 실행:
```bash
ros2 topic list | grep -E "odom|navigate"
ros2 launch pinky_navigation nav2_view.launch.xml
```
RViz에서 2D Pose Estimate로 초기 위치를 잡고 2D Goal Pose로 주행을 확인합니다. 이때 화면에 표시되는 좌표가 `pick_pose`, `pack_pose`, `home_pose`의 값이 됩니다.
노트북에서 핑키 토픽이 안 보이면 공용 Wi-Fi가 멀티캐스트를 막고 있는 경우가 많습니다. 핑키 AP(192.168.4.1)에 직접 연결하거나, 양쪽에 `export ROS_STATIC_PEERS=<상대 IP>`를 설정하세요.

**OMX** (TP-03, TP-06). 핑키를 `pack_pose`에 세워 둔 상태에서 실행합니다.
```bash
ros2 run couping_integration omx_packer --ros-args --params-file ~/couping_ws/src/couping/config/couping.yaml
ros2 topic pub --once /couping/packer/cmd std_msgs/msg/String "{data: start}"
```
실행 로그는 `~/couping_ws/log_omx/eval_latest.log`에 남습니다. 손목 카메라 오류가 나면 `~/il_ws/scripts/reset_wrist.sh`를 실행하세요.

## 4단계. 전체 연동 (TP-07, TP-08)
```bash
ros2 launch couping_integration integration.launch.py host:=110.120.1.13 model:=e0509   # 터미널 1
ros2 topic echo /couping/state                                               # 터미널 2
ros2 service call /couping/start std_srvs/srv/Trigger                        # 터미널 3
ros2 service call /couping/stop std_srvs/srv/Trigger                         # 정지할 때
```
- 일부 로봇 없이 시험하려면 `doosan:=false` 또는 `omx:=false`를 붙입니다. 그 단계는 건너뜁니다.
- `/couping/stop`은 핑키 주행과 OMX 정책을 멈춥니다. **두산은 동작 도중 멈출 수 없으니 비상정지 버튼을 쓰세요.**
- 기록: `ros2 bag record /couping/state /couping/picker/status /couping/packer/status`

## 진열대 비전 (RealSense + YOLO, `shelf_vision`)
YOLO 는 `~/venv/yolo`(Python 3.12)에만 있어서 `ros2 run` 대신 launch 로 띄웁니다 (ROS 환경은 그대로 물려받음).
```bash
ros2 launch couping vision.launch.py                       # RealSense (USB 3 포트에 직접)
ros2 launch couping vision.launch.py image_dir:=~/couping_ws/dataset/yolo/images/val   # 카메라 없이 사진으로
ros2 launch couping vision.launch.py preview_rate:=2.0     # 미리보기 그림도 2Hz 로 발행
ros2 launch couping_integration integration.launch.py vision:=true     # 통합 실행에 포함
ros2 service call /couping/vision/detect std_srvs/srv/Trigger   # 인식 1회 → message 에 결과 JSON
```
| 이름 | 종류 | 내용 |
|---|---|---|
| `/couping/vision/detect` | srv `std_srvs/Trigger` | 인식 1회, `message` = 결과 JSON |
| `/couping/vision/cmd` → `/couping/vision/status` | `std_msgs/String` | `start` → `done` / `fail:<이유>` (다른 노드와 같은 방식) |
| `/couping/vision/detections` | `std_msgs/String` (마지막 값 유지) | 결과 JSON: cls, conf, votes, box, xyz(m), depth_ok, row, col |
| `/couping/vision/targets` | `geometry_msgs/PoseArray` | 깊이가 유효한 물체 위치, `camera_color_optical_frame` 기준 |
| `/couping/vision/image/compressed` | `sensor_msgs/CompressedImage` | 결과 그림 (`~/couping_ws/captures/vision_latest.jpg` 에도 저장) |

- 한 번 인식할 때 5장을 찍어 흐린 프레임을 버리고, 3장 이상에서 보인 물체만 남깁니다 (흔들림·순간 가림 대책).
- **로봇팔이 화면 밖 관찰 자세에 멈춘 뒤 호출**해야 합니다 (스트레스 테스트: 흔들림·절반 가림에 약함).
- row/col 은 화면 기준 위→아래, 왼→오른쪽 (1부터). xyz 는 카메라 좌표라 로봇 좌표로 바꾸려면 핸드-아이 캘리브레이션이 필요합니다.
- 모델 교체: `config/couping.yaml` 의 `shelf_vision.weights` 또는 `weights:=` 인자.
- 모델 점검: `~/venv/yolo/bin/python tools/stress_test.py --weights <a.pt> <b.pt>` (조명·거리·기울기·가림 변형별 재현율 비교)

### 카메라 → 로봇 좌표 변환 (핸드-아이 캘리브레이션)
카메라는 고정, 마커는 그리퍼에 붙이는 방식(eye-to-hand). 마커 중심 위치(깊이)와 로봇 TCP 자세(posx)를 12~15쌍 모아
카메라→`base_link` 변환과 마커-TCP 거리를 함께 구합니다 (`couping/handeye.py`, numpy).
1. 마커 인쇄: `~/couping_ws/calib/aruco_4x4_id0_50mm.pdf` 를 **실제 크기(100%)** 로 → 눈금자 100mm 확인 → 그리퍼 끝에 평평하게 부착
2. 두산 bringup 실행 → 아래 도구 실행 (창이 뜸)
   ```bash
   ~/venv/yolo/bin/python -m couping.handeye_calib          # SPACE 저장, u 취소, c 계산·저장, q 종료
   ~/venv/yolo/bin/python -m couping.handeye_calib --compute   # 저장된 표본으로 다시 계산
   ```
3. 로봇을 티치펜던트/직접교시로 옮기고 **멈춘 뒤** SPACE. 진열대 앞 공간 전체에 골고루, **손목을 20~40° 기울인 자세를 절반 이상** 섞기
   (안 기울이면 마커-TCP 거리를 못 구해 좌표가 통째로 어긋남 → 도구가 저장을 막음)
4. `c` → 표본별 오차와 **빼고예측 RMS**(실제 정확도에 가까움) 출력, 5mm 이하면 OK. 결과: `~/couping_ws/calib/handeye.json`
5. `shelf_vision` 이 자동으로 읽어서 `base_mm`(컵 중심, 두산 베이스 mm) 과 `/couping/vision/targets_base`, TF 를 냄. 파일이 바뀌면 다음 인식 때 다시 읽음
6. 검증: 비전이 낸 `base_mm` 위치 위 10cm 로 TCP 를 천천히 옮겨(티치펜던트에서 좌표 입력) 컵 바로 위인지 눈으로 확인

## 주문 처리 (번호로 주문 → 두산 자동 피킹)
신호 흐름: `ros2 run couping_integration order` →`/couping/order`→ 오케스트레이터(대기열) →`/couping/picker/cmd: pick:tomato,salad`→ `doosan_picker`
→ 품목마다 관찰 자세에서 인식 → 수평으로 집기 → 핑키 k번 자리에 놓기 →`/couping/picker/status: progress:… / done / fail:…`

| 번호 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| 품목 | tomato | carrot | salad | bath_tissue | tide | paper_towel |

1. 터미널 3개: 두산 bringup, `ros2 run dsr_gripper gripper_service_a`, 그리고
   ```bash
   ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false omx:=false vision:=true
   ```
2. 주문 터미널: `ros2 run couping_integration order` → `1, 3` 입력 (`1 3`, `13`, `1,1`(두 개)도 가능). `s` 상태, `x` 정지, `r` 재개, `q` 종료
3. 자세 (직접교시로 맞춘 뒤 저장, `~/couping_ws/calib/poses.json`)
   - `ros2 run couping teach_pose front` — 진열대 앞 수평 자세 (필수)
   - `ros2 run couping teach_pose place_above` / `place` / `place_2` … — 핑키 위 대기, 1번째·2번째… 품목 자리
   - 핑키 자세가 없으면 집은 물건을 **진열대 제자리에 도로 놓음** (`no_place: return`, 반복 시험용)
   - 핑키에 놓는 모드에서 주문 품목 수만큼 자리(place_k)가 없으면 움직이기 전에 거부 (앞 품목 위에 겹쳐 놓지 않도록)
- 진열대에 없는 품목은 건너뛰고 나머지를 집은 뒤 실패로 알림. 실패·정지 후에는 대기열이 멈추고 `r` 로 재개
- `x` 는 지금 품목까지만 하고 멈춤. 동작 중 즉시 정지는 **비상정지 버튼**
- 로봇·카메라 없이 신호 흐름만: `... fake:=true` (품목당 2초). 한 품목만 단독 시험: `ros2 run couping auto_pick tomato --dry-run`
- 속도·그리퍼 값은 `config/couping.yaml` 의 `doosan_picker` (첫 시험은 `vel` 을 낮게)

## 웹 화면 (고객 주문 · 관리자 관제)
`integration.launch.py` 를 띄우면 웹 화면도 같이 뜹니다 (`web:=false` 로 끔, `web_port:=8090` 로 포트 변경).
- 고객: `http://localhost:8080/` — 상품 고르기 → 주문 → 배송 현황(주문 확인 · 상품 담는 중 · 포장 시작 · 배송 시작 · 배송 완료)
- 관리자: `http://localhost:8080/admin` — 들어온 주문·품목별 상태, 두산 진행, 실시간 카메라(화면 켜기), 로봇 상태, 신호 로그, 정지·재개
- 같은 와이파이의 휴대폰에서는 `http://<노트북 IP>:8080/` (실습실 내부망 전용, 로그인 없음)
- 추가 설치 없음: `couping/web_ui.py` 가 ROS 토픽을 받아 HTTP(/events, /camera.mjpg)로 전달

## 다른 PC로 옮기기 (같은 기종·Ubuntu 24.04)
```bash
bash ~/couping_ws/migrate/push_to_pc.sh <user@host> omx_act_jun_pumpkin   # 노트북에서 실행. 보낼 정책 이름을 나열
ssh <user@host>
bash ~/couping_ws/migrate/setup_omx.sh     # LeRobot venv(같은 버전), udev, dialout, .bashrc
bash ~/couping_ws/setup_laptop.sh          # ROS 2 Jazzy + 빌드
```
- venv와 ROS는 복사하지 않고 새로 설치합니다. 패키지 버전은 `migrate/requirements-il.txt`에 고정돼 있습니다.
- Hugging Face 로그인 토큰은 복사하지 않습니다. 데이터셋을 업로드해야 할 때만 그 PC에서 `hf auth login`을 실행하세요.
- 로봇을 움직이는 명령은 SSH 연결이 끊겨도 계속 돌도록 `tmux` 안에서 실행하세요.

## 주의
- OMX ACT 정책은 박스 위치에 민감합니다. 핑키 정차 오차(Nav2 기준 수 cm) 때문에 실패가 잦으면, 정차 위치에 물리 가이드를 두거나 실제로 정차한 상태에서 데이터를 수집하세요.
- 핑키 AP에 직결하면 인터넷이 끊깁니다. `omx_packer`는 `hf_user` 파라미터로 HF 계정명을 넘기므로 오프라인에서도 동작합니다. 정책도 로컬 경로를 쓰세요.
