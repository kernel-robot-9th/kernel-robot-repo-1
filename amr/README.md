# 🚗 Pinky Pro 자율주행 (`pinky_navigation` 패키지)

경기장(1.5m × 2.25m) SLAM 매핑(`slam_toolbox`) 및 Nav2 기반 자율주행(`AMCL` + `RegulatedPurePursuitController`) 패키지입니다.
통합 오케스트레이터(`../integration/couping_integration`)의 `/navigate_to_pose` 액션 호출을 받아 **대기 위치(`home_pose`) → 피킹 위치(`pick_pose`) → OMX 포장 위치(`pack_pose`) → 대기 위치** 순서의 자율주행을 수행합니다.

---

## 폴더 구성
| 경로 | 역할 |
|---|---|
| `pinky_navigation/launch/bringup_arena.launch.xml` | 경기장 전용 Nav2 자율주행 런치 (`arena_map.yaml` + `nav2_params_arena.yaml` 자동 로드) |
| `pinky_navigation/launch/map_building_arena.launch.xml` | 경기장 전용 SLAM 매핑 런치 (`mapper_params_arena.yaml` 로드) |
| `pinky_navigation/launch/nav2_view.launch.xml` | Nav2 주행 상태 및 초기 위치(`2D Pose Estimate`) 설정용 RViz2 실행 |
| `pinky_navigation/map/arena_map.yaml`, `arena_map.pgm` | 실측 매핑 완료된 경기장 지도 (해상도 `0.05m/px`) |
| `pinky_navigation/params/nav2_params_arena.yaml` | 핑키 실측 크기(`13cm × 13cm`) 및 소형 경기장 맞춤 Nav2 파라미터 |
| `pinky_navigation/params/mapper_params_arena.yaml` | 소형 경기장용 SLAM Toolbox 파라미터 |

---

## 팀 연동 체크리스트

### 1. 실행 위치 (노트북 / 핑키)
역할에 따라 **핑키 본체(Raspberry Pi 5)**와 **노트북(제어 PC)** 두 곳에서 나누어 실행합니다. (동일한 Wi-Fi 네트워크 및 동일한 `ROS_DOMAIN_ID` 사용 필수)

| 실행 기기 | 실행 대상 | 역할 |
|---|---|---|
| **핑키 (Raspberry Pi 5)** | `pinky_bringup` | 모터 드라이버(`/cmd_vel` 수신, `/odom` 발행) 및 2D LiDAR(`/scan` 발행) |
| **노트북 (Ubuntu 24.04 PC)** | `pinky_navigation` | 지도 서버(`map_server`), 위치 추정(`amcl`), 경로 계획·제어(`Nav2`), RViz2 |

---

### 2. ROS 버전 및 필요한 apt 패키지
- **ROS 버전**: **ROS 2 Jazzy** (Ubuntu 24.04)
- **통신 미들웨어**: `rmw_cyclonedds_cpp` 권장 (`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
- **노트북(PC) 필수 apt 패키지 설치**:
  ```bash
  sudo apt update && sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf-transformations
  ```
  *(참고: 핑키 본체에는 기본 SD 이미지에 `pinky_bringup` 및 `sllidar_ros2`가 이미 설치되어 있습니다.)*

---

### 3. Python 환경
- `pinky_navigation` 패키지는 순수 ROS 2 Launch/설정 패키지(`ament_cmake`)이므로 **별도의 Python 가상환경(venv)이나 추가 `pip` 패키지(`requirements.txt`)가 필요하지 않습니다.**
- 시스템 기본 ROS 2 Jazzy 환경(`source /opt/ros/jazzy/setup.bash`)에서 바로 빌드 및 실행됩니다.

---

### 4. 하드웨어 연결 (포트 이름, udev 규칙, 시리얼 번호)
모든 센서·모터 하드웨어는 노트북이 아닌 **핑키(Raspberry Pi 5) 내부 온보드 포트**에 고정 연결되어 있으므로, 노트북 측에서는 USB 연결이나 udev 규칙 설정이 필요 없습니다.

**핑키(RPi 5) 내부 고정 포트 구성 (`pinky_bringup` 기준)**:
| 하드웨어 | 연결 기기 | 포트 / 장치 경로 | 설정 및 식별 정보 |
|---|---|---|---|
| **2D LiDAR (SLLIDAR C1)** | 핑키 (RPi 5) | `/dev/ttyAMA0` | RPi 온보드 UART0, `DenseBoost` 모드, `frame_id: rplidar_link` (차체 대비 Yaw 180° 장착) |
| **양측 구동 모터 (Dynamixel)** | 핑키 (RPi 5) | `/dev/ttyAMA4` | RPi 온보드 UART4, Baudrate `1000000`, 모터 ID: `1`(좌측 바퀴), `2`(우측 바퀴) |
| **배터리 / 초음파 ADC** | 핑키 (RPi 5) | `/dev/i2c-1` | I2C Bus 1, 주소 `0x08` |
| **전면 카메라 (OV5647)** | 핑키 (RPi 5) | `/dev/media0` | RPi CSI 포트 (`picamera2` / `libcamera`) |

- 온보드 고정 UART(`/dev/ttyAMA0`, `/dev/ttyAMA4`)를 사용하므로 재부팅 시에도 포트 이름이 바뀌지 않으며 별도의 USB 시리얼 번호 udev 규칙이 필요하지 않습니다.

---

### 5. 실행 명령 및 `done` / `fail` 신호 토픽

#### 5-1. 패키지 빌드 (노트북 워크스페이스)
```bash
# 워크스페이스 src 폴더에 복사 후 빌드
cp -r amr/pinky_navigation ~/couping_ws/src/
cd ~/couping_ws
colcon build --packages-select pinky_navigation --symlink-install
source install/setup.bash
```

#### 5-2. 자율주행 실행 순서
1. **[핑키 RPi 5 터미널] 하드웨어 브링업 실행**:
   ```bash
   export ROS_DOMAIN_ID=30   # 팀 통합 도메인 ID로 통일
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ros2 launch pinky_bringup bringup_robot.launch.xml
   ```
2. **[노트북 터미널 1] 경기장 Nav2 자율주행 스택 실행**:
   ```bash
   export ROS_DOMAIN_ID=30
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   source ~/couping_ws/install/setup.bash
   ros2 launch pinky_navigation bringup_arena.launch.xml
   ```
3. **[노트북 터미널 2] RViz2 실행 및 초기 위치(`2D Pose Estimate`) 지정**:
   ```bash
   export ROS_DOMAIN_ID=30
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   source ~/couping_ws/install/setup.bash
   ros2 launch pinky_navigation nav2_view.launch.xml
   ```
   > **주의**: 최초 실행 시 RViz2 상단 **`2D Pose Estimate`** 버튼을 눌러 현재 핑키가 놓인 위치와 바라보는 방향을 맵 위에 한 번 찍어주어야 AMCL 위치 추정이 활성화되고 `/navigate_to_pose` 명령을 수행합니다.

4. **(참고) 경기장 지도를 새로 매핑할 때**:
   ```bash
   # Nav2 대신 SLAM 매핑 런치 실행
   ros2 launch pinky_navigation map_building_arena.launch.xml
   # 매핑 완료 후 지도 저장
   ros2 run nav2_map_server map_saver_cli -f ~/couping_ws/src/pinky_navigation/map/arena_map
   ```

---

#### 5-3. 오케스트레이터 연동 및 `done` / `fail` 신호 방식

통합 오케스트레이터(`couping_integration/orchestrator.py`)와는 **모드에 따라 다음 두 가지 방식**으로 연동됩니다:

##### ① 실제 자율주행 모드 (기본값, `signal_only: false`)
오케스트레이터가 별도의 문자열 토픽 대신 **Nav2 표준 액션 서버**를 직접 호출하여 주행 명령을 내리고 완료(`done`)/실패(`fail`) 결과를 받습니다.
- **액션 이름**: `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- **목표 좌표 설정**: `integration/couping_integration/config/integration.yaml`의 `pick_pose: [x, y, yaw]`(피킹 지정 위치), `pack_pose: [x, y, yaw]`(OMX 앞), `home_pose: [x, y, yaw]`(핑키 대기 위치)
- **완료 / 실패 판정 신호**:
  | 상태 | 반환 신호 (Action GoalStatus) | 오케스트레이터 동작 |
  |---|---|---|
  | **주행 완료 (`done`)** | `GoalStatus.STATUS_SUCCEEDED` (`status == 4`) | 도착 완료로 판정하고 다음 단계(`PACKING` 또는 `DONE`)로 진행 |
  | **주행 실패 (`fail`)** | `STATUS_ABORTED` (`6`) / `STATUS_CANCELED` (`5`) / 타임아웃 | `StageError('nav failed (status=...)')` 발생 및 `ERROR` 상태 전환 |

##### ② 토픽 신호 단독 테스트 모드 (`signal_only: true` 사용 시)
주행 없이 토픽 신호만 주고받을 때(`pinky_signal.py` 등)는 팀 공통 토픽 규약을 따릅니다:
| 토픽 | 타입 | 방향 | 메시지 내용 |
|---|---|---|---|
| `/couping/pinky/cmd` | `std_msgs/msg/String` | 오케스트레이터 → 핑키 | `"start"` (출발 명령), `"stop"` (정지) |
| `/couping/pinky/status` | `std_msgs/msg/String` | 핑키 → 오케스트레이터 | `"running"` → 완료 시 **`"done"`**, 실패 시 **`"fail:<이유>"`** |
