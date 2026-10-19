# omx — OMX 양팔(bimanual) LeRobot 설정과 ACT 실행

OMX 양팔(bimanual)을 LeRobot으로 텔레오퍼레이션하고, 학습한 정책(ACT)으로 로봇을 움직이는 방법을 정리한 폴더입니다.

> `<채우기: ...>` 표시는 사람이 직접 채워야 하는 값입니다.

## 구성

- 로봇: OMX leader 2대 + follower 2대 (left / right)
- 카메라: 3대 (left wrist, right wrist, head), 640x480 @30fps
- 소프트웨어: LeRobot(소스 설치), Python 3.10

## 폴더 구조

```
omx/
├── README.md
├── requirements.txt        # Python 패키지 버전 고정
├── scripts/
│   └── run_act.sh          # 학습한 ACT 모델로 로봇 실행
├── lerobot_patch/
│   ├── bi_omx_follower/    # LeRobot에 추가할 양팔 follower 코드
│   ├── bi_omx_leader/      # LeRobot에 추가할 양팔 leader 코드
│   └── register.patch      # 위 코드를 LeRobot에 등록하는 변경분
└── models/
    └── README.md           # 모델 파일 다운로드 링크와 놓는 위치
```

모델 가중치와 데이터셋은 용량이 커서 저장소에 직접 올리지 않고, `models/README.md`의 링크로 받습니다.

## 1. 준비 (처음 한 번)

아래 명령은 저장소 루트에서 실행하는 것을 기준으로 합니다.

### 1-1. LeRobot 설치

LeRobot을 소스로 설치하고(`pip install -e .`) 가상환경에서 `lerobot-record` 명령이 실행되는지 확인합니다. 설치 버전은 5-3을 참고하세요.

### 1-2. bi_omx 코드 추가

이 버전의 LeRobot에는 `bi_omx_follower`, `bi_omx_leader`가 기본 제공되지 않아 직접 추가해야 합니다.

```bash
# LeRobot 패키지 폴더 (robots, teleoperators, scripts가 있는 곳)
LR=$(python -c 'import lerobot, os; print(os.path.dirname(lerobot.__file__))')

cp -r omx/lerobot_patch/bi_omx_follower $LR/robots/
cp -r omx/lerobot_patch/bi_omx_leader $LR/teleoperators/

# LeRobot 저장소 루트에서 등록 변경분 적용
cd <LeRobot 저장소 루트>
git apply <이 저장소 경로>/omx/lerobot_patch/register.patch

python -c 'from lerobot.robots import bi_omx_follower; from lerobot.teleoperators import bi_omx_leader; print("import ok")'
```

`import ok`가 나오면 완료입니다. `git apply`가 실패하면 LeRobot 버전이 다른 것이니 이슈로 알려주세요.

### 1-3. 시리얼 포트 권한과 포트 이름

```bash
sudo usermod -aG dialout $USER   # 적용하려면 로그아웃 후 다시 로그인
```

`/etc/udev/rules.d/99-omx.rules`에 보드 시리얼 번호별로 이름을 붙입니다. 실제 사용 중인 규칙과 시리얼 번호는 5-4에 있습니다.

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="<SERIAL>", SYMLINK+="left_omx_follower"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="<SERIAL>", SYMLINK+="left_omx_leader"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="<SERIAL>", SYMLINK+="right_omx_follower"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="<SERIAL>", SYMLINK+="right_omx_leader"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/*omx*
```

링크 이름이 맞게 붙었는지는 이름이 아니라 **응답하는 모터 ID**로 확인합니다. leader는 ID 1~6, follower는 ID 11~16이 응답해야 합니다.

### 1-4. 카메라 번호 확인

```bash
lerobot-find-cameras opencv
```

`/dev/videoN` 번호는 PC마다, 재연결할 때마다 바뀔 수 있습니다. 어느 번호가 어느 카메라인지 확인한 뒤 `scripts/run_act.sh`의 값을 고칩니다.

## 2. 모델 파일

`models/README.md`의 링크에서 받아 아래 위치에 둡니다. `model.safetensors` 하나가 아니라 **`pretrained_model` 폴더 전체**가 필요합니다(설정과 정규화 정보 포함).

```
omx/models/act_100000/pretrained_model/
├── model.safetensors
├── config.json
├── train_config.json
└── ... (나머지 파일)
```

## 3. 실행

`scripts/run_act.sh` 맨 위의 "바꿀 값"을 환경에 맞게 고칩니다.

| 값 | 의미 |
|---|---|
| `POLICY_PATH` | `pretrained_model` 폴더 경로 |
| `LEFT_CAM`, `RIGHT_CAM`, `HEAD_CAM` | 카메라 장치 경로 |
| `EPISODE_TIME` | 로봇이 한 번에 움직이는 시간(초) |
| `TASK` | 작업 설명 (학습 때 쓴 문장과 같게) |

스크립트의 `source ~/venv/il/bin/activate` 줄도 가상환경 경로에 맞게 고칩니다.

```bash
bash omx/scripts/run_act.sh
```

실행 전 확인:

- 팔이 직접 움직이므로 작업 공간을 비우고 전원을 바로 끌 수 있는 위치에서 지켜보세요.
- follower를 데이터 수집 때 시작하던 자세와 비슷하게 놓으세요.
- 카메라 이름(`left_wrist`, `right_wrist`, `head`)과 해상도는 학습 때와 같아야 합니다.
- 결과(영상, 행동)는 `~/.cache/huggingface/lerobot/local/eval_bi_omx_act_<날짜시간>`에 저장됩니다.

## 4. 자주 나는 문제

| 증상 | 원인 | 해결 |
|---|---|---|
| `command not found: lerobot-record` | 가상환경이 꺼져 있음 | `source <가상환경>/bin/activate` |
| `Permission denied: /dev/..._omx_...` | `dialout` 그룹에 없음 | 1-3의 `usermod` 후 다시 로그인 |
| `invalid choice: 'bi_omx_follower'` | bi_omx 코드가 없거나 등록 안 됨 | 1-2 다시 확인 |
| `Missing motor IDs 11, 12, 13` | 모터 에러 상태, 전원 또는 케이블 접촉 | 팔을 받치고 로봇 전원 껐다 켜기 |
| leader에서 모터 6개 전부 없음 | leader와 follower 포트 이름이 서로 바뀜 | udev 규칙의 이름을 맞바꾸기 |
| `FileExistsError` | 같은 이름의 결과 폴더가 이미 있음 | 폴더 삭제 또는 이름 변경 (스크립트는 시각이 자동으로 붙음) |
| `Record loop is running slower` | 추론이나 카메라 처리가 목표 30Hz보다 느림 | GPU, CPU 사용률 확인 |

## 5. 팀 연동 체크리스트

### 5-1. 실행 위치

- 실행 PC: `노트북`
- OMX leader 2대, follower 2대, 카메라 3대가 이 PC에 USB로 직접 연결되어야 합니다.
- 추론은 `--policy.device=cuda`로 실행하므로 NVIDIA GPU가 필요합니다. RTX 5080 환경에서는 PyTorch 2.10.0+cu128로 동작을 확인했습니다. 최신 GPU는 CUDA 12.8 이상 빌드의 PyTorch가 필요할 수 있습니다.

### 5-2. ROS 버전, 필요한 apt 패키지

- 팀 통일 버전은 ROS 2 Jazzy입니다. 이 폴더의 코드는 ROS를 사용하지 않고 LeRobot 명령(`lerobot-record`)을 직접 실행하므로, ROS 버전과 무관하게 동작합니다.
- Gazebo 연동(선택): `bi_omx_follower`의 `sim_publisher.py`가 환경변수 `OMX_SIM_UDP`(예: `127.0.0.1:5005`)가 설정된 경우에만 관절 값을 UDP로 내보냅니다. 설정하지 않으면 아무 영향이 없습니다. 받는 쪽(ROS 2 Jazzy의 `omx_udp_bridge.py`)은 이 폴더에 포함되어 있지 않습니다.
- 필수 apt 패키지는 확인된 것이 없습니다. 선택 사항:

| 패키지 | 용도 |
|---|---|
| `v4l-utils` | 카메라 장치 확인 (`v4l2-ctl --list-devices`) |
| `speech-dispatcher` | 녹화 중 음성 안내 |

### 5-3. Python 환경

- Python 3.10 가상환경을 사용합니다. 패키지 버전은 `requirements.txt`(`pip freeze`로 고정)에 있습니다. LeRobot 자체는 소스로 따로 설치하므로 이 파일에 포함하지 않았습니다.
- 사용한 LeRobot 커밋: `8fff0fde7c79f23a93d845d1a50e985de01f8b8a`
- `register.patch`는 이 커밋 기준으로 만들었습니다. 다른 버전에서는 적용되지 않을 수 있습니다.

```bash
python3 -m venv ~/venv/il
source ~/venv/il/bin/activate
pip install -r omx/requirements.txt

git clone https://github.com/huggingface/lerobot.git ~/il_ws/src/lerobot
cd ~/il_ws/src/lerobot
git checkout 8fff0fde7c79f23a93d845d1a50e985de01f8b8a
pip install -e .
```

`requirements.txt` 맨 위에 `--extra-index-url` 줄이 있으면 CUDA 빌드의 PyTorch를 받기 위한 것입니다.

### 5-4. 하드웨어 연결 (포트 이름, udev 규칙, 시리얼 번호)

| 링크 | 역할 | 응답해야 하는 모터 ID |
|---|---|---|
| `/dev/left_omx_leader` | 왼쪽 leader | 1~6 |
| `/dev/right_omx_leader` | 오른쪽 leader | 1~6 |
| `/dev/left_omx_follower` | 왼쪽 follower | 11~16 |
| `/dev/right_omx_follower` | 오른쪽 follower | 11~16 |

실제 사용 중인 udev 규칙(`/etc/udev/rules.d/`의 omx 관련 줄, `ATTRS{serial}`이 보드 시리얼 번호):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="1A6E643A50323952582E3120FF07262C", SYMLINK+="right_omx_follower"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="C42B24BB50323952582E3120FF071C01", SYMLINK+="right_omx_leader"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="B89C3B4250323952582E3120FF050D39", SYMLINK+="left_omx_follower"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{serial}=="4283A52B50323952582E3120FF072628", SYMLINK+="left_omx_leader"
```

- 보드를 바꾸면 시리얼 번호가 달라져 규칙을 고쳐야 합니다. 새 보드의 시리얼 번호는 `udevadm info -a -n /dev/ttyACM0 | grep serial`로 확인합니다.
- 카메라 기본 장치: 왼쪽 손목 `/dev/video4`, 오른쪽 손목 `/dev/video6`, head `/dev/video8` (번호는 바뀔 수 있음, 1-4 참고)

### 5-5. 실행 명령, done / fail 신호

실행 명령(저장소 루트에서):

```bash
bash omx/scripts/run_act.sh
```

done / fail 신호: 현재 이 스크립트는 ROS 토픽으로 `done`/`fail`을 발행하지 않습니다.

- 에피소드 시간(`EPISODE_TIME`)이 지나거나 키보드 오른쪽 방향키를 누르면 에피소드가 끝나고 프로세스가 종료됩니다.
- 오류가 나면 에러 메시지와 함께 비정상 종료합니다. 종료 코드로 구분하는 방식은 아직 검증하지 않았습니다.
- 학습된 정책은 작업 성공 여부를 스스로 판단하지 않습니다. 토픽 연동이 필요하면 팀에서 정한 토픽 이름, 메시지 타입, 성공 판단 기준에 맞춰 발행하는 래퍼 노드를 추가해야 합니다.

### 5-6. 캘리브레이션 파일

`omx/calibration/`의 `bi_omx_follower_left.json`, `bi_omx_follower_right.json`은 학습 때 사용한 값입니다(공장 기본값: homing_offset 0, range 0~4095). 실행하는 PC의 `~/.cache/huggingface/lerobot/calibration/robots/omx_follower/`에 복사하세요: `mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/omx_follower && cp omx/calibration/*.json ~/.cache/huggingface/lerobot/calibration/robots/omx_follower/`. 없으면 처음 실행할 때 같은 기본값으로 자동 생성됩니다.
