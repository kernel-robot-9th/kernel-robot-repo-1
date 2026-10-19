# bi-omx-lerobot

OMX 양팔(bimanual)을 LeRobot으로 텔레오퍼레이션하고, 학습한 정책(ACT)으로 로봇을 움직이는 방법을 정리한 저장소입니다.

## 구성

- 로봇: OMX leader 2대 + follower 2대 (left / right)
- 카메라: 3대 (left wrist, right wrist, head), 640x480 @30fps
- 소프트웨어: LeRobot(소스 설치), Python 3.10

## 폴더 구조

```
bi-omx-lerobot/
├── README.md
├── scripts/
│   └── run_act.sh          # 학습한 ACT 모델로 로봇 실행
├── lerobot_patch/
│   ├── bi_omx_follower/    # LeRobot에 추가할 양팔 follower 코드
│   ├── bi_omx_leader/      # LeRobot에 추가할 양팔 leader 코드
│   └── register.patch      # 위 코드를 LeRobot에 등록하는 변경분
└── models/
    └── README.md           # 모델 파일 다운로드 링크와 놓는 위치
```

모델 가중치와 데이터셋은 용량이 커서 이 저장소에 직접 올리지 않고, `models/README.md`의 링크로 받습니다.

## 1. 준비 (처음 한 번)

### 1-1. LeRobot 설치

LeRobot을 소스로 설치하고(`pip install -e .`) 가상환경에서 `lerobot-record` 명령이 실행되는지 확인합니다.

### 1-2. bi_omx 코드 추가

이 버전의 LeRobot에는 `bi_omx_follower`, `bi_omx_leader`가 기본 제공되지 않아 직접 추가해야 합니다.

```bash
# LeRobot 패키지 폴더 (robots, teleoperators, scripts가 있는 곳)
LR=$(python -c 'import lerobot, os; print(os.path.dirname(lerobot.__file__))')

cp -r lerobot_patch/bi_omx_follower $LR/robots/
cp -r lerobot_patch/bi_omx_leader $LR/teleoperators/

# LeRobot 저장소 루트에서 등록 변경분 적용
cd <LeRobot 저장소 루트>
git apply <이 저장소 경로>/lerobot_patch/register.patch

python -c 'from lerobot.robots import bi_omx_follower; from lerobot.teleoperators import bi_omx_leader; print("import ok")'
```

`import ok`가 나오면 완료입니다. `git apply`가 실패하면 LeRobot 버전이 다른 것이니 이슈로 알려주세요.

### 1-3. 시리얼 포트 권한과 포트 이름

```bash
sudo usermod -aG dialout $USER   # 적용하려면 로그아웃 후 다시 로그인
```

`/etc/udev/rules.d/99-omx.rules`에 보드 시리얼 번호별로 이름을 붙입니다. `<SERIAL>`에는 각 보드의 실제 시리얼 번호를 넣습니다.

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
models/act_100000/pretrained_model/
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

```bash
bash scripts/run_act.sh
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

## 참고: Gazebo 연동 (선택)

`bi_omx_follower`에는 관절 값을 UDP로 내보내는 `sim_publisher.py`가 들어 있습니다. 환경변수 `OMX_SIM_UDP`(예: `127.0.0.1:5005`)가 설정된 경우에만 동작하며, 설정하지 않으면 아무 영향이 없습니다. 받는 쪽(ROS) 스크립트는 이 저장소에 포함되어 있지 않습니다.
