#!/bin/bash
source ~/venv/il/bin/activate
# PyTorch 가 CPU 코어 전부(24)를 쓰면 카메라·저장 스레드와 경쟁해 제어 주기가 15Hz 로 떨어짐.
# 6 으로 제한하면 1스텝 43ms → 3ms (10/19 연동 노트북 측정)
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-6}

# ===== 확인필요 =====  (환경변수로 주면 그 값을 씀: 연동의 omx_packer 가 config 값으로 넘김)
POLICY_PATH=${POLICY_PATH:-$HOME/outputs/train/bi_omx_act_100ep/checkpoints/100000/pretrained_model}
LEFT_CAM=${LEFT_CAM:-/dev/video4}
RIGHT_CAM=${RIGHT_CAM:-/dev/video6}
HEAD_CAM=${HEAD_CAM:-/dev/video8}
EPISODE_TIME=${EPISODE_TIME:-60}
TASK=${TASK:-"Packaging the box"}
CAM_FOURCC=${CAM_FOURCC:-}   # 예: MJPG (USB 2 허브에 카메라 여러 대면 기본 형식으로 영상이 안 나올 수 있음)
# ===================
FOURCC_OPT=""
[ -n "$CAM_FOURCC" ] && FOURCC_OPT=", fourcc: $CAM_FOURCC"

lerobot-record \
--robot.type=bi_omx_follower \
--robot.left_arm_config.port=/dev/left_omx_follower \
--robot.right_arm_config.port=/dev/right_omx_follower \
--robot.left_arm_config.cameras="{wrist: {type: opencv, index_or_path: $LEFT_CAM, width: 640, height: 480, fps: 30$FOURCC_OPT}}" \
--robot.right_arm_config.cameras="{wrist: {type: opencv, index_or_path: $RIGHT_CAM, width: 640, height: 480, fps: 30$FOURCC_OPT}}" \
--robot.cameras="{head: {type: opencv, index_or_path: $HEAD_CAM, width: 640, height: 480, fps: 30$FOURCC_OPT}}" \
--robot.id=bi_omx_follower \
--policy.path=$POLICY_PATH \
--policy.device=cuda \
--dataset.repo_id=local/eval_bi_omx_act_$(date +%m%d_%H%M%S) \
--dataset.single_task="$TASK" \
--dataset.num_episodes=1 \
--dataset.episode_time_s=$EPISODE_TIME \
--dataset.reset_time_s=3 \
--dataset.push_to_hub=false
