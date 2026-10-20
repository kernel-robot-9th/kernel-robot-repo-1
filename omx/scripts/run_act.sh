#!/bin/bash
source ~/venv/il/bin/activate

# ===== 확인필요 =====  (환경변수로 주면 그 값을 씀: 연동의 omx_packer 가 config 값으로 넘김)
POLICY_PATH=${POLICY_PATH:-$HOME/outputs/train/bi_omx_act_100ep/checkpoints/100000/pretrained_model}
LEFT_CAM=${LEFT_CAM:-/dev/video4}
RIGHT_CAM=${RIGHT_CAM:-/dev/video6}
HEAD_CAM=${HEAD_CAM:-/dev/video8}
EPISODE_TIME=${EPISODE_TIME:-60}
TASK=${TASK:-"Packaging the box"}
POLICY_DEVICE=${POLICY_DEVICE:-cuda}   # GPU 없는 PC 는 cpu (100걸음마다 약 0.6초 멈췄다 이어감)
# ===================

lerobot-record \
--robot.type=bi_omx_follower \
--robot.left_arm_config.port=/dev/left_omx_follower \
--robot.right_arm_config.port=/dev/right_omx_follower \
--robot.left_arm_config.cameras="{wrist: {type: opencv, index_or_path: $LEFT_CAM, width: 640, height: 480, fps: 30}}" \
--robot.right_arm_config.cameras="{wrist: {type: opencv, index_or_path: $RIGHT_CAM, width: 640, height: 480, fps: 30}}" \
--robot.cameras="{head: {type: opencv, index_or_path: $HEAD_CAM, width: 640, height: 480, fps: 30}}" \
--robot.id=bi_omx_follower \
--policy.path=$POLICY_PATH \
--policy.device=$POLICY_DEVICE \
--dataset.repo_id=local/eval_bi_omx_act_$(date +%m%d_%H%M%S) \
--dataset.single_task="$TASK" \
--dataset.num_episodes=1 \
--dataset.episode_time_s=$EPISODE_TIME \
--dataset.reset_time_s=3 \
--dataset.push_to_hub=false
