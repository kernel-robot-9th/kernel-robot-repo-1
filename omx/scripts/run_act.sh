#!/bin/bash
source ~/venv/il/bin/activate

# ===== 확인필요 =====
POLICY_PATH=$HOME/outputs/train/bi_omx_act_100ep/checkpoints/100000/pretrained_model
LEFT_CAM=/dev/video4
RIGHT_CAM=/dev/video6
HEAD_CAM=/dev/video8
EPISODE_TIME=60
TASK="Packaging the box"
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
--policy.device=cuda \
--dataset.repo_id=local/eval_bi_omx_act_$(date +%m%d_%H%M%S) \
--dataset.single_task="$TASK" \
--dataset.num_episodes=1 \
--dataset.episode_time_s=$EPISODE_TIME \
--dataset.reset_time_s=3 \
--dataset.push_to_hub=false
