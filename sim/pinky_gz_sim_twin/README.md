# pinky_gz_sim 디지털 트윈 추가분

핑키 공식 [pinky_pro](https://github.com/pinklab-art/pinky_pro) 의 `pinky_gz_sim`(커밋 75f76e8) 위에 덮어쓰는 파일들.
`bash install.sh` 로 `~/pinky_pro/src/pinky_pro/pinky_gz_sim` 에 복사하고 빌드한다 (원래 파일은 `~/pinky_pro/backup/` 에 백업).

- `launch/digital_twin.launch.xml`: Gazebo + 핑키(위에 축소 e0509) + OMX 2대 + 실제 로봇 따라 하기 노드
- `scripts/`: `mirror_pinky_pose.py`(핑키 위치), `mirror_dsr_arm.py`(두산 관절), `omx_udp_bridge.py`·`omx_sim_cmd.py`(OMX)
- `params/pinky_link_bridge.yaml`: 실제 도메인 → 시뮬 도메인으로 가져올 토픽 (`/amcl_pose`, `/odom`, `/dsr01/joint_states`)
- `worlds/pinky_factory.world`, `models/`: 진열대·OMX·바닥

실행은 `integration/README.md` 의 '10/30 Gazebo 디지털 트윈'.
