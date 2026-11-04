# 노트북·핑키 ROS 통신 설정
# 사용: source ~/couping_ws/dds_env.sh   (ros2 실행 전에)
# 핑키 pinky-26116(192.168.111.102)은 CycloneDDS 가 없어 기본 FastDDS 를 씀 → 노트북도 FastDDS (10/21)
# 예전 핑키(pinky-26119, CycloneDDS)를 쓸 때는: COUPING_DDS=cyclone source ~/couping_ws/dds_env.sh
export ROS_DOMAIN_ID=30
if [ "${COUPING_DDS:-fastdds}" = "cyclone" ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlo1"/></Interfaces></General></Domain></CycloneDDS>'
else
  unset RMW_IMPLEMENTATION CYCLONEDDS_URI
fi
