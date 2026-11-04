"""Gazebo 디지털 트윈(pinky_gz_sim)을 arena_map 기준 보정값으로 실행.

pinky_gz_sim 의 digital_twin.launch.xml 기본 보정값은 다른 맵(mapping_2) 기준이라,
팀 맵 amr/pinky_navigation/map/arena_map 좌표(integration.yaml 의 pose 들)를 쓰면 가상 핑키가 엉뚱한 곳에 선다.
여기의 offset 은 arena_map 의 벽을 Gazebo 월드(pinky_factory.world)의 벽에 맞춘 값 (평균 오차 약 2cm):
    gazebo = R(offset_yaw) * map + (offset_x, offset_y)

실제 로봇 따라 움직이기 (핑키 2호기 = 도메인 2, 먼저 source ~/couping_ws/dds_env.sh 2 로 로봇 쪽과 같은 DDS 설정):
    ros2 launch couping_integration twin.launch.py real_domain:=2
  - 핑키: /amcl_pose·/odom, 두산: /dsr01/joint_states 를 domain_bridge 로 받아 따라 움직임
  - OMX: omx_packer 를 OMX_SIM_UDP=<이 PC IP>:5005 로 띄우면 LeRobot 이 관절 값을 UDP 로 보냄
    (같은 노트북이면 127.0.0.1:5005, 다른 노트북이면 여기서 omx_udp_bind:=0.0.0.0)
pinky_gz_sim 트윈 추가분(sim/pinky_gz_sim_twin/install.sh)이 ~/pinky_pro 에 설치·source 되어 있어야 한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARENA_OFFSET = {'offset_x': '-0.7560', 'offset_y': '0.6280', 'offset_yaw': '-1.5725'}


def generate_launch_description():
    args = [DeclareLaunchArgument('real_domain', default_value='0', description='따라 할 로봇 쪽 ROS_DOMAIN_ID'),
            DeclareLaunchArgument('sim_domain', default_value='10', description='디지털 트윈 ROS_DOMAIN_ID'),
            DeclareLaunchArgument('omx_udp_bind', default_value='127.0.0.1',
                                  description='실제 OMX(LeRobot) UDP 받는 주소. OMX 가 다른 노트북이면 0.0.0.0')]
    args += [DeclareLaunchArgument(k, default_value=v) for k, v in ARENA_OFFSET.items()]
    twin = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pinky_gz_sim'), 'launch', 'digital_twin.launch.xml')),
        launch_arguments={**{k: LaunchConfiguration(k) for k in ('real_domain', 'sim_domain', 'omx_udp_bind', *ARENA_OFFSET)},
                          'calibrate_on_first': 'false'}.items())
    return LaunchDescription(args + [twin])
