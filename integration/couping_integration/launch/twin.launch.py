"""Gazebo 디지털 트윈(pinky_gz_sim)을 arena_map 기준 보정값으로 실행.

pinky_gz_sim 의 digital_twin.launch.xml 기본 보정값은 다른 맵(mapping_2) 기준이라,
팀 맵 amr/pinky_navigation/map/arena_map 좌표(integration.yaml 의 pose 들)를 쓰면 가상 핑키가 엉뚱한 곳에 선다.
여기의 offset 은 arena_map 의 벽을 Gazebo 월드(pinky_factory.world)의 벽에 맞춘 값 (평균 오차 약 2cm):
    gazebo = R(offset_yaw) * map + (offset_x, offset_y)

예) 가짜 주문 흐름(연동 ROS_DOMAIN_ID=87)을 따라 움직이기
    ros2 launch couping_integration twin.launch.py real_domain:=87
    실제 로봇(도메인 0)을 따라 움직이기
    ros2 launch couping_integration twin.launch.py real_domain:=0
pinky_gz_sim 패키지(~/pinky_pro)가 빌드·source 되어 있어야 한다.
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
            DeclareLaunchArgument('sim_domain', default_value='10', description='디지털 트윈 ROS_DOMAIN_ID')]
    args += [DeclareLaunchArgument(k, default_value=v) for k, v in ARENA_OFFSET.items()]
    twin = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pinky_gz_sim'), 'launch', 'digital_twin.launch.xml')),
        launch_arguments={**{k: LaunchConfiguration(k) for k in ('real_domain', 'sim_domain', *ARENA_OFFSET)},
                          'calibrate_on_first': 'false'}.items())
    return LaunchDescription(args + [twin])
