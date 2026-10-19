"""Couping 1차 연동 launch.

예) ros2 launch couping_integration integration.launch.py host:=110.120.1.13 model:=e0509
    ros2 launch couping_integration integration.launch.py doosan:=false   # 두산 없이 핑키·OMX 만
    ros2 launch couping_integration integration.launch.py signal_only:=true   # 로봇 연결된 상태에서 신호만 주고받기
    ros2 launch couping_integration integration.launch.py vision:=true        # 진열대 비전 노드(RealSense+YOLO)도 함께
    # 두산만 주문 처리 (bringup·그리퍼 서비스는 따로 띄운 상태), 주문은 다른 터미널에서 ros2 run couping_integration order
    ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false omx:=false vision:=true
    # 로봇·카메라 없이 주문 신호 흐름만 시험
    ros2 launch couping_integration integration.launch.py dsr_bringup:=false pinky:=false omx:=false fake:=true
    # 핑키까지 가짜로 전체 흐름 시험 (가짜 Nav2 = fake_pinky)
    ros2 launch couping_integration integration.launch.py dsr_bringup:=false fake:=true fake_pinky:=true
핑키 쪽 bringup·navigation 은 핑키에서 따로 실행합니다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_default = os.path.join(get_package_share_directory('couping_integration'), 'config', 'integration.yaml')
    doosan_default = os.path.join(get_package_share_directory('couping'), 'config', 'couping.yaml')
    config = LaunchConfiguration('config')            # 오케스트레이터 · OMX
    doosan_config = LaunchConfiguration('doosan_config')  # 두산 피킹 · 비전
    doosan = LaunchConfiguration('doosan')
    omx = LaunchConfiguration('omx')
    signal_only = ParameterValue(LaunchConfiguration('signal_only'), value_type=bool)
    fake = ParameterValue(LaunchConfiguration('fake'), value_type=bool)

    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('dsr_bringup2'), 'launch', 'dsr_bringup2_rviz.launch.py')),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'host': LaunchConfiguration('host'),
            'model': LaunchConfiguration('model'),
            'name': LaunchConfiguration('robot_name'),
            'gui': LaunchConfiguration('rviz'),
        }.items(),
        # 두산 bringup 을 다른 터미널에서 이미 띄웠으면 dsr_bringup:=false
        condition=IfCondition(PythonExpression(["'", doosan, "' == 'true' and '", LaunchConfiguration('dsr_bringup'), "' == 'true'"])))

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_default),
        DeclareLaunchArgument('doosan_config', default_value=doosan_default),
        DeclareLaunchArgument('doosan', default_value='true', description='두산 bringup + picker 실행'),
        DeclareLaunchArgument('omx', default_value='true', description='OMX packer 실행'),
        DeclareLaunchArgument('dsr_bringup', default_value='true', description='두산 bringup 포함 (이미 따로 띄웠으면 false)'),
        DeclareLaunchArgument('signal_only', default_value='false',
                              description='신호만 주고받기: 두산은 관절 각도, OMX 는 모터 위치만 읽고, 핑키는 주행 대신 pinky_signal 응답'),
        DeclareLaunchArgument('pinky', default_value='true', description='핑키 주행 단계 포함 (false 면 건너뜀)'),
        DeclareLaunchArgument('fake', default_value='false', description='두산 picker · OMX 를 로봇·카메라 없이 흉내 (신호 흐름 시험)'),
        DeclareLaunchArgument('fake_pinky', default_value='false',
                              description='실제 핑키 대신 가짜 Nav2(fake_pinky) 실행. 실제 핑키와 같은 ROS_DOMAIN_ID 에서 쓰지 말 것'),
        DeclareLaunchArgument('manual_transfer', default_value='false',
                              description='핑키 없이: 피킹 뒤 사람이 박스를 OMX 로 옮기고 관리자 화면에서 확인 (pinky:=false 와 함께)'),
        DeclareLaunchArgument('mode', default_value='real', description='real 또는 virtual(에뮬레이터, Docker 필요)'),
        DeclareLaunchArgument('host', default_value='110.120.1.13', description='두산 IP = 110.120.1.<시리얼 번호 숫자> (로봇마다 다름)'),
        DeclareLaunchArgument('model', default_value='e0509'),
        DeclareLaunchArgument('robot_name', default_value='dsr01'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('vision', default_value='false', description='진열대 비전 노드 실행 (RealSense + YOLO)'),
        DeclareLaunchArgument('web', default_value='true', description='고객·관리자 웹 화면 (http://<IP>:8080/, /admin)'),
        DeclareLaunchArgument('web_port', default_value='8080'),

        dsr_bringup,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('couping'), 'launch', 'vision.launch.py')),
            launch_arguments={'config': doosan_config}.items(),
            condition=IfCondition(LaunchConfiguration('vision'))),
        Node(package='couping', executable='doosan_picker', namespace=LaunchConfiguration('robot_name'),
             parameters=[doosan_config, {'robot_model': LaunchConfiguration('model'),
                                  'signal_only': signal_only, 'fake': fake}],
             output='screen', condition=IfCondition(doosan)),
        Node(package='couping_integration', executable='omx_packer',
             parameters=[config, {'signal_only': signal_only, 'fake': fake}],
             output='screen', condition=IfCondition(omx)),
        Node(package='couping_integration', executable='fake_pinky', parameters=[config], output='screen',
             condition=IfCondition(LaunchConfiguration('fake_pinky'))),
        Node(package='couping_integration', executable='web_ui', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int)}],
             condition=IfCondition(LaunchConfiguration('web'))),
        # 실행하지 않은 로봇의 단계는 오케스트레이터도 건너뜀
        Node(package='couping_integration', executable='orchestrator', output='screen',
             parameters=[config, {'signal_only': signal_only,
                                  'enable_picker': ParameterValue(doosan, value_type=bool),
                                  'enable_pinky': ParameterValue(LaunchConfiguration('pinky'), value_type=bool),
                                  'enable_packer': ParameterValue(omx, value_type=bool),
                                  'manual_transfer': ParameterValue(LaunchConfiguration('manual_transfer'), value_type=bool)}]),
    ])
