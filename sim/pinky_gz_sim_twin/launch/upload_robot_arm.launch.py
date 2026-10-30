import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """pinky_description/launch/upload_robot.launch.py 의 복사본.
    robot.urdf.xacro 대신 pinky_gz_sim/urdf/pinky_with_arm.urdf.xacro 를 사용한다."""
    ld = LaunchDescription()

    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    is_sim = DeclareLaunchArgument("is_sim", default_value="true")
    cam_tilt_deg = DeclareLaunchArgument("cam_tilt_deg", default_value="0")
    arm_scale = DeclareLaunchArgument("arm_scale", default_value="0.2")

    namespace = PythonExpression([
        "'", LaunchConfiguration('namespace'), "' + ('/' if '", LaunchConfiguration('namespace'), "' != '' else '')"
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                get_package_share_directory('pinky_gz_sim'),
                'urdf/pinky_with_arm.urdf.xacro',
            ]),
            ' namespace:=', namespace,
            ' is_sim:=', LaunchConfiguration('is_sim'),
            ' cam_tilt_deg:=', LaunchConfiguration('cam_tilt_deg'),
            ' arm_scale:=', LaunchConfiguration('arm_scale'),
        ]),
        value_type=str,
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'ignore_timestamp': False,
            'use_sim_time': LaunchConfiguration('is_sim'),
            'robot_description': robot_description,
            'frame_prefix': [namespace],
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'source_list': ['joint_states'],
            'rate': 20.0,
            'use_sim_time': LaunchConfiguration('is_sim'),
        }],
        output='screen'
    )

    ld.add_action(namespace_arg)
    ld.add_action(is_sim)
    ld.add_action(cam_tilt_deg)
    ld.add_action(arm_scale)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)

    return ld
