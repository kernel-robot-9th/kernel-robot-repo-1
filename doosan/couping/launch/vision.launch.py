"""진열대 비전 노드 (shelf_vision) launch.

YOLO(ultralytics)는 ~/venv/yolo 에만 있어서 ros2 run 대신 venv Python 으로 모듈을 실행한다.
ROS 환경(source)은 그대로 물려받으므로 rclpy·메시지는 ROS 것을 쓴다.

예) ros2 launch couping vision.launch.py
    ros2 launch couping vision.launch.py image_dir:=~/couping_ws/dataset/yolo/images/val   # 카메라 없이
    ros2 launch couping vision.launch.py preview_rate:=2.0
    ros2 service call /couping/vision/detect std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def make_process(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    cmd = [os.path.expanduser(get('python')), '-m', 'couping.shelf_vision',
           '--ros-args', '-r', '__node:=shelf_vision', '--params-file', get('config')]
    # 비어 있지 않은 값만 덮어씀 (빈 값은 config 의 값을 그대로 사용)
    for name in ('image_dir', 'weights', 'preview_rate'):
        if get(name):
            cmd += ['-p', f'{name}:={os.path.expanduser(get(name))}']
    return [ExecuteProcess(cmd=cmd, output='screen', name='shelf_vision')]


def generate_launch_description():
    config_default = os.path.join(get_package_share_directory('couping'), 'config', 'couping.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_default),
        DeclareLaunchArgument('python', default_value='~/venv/yolo/bin/python', description='YOLO 가 설치된 venv'),
        DeclareLaunchArgument('image_dir', default_value='', description='카메라 대신 쓸 사진 폴더 (비우면 RealSense)'),
        DeclareLaunchArgument('weights', default_value='', description='모델 경로 (비우면 config 값)'),
        DeclareLaunchArgument('preview_rate', default_value='', description='미리보기 발행 주기 Hz (비우면 config 값)'),
        OpaqueFunction(function=make_process),
    ])
