from glob import glob

from setuptools import setup

package_name = 'couping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omen',
    maintainer_email='iyuchan90@gmail.com',
    description='Couping 1차 연동: 두산 → 핑키 → OMX',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'doosan_picker = couping.doosan_picker:main',
            'omx_packer = couping.omx_packer:main',
            'orchestrator = couping.orchestrator:main',
            'pinky_signal = couping.pinky_signal:main',
            'monitor = couping.monitor:main',
            'order = couping.order_terminal:main',
            'auto_pick = couping.auto_pick:main',
            'teach_pose = couping.teach_pose:main',
            'box_poses = couping.box_poses:main',
            'web_ui = couping.web_ui:main',
        ],
    },
)
