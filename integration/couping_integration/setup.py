from glob import glob

from setuptools import setup

package_name = 'couping_integration'

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
    description='Couping 연동: 주문 대기열 · 두산 → 핑키 → OMX 단계 전환 · 주문자/관리자 웹 UI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'orchestrator = couping_integration.orchestrator:main',
            'web_ui = couping_integration.web_ui:main',
            'order = couping_integration.order_terminal:main',
            'monitor = couping_integration.monitor:main',
            'omx_packer = couping_integration.omx_packer:main',
            'pinky_signal = couping_integration.pinky_signal:main',
            'fake_pinky = couping_integration.fake_pinky:main',
        ],
    },
)
