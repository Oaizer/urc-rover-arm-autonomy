from glob import glob
from setuptools import find_packages, setup

setup(
    name='urc_perception',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/urc_perception']),
        ('share/urc_perception', ['package.xml']),
        ('share/urc_perception/launch', glob('launch/*.launch.py')),
        ('share/urc_perception/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC maintainers',
    maintainer_email='maintainers@example.com',
    description='Latest-frame USB camera and conservative ArUco detector',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'usb_camera = urc_perception.camera_node:main',
        'aruco_detector = urc_perception.detector_node:main',
    ]},
)
