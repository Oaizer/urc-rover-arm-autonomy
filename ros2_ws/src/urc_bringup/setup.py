from glob import glob
from setuptools import setup

setup(
    name='urc_bringup', version='0.1.0', packages=['urc_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/urc_bringup']),
        ('share/urc_bringup', ['package.xml']),
        ('share/urc_bringup/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='URC team', maintainer_email='team@example.com',
    description='Safe component launch files', license='Apache-2.0',
)
