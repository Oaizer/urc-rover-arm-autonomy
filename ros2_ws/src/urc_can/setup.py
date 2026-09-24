from glob import glob
from setuptools import find_packages, setup

setup(
    name='urc_can', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/urc_can']),
        ('share/urc_can', ['package.xml', 'README.md']),
        ('share/urc_can/launch', glob('launch/*.launch.py')),
        ('share/urc_can/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'hardware': ['moteus', 'python-can>=4.0']},
    zip_safe=True,
    maintainer='URC maintainers', maintainer_email='maintainers@example.com',
    description='Simulated or query-only six-joint moteus feedback',
    license='Apache-2.0',
    entry_points={'console_scripts': ['feedback = urc_can.node:main']},
)
