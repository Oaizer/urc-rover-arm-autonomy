from glob import glob
from setuptools import setup
setup(name='urc_autonomy', version='0.1.0', packages=['urc_autonomy'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/urc_autonomy']),
                  ('share/urc_autonomy', ['package.xml']),
                  ('share/urc_autonomy/config', glob('config/*.json'))],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='URC team', maintainer_email='team@example.com',
      description='ROS perception, motion and hover mission',
      entry_points={'console_scripts': [
          'arm_controller = urc_autonomy.motion_node:main',
          'tag_mapper = urc_autonomy.mapping_node:main',
          'hover_mission = urc_autonomy.mission_node:main']})
