from glob import glob
from setuptools import setup
setup(name='urc_simulation', version='0.1.0', packages=['urc_simulation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/urc_simulation']),
                  ('share/urc_simulation', ['package.xml']),
                  ('share/urc_simulation/launch', glob('launch/*.launch.py')),
                  ('share/urc_simulation/config', glob('config/*'))],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='URC team', maintainer_email='team@example.com',
      description='RViz simulation and image-based ArUco testbed',
      entry_points={'console_scripts': ['simulator = urc_simulation.node:main',
                                        'simulation_clock = urc_simulation.clock_node:main',
                                        'joint_executor = urc_simulation.executor_node:main']})
