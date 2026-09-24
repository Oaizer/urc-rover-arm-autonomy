from setuptools import setup
setup(name='urc_description', version='0.1.0', packages=['urc_description'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/urc_description']),
                  ('share/urc_description', ['package.xml'])],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='URC team', maintainer_email='team@example.com',
      description='Single-source URDF generator')
