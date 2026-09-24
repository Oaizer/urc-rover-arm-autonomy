from setuptools import find_packages, setup

setup(
    name='urc_kinematics',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/urc_kinematics']),
        ('share/urc_kinematics', ['package.xml', 'README.md']),
        ('share/urc_kinematics/config', ['config/demo.json']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=False,
    maintainer='URC maintainers',
    maintainer_email='maintainers@example.com',
    description='Service-only SI full-pose IK/FK for a local YZZXZX arm.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['kinematics_node = urc_kinematics.node:main']},
)
