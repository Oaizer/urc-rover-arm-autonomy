"""Visual approximations only. Joint transforms exactly match the IK model.

No invented inertias or claim of collision/physics fidelity. The camera mount is
included only in the explicitly synthetic simulation description.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from urc_kinematics.core import AXES, load_config

CAMERA_TRANSLATION = np.array([0.0, 0.05, 0.0])
CAMERA_ROTATION = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])
SIM_START = np.array([0., .5, -1., 0., .5, 0.])


def nums(values):
    return ' '.join(format(float(x), '.12g') for x in values)


def origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    ET.SubElement(parent, 'origin', xyz=nums(xyz), rpy=nums(rpy))


def visual_box(link, size, center, color, rpy=(0, 0, 0)):
    visual = ET.SubElement(link, 'visual')
    origin(visual, center, rpy)
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size=nums(size))
    material = ET.SubElement(visual, 'material', name='color_' + '_'.join(map(str, color)))
    ET.SubElement(material, 'color', rgba=nums(color))


def segment(link, a, b, color):
    a, b = np.array(a), np.array(b)
    delta = b - a
    length = np.linalg.norm(delta)
    if length < 1e-9:
        return
    # Box's long local X axis points along the segment.
    x = delta / length
    helper = np.array([0., 0., 1.]) if abs(x[2]) < .9 else np.array([0., 1., 0.])
    y = np.cross(helper, x); y /= np.linalg.norm(y)
    z = np.cross(x, y)
    rpy = Rotation.from_matrix(np.column_stack([x, y, z])).as_euler('xyz')
    visual_box(link, [length, .024, .024], (a + b) / 2, color, rpy)


def fixed(robot, name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    joint = ET.SubElement(robot, 'joint', name=name, type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    origin(joint, xyz, rpy)


def make_urdf(config_path, include_sim_camera=False, control_backend=None):
    model, config = load_config(config_path)
    robot = ET.Element('robot', name='urc_yzzxzx')
    world = ET.SubElement(robot, 'link', name='world')
    base = ET.SubElement(robot, 'link', name=config['base_frame'])
    visual_box(base, [.13, .035, .13], [0, -.025, 0], [.18, .20, .24, 1])
    fixed(robot, 'world_to_legacy', 'world', config['base_frame'], rpy=[np.pi / 2, 0, 0])
    names = ['arm_joint1_output', 'arm_joint2_output', 'arm_joint3_output',
             'arm_joint4_output', 'arm_joint5_output', 'arm_joint6_output']
    parent = config['base_frame']
    for index, (name, axis, offset) in enumerate(zip(names, AXES, model.geometry.offsets)):
        link = ET.SubElement(robot, 'link', name=name)
        color = [.15, .55, .78, 1] if index % 2 == 0 else [.7, .74, .78, 1]
        if index == 2:
            corner = [0, model.geometry.L3y, 0]
            segment(link, [0, 0, 0], corner, color)
            segment(link, corner, offset, color)
        else:
            segment(link, [0, 0, 0], offset, color)
        visual_box(link, [.035, .035, .035], [0, 0, 0], [.24, .26, .30, 1])
        joint = ET.SubElement(robot, 'joint', name=f'joint_{index+1}', type='revolute')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name)
        origin(joint, [0, 0, 0] if index == 0 else model.geometry.offsets[index-1])
        ET.SubElement(joint, 'axis', xyz=nums(axis))
        ET.SubElement(joint, 'limit', lower=str(model.lower[index]), upper=str(model.upper[index]),
                      effort='1', velocity='0.35')  # visualization placeholders, never hardware limits
        parent = name
    ET.SubElement(robot, 'link', name='arm_chain_end')
    fixed(robot, 'chain_end', parent, 'arm_chain_end', model.geometry.offsets[-1])
    tool = ET.SubElement(robot, 'link', name=config['tcp_frame'])
    visual_box(tool, [.015, .015, .015], [0, 0, 0], [.95, .35, .12, 1])
    fixed(robot, 'tool_mount', 'arm_chain_end', config['tcp_frame'], model.tool_p,
          Rotation.from_matrix(model.tool_r).as_euler('xyz'))
    if include_sim_camera:
        camera = ET.SubElement(robot, 'link', name='sim_camera_optical_frame')
        visual_box(camera, [.036, .026, .030], [0, 0, 0], [.18, .20, .22, 1])
        fixed(robot, 'sim_camera_mount', 'arm_joint5_output', 'sim_camera_optical_frame',
              CAMERA_TRANSLATION, [np.pi, -np.pi/2, 0])
    if control_backend is not None:
        if control_backend != 'mock':
            raise ValueError('Only the non-actuating mock ros2_control backend is commissioned')
        control = ET.SubElement(robot, 'ros2_control', name='URCMockArm', type='system')
        hardware = ET.SubElement(control, 'hardware')
        ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
        for index in range(6):
            joint = ET.SubElement(control, 'joint', name=f'joint_{index+1}')
            ET.SubElement(joint, 'command_interface', name='position')
            ET.SubElement(joint, 'command_interface', name='velocity')
            position = ET.SubElement(joint, 'state_interface', name='position')
            ET.SubElement(position, 'param', name='initial_value').text = nums([SIM_START[index]])
            ET.SubElement(joint, 'state_interface', name='velocity')
    return ET.tostring(robot, encoding='unicode')
