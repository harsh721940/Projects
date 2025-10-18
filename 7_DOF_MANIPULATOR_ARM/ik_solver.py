# ik_solver.py 
from ikpy.chain import Chain
from ikpy.link import URDFLink
import numpy as np

arm_chain = Chain(name='kinova_gen3', links=[
    URDFLink("base",     [0, 0, 0.1], [0, 0, 0], [0, 0, 1]),   #first bracket is distance from 1 joint to another then second bracket is for pitch yaw roll like twisted then orientation
    URDFLink("shoulder", [0, 0, 0.2], [0, 0, 0], [0, 1, 0]),
    URDFLink("elbow",    [0, 0, 0.2], [0, 0, 0], [1, 0, 0]),
    URDFLink("wrist1",   [0, 0, 0.2], [0, 0, 0], [0, 1, 0]),
    URDFLink("wrist2",   [0, 0, 0.1], [0, 0, 0], [1, 0, 0]),
    URDFLink("wrist3",   [0, 0, 0.1], [0, 0, 0], [0, 1, 0]),
    URDFLink("tool",     [0, 0, 0.1], [0, 0, 0], [1, 0, 0]),
])

def solve_ik(target_pos):
    joints = arm_chain.inverse_kinematics(target_position=target_pos)
    return joints

def get_fk_end_effector(joint_angles):
    T = arm_chain.forward_kinematics(joint_angles)
    return T[:3, 3]
