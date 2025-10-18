# arm_model.py
import numpy as np

# DH Parameters for Kinova Gen3-like 7-DOF arm (approximate)
# Format: [theta , d, a, alpha]
DH_PARAMS = [
    [0, 0.1564, 0,         np.pi/2],
    [0, 0,      0.1284,    0],
    [0, 0,      0.2104,    0],
    [0, 0.2104, 0,         np.pi/2],
    [0, 0,      0,        -np.pi/2],
    [0, 0.2084, 0,         np.pi/2],
    [0, 0.1059, 0,         0]
]

def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(joint_angles):       #takes 7 joint angles as we have 7dof 
    """
    Forward Kinematics using DH model
    :param joint_angles: 7 joint angles in radians
    :return: 4x4 transformation matrix
    """
    assert len(joint_angles) == 7           #cheaks that only 7 joint angls are given 
    T = np.eye(4)                       #makes a base position of robot before adding any joints
    for i in range(7):
        θ = joint_angles[i]
        d, a, α = DH_PARAMS[i][1], DH_PARAMS[i][2], DH_PARAMS[i][3]
        T_i = dh_transform(θ, d, a, α)
        T = np.dot(T, T_i)
    return T
