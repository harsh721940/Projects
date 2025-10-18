# visualize.py
import matplotlib.pyplot as plt           #to create lables figures and plots
from mpl_toolkits.mplot3d import Axes3D      #3d plotting supports
from ik_solver import arm_chain               #pre defined robot arm used to compute forward kinematics

def visualize_arm_live(joint_history):              #something like path recostruction
    fig = plt.figure()                            #created new fig window
    ax = fig.add_subplot(111, projection='3d')

    for joint_angles in joint_history:
        frames = arm_chain.forward_kinematics(joint_angles, full_kinematics=True)     #used to extract position of each joints P & O

        xs, ys, zs = [], [], []
        for frame in frames:
            xs.append(frame[0, 3])
            ys.append(frame[1, 3])
            zs.append(frame[2, 3])

        ax.clear()
        ax.plot(xs, ys, zs, '-o', color='blue', markersize=6, label="Arm")
        ax.scatter(xs[-1], ys[-1], zs[-1], c='red', s=80, label="End Effector")
        ax.set_xlim([-0.5, 0.5])                #ensures the camera view stays the same
        ax.set_ylim([-0.5, 0.5])
        ax.set_zlim([0, 0.6])
        ax.set_title("Kinova Gen3 Arm 7DOF ")
        ax.set_xlabel("X") 
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.pause(0.6)

    plt.show()
