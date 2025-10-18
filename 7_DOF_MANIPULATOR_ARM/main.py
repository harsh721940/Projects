# main.py
from a_star import a_star
from ik_solver import solve_ik, get_fk_end_effector
from visualize import visualize_arm_live
import numpy as np

# Get user-defined grid and obstacles
rows, cols = map(int, input("Enter grid size as rows cols: ").split())
grid = [[0 for _ in range(cols)] for _ in range(rows)]

num_obstacles = int(input("Enter number of obstacles: "))
print("Enter each obstacle as: row col")
for _ in range(num_obstacles):
    r, c = map(int, input().split())
    grid[r][c] = 1

goal = tuple(map(int, input("Enter goal cell as row col: ").split()))
start = (0, 0)

# Run A* algorithm
path = a_star(grid, start, goal)

if not path:
    print(" No path found.")
    exit()

print(" Path found:")
for step in path:
    print(step)

# Convert path to joint angles
grid_resolution = 0.05
z_height = 0.2
joint_history = []

for x, y in path:
    world_pos = [x * grid_resolution, y * grid_resolution, z_height]

    # Prevent IK failure at origin
    if world_pos[0] == 0.0 and world_pos[1] == 0.0:
        world_pos[0] += 0.01
        world_pos[1] += 0.01

    joint_angles = solve_ik(world_pos)
    effector_pos = get_fk_end_effector(joint_angles)

    print(f"\nTarget Position:        {np.round(world_pos, 3)}")
    print(f"FK End Effector Result: {np.round(effector_pos, 3)}")

    joint_history.append(joint_angles)

# Animate motion through all poses
visualize_arm_live(joint_history)
