"""
Visualize the Q1 robot's coordinate system to check alignment.
Prints detailed orientation information.
"""

import genesis as gs
import math
import numpy as np

# Initialize Genesis
gs.init()

# Create scene
scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.02, substeps=2),
    viewer_options=gs.options.ViewerOptions(
        max_FPS=60,
        camera_pos=(1.5, -1.5, 1.0),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
    ),
    show_viewer=True,
)

# Add plane
plane = scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

# Add Q1 robot with the training spawn quaternion
robot = scene.add_entity(
    gs.morphs.URDF(
        file="urdf/q1/kutta.urdf",
        pos=(0, 0, 0.5),
        quat=(math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0),  # Training spawn quat
    ),
)

# Build scene
scene.build()

# Set default standing pose
init_dof_pos = [
    0.45, 0.15,  # FL: thigh, calf
    0.45, 0.15,  # FR: thigh, calf
    0.25, 0.15,  # RL: thigh, calf
    0.25, 0.15,  # RR: thigh, calf
]

joint_names = [
    "FL_thigh_joint", "FL_calf_joint",
    "FR_thigh_joint", "FR_calf_joint",
    "RL_thigh_joint", "RL_calf_joint",
    "RR_thigh_joint", "RR_calf_joint",
]

motors_dof_idx = [robot.get_joint(name).dof_start for name in joint_names]
robot.set_dofs_position(init_dof_pos, motors_dof_idx)

print("="*80)
print("COORDINATE SYSTEM ANALYSIS")
print("="*80)
print("\nRobot spawned with quaternion: (0.707, -0.707, 0, 0)")
print("This is a 90-degree rotation around X-axis")

# Get robot pose
base_pos = robot.get_pos()
base_quat = robot.get_quat()

print(f"\nRobot base position: {base_pos}")
print(f"Robot base quaternion: {base_quat}")

# Transform axes according to robot orientation
from genesis.utils.geom import transform_by_quat, inv_quat, quat_to_xyz
import torch

# Handle both batched and non-batched tensors
if base_quat.dim() == 1:
    base_quat_batch = base_quat.unsqueeze(0)
else:
    base_quat_batch = base_quat

# Robot's local axes in robot frame
local_x = torch.tensor([[1.0, 0.0, 0.0]], device=gs.device)
local_y = torch.tensor([[0.0, 1.0, 0.0]], device=gs.device)
local_z = torch.tensor([[0.0, 0.0, 1.0]], device=gs.device)

# Transform to world frame - use INVERSE quaternion to go from local to world
inv_base_quat = inv_quat(base_quat_batch)
world_x = transform_by_quat(local_x, inv_base_quat)
world_y = transform_by_quat(local_y, inv_base_quat)
world_z = transform_by_quat(local_z, inv_base_quat)

print(f"\n" + "="*80)
print("ROBOT'S LOCAL COORDINATE AXES IN WORLD FRAME:")
print("="*80)
print(f"Robot Local +X (forward) -> World: {world_x[0].cpu().numpy()}")
print(f"Robot Local +Y (left)    -> World: {world_y[0].cpu().numpy()}")
print(f"Robot Local +Z (up)      -> World: {world_z[0].cpu().numpy()}")

print("\n" + "="*80)
print("EXPECTED FOR PROPER ALIGNMENT:")
print("="*80)
print("  Local +X should map to World [1, 0, 0] or close (robot facing +X)")
print("  Local +Y should map to World [0, 1, 0] or close (robot's left is +Y)")
print("  Local +Z should map to World [0, 0, 1] or close (robot's up is +Z)")

# Convert quaternion to Euler angles
euler = quat_to_xyz(base_quat_batch, rpy=True, degrees=True)
print(f"\n" + "="*80)
print("EULER ANGLES (roll, pitch, yaw):")
print("="*80)
print(f"Roll:  {euler[0, 0].item():7.2f}° (rotation around X)")
print(f"Pitch: {euler[0, 1].item():7.2f}° (rotation around Y)")
print(f"Yaw:   {euler[0, 2].item():7.2f}° (rotation around Z)")
print("\nFor upright robot facing +X:")
print("  Roll  should be ~0°")
print("  Pitch should be ~0°")
print("  Yaw   can be any value")

# Check which world direction the robot is facing
wx = world_x[0].cpu().numpy()
print(f"\n" + "="*80)
print("ROBOT FACING DIRECTION:")
print("="*80)
max_component = np.argmax(np.abs(wx))
direction_names = ['X', 'Y', 'Z']
sign = '+' if wx[max_component] > 0 else '-'
print(f"Robot is primarily facing: {sign}{direction_names[max_component]}")
print(f"  (Robot's local +X aligns most with World {sign}{direction_names[max_component]})")

if max_component == 0 and wx[0] > 0:
    print("✓ GOOD: Robot facing World +X (forward)")
elif max_component == 1:
    print("✗ PROBLEM: Robot facing World ±Y (sideways!)")
elif max_component == 2:
    print("✗ PROBLEM: Robot facing World ±Z (up/down!)")
else:
    print("✗ PROBLEM: Robot facing World -X (backward!)")

print("\n" + "="*80)
print("Press Ctrl+C to exit")
print("="*80)

# Run simulation - just keep robot standing
for i in range(100000):
    robot.control_dofs_position(init_dof_pos, motors_dof_idx)
    scene.step()
