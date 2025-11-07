"""
Visualize the robot's local coordinate axes to debug orientation issues.
This script shows the robot's X (forward), Y (left), and Z (up) directions
using colored arrows/boxes to help identify if the coordinate system is aligned correctly.
"""

import genesis as gs
import numpy as np
import math

def main():
    # Initialize Genesis
    gs.init()
    
    # Create scene
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01, substeps=2),
        viewer_options=gs.options.ViewerOptions(
            max_FPS=60,
            camera_pos=(3.0, -3.0, 2.0),
            camera_lookat=(0.0, 0.0, 0.5),
            camera_fov=40,
        ),
        show_viewer=True,
    )
    
    # Add plane
    plane = scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))
    
    # Test both URDF and MJCF
    print("\n" + "="*80)
    print("Testing coordinate systems for Q1 robot")
    print("="*80)
    
    # Add robot with the spawn quaternion used in training
    use_mjcf = True  # Set to False to test URDF
    
    spawn_quat = (math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0)  # 90° around X-axis
    
    if use_mjcf:
        print("\nUsing MJCF model: xml/q1/q1_mjx_full.xml")
        robot = scene.add_entity(
            gs.morphs.MJCF(
                file="xml/q1/q1_mjx_full.xml",
                pos=(0, 0, 0.5),
                quat=spawn_quat,
            ),
        )
    else:
        print("\nUsing URDF model: urdf/q1/kutta.urdf")
        robot = scene.add_entity(
            gs.morphs.URDF(
                file="urdf/q1/kutta.urdf",
                pos=(0, 0, 0.5),
                quat=spawn_quat,
            ),
        )
    
    # Add visual markers for world coordinate system (at origin)
    print("\nAdding world coordinate axes at origin...")
    
    # World X-axis (RED) - points in world +X direction
    world_x = scene.add_entity(
        morph=gs.morphs.Box(
            size=(1.0, 0.05, 0.05),
            pos=(0.5, 0, 0.025),
            fixed=True,
        ),
        surface=gs.surfaces.Default(
            color=(1.0, 0.0, 0.0, 1.0),
            vis_mode='visual',
        ),
    )
    
    # World Y-axis (GREEN) - points in world +Y direction  
    world_y = scene.add_entity(
        morph=gs.morphs.Box(
            size=(0.05, 1.0, 0.05),
            pos=(0, 0.5, 0.025),
            fixed=True,
        ),
        surface=gs.surfaces.Default(
            color=(0.0, 1.0, 0.0, 1.0),
            vis_mode='visual',
        ),
    )
    
    # World Z-axis (BLUE) - points in world +Z direction
    world_z = scene.add_entity(
        morph=gs.morphs.Box(
            size=(0.05, 0.05, 1.0),
            pos=(0, 0, 0.5),
            fixed=True,
        ),
        surface=gs.surfaces.Default(
            color=(0.0, 0.0, 1.0, 1.0),
            vis_mode='visual',
        ),
    )
    
    # Add robot's local coordinate axes (attached to robot base)
    # These will show where the robot thinks forward/left/up are
    robot_base_pos = (0, 0, 0.5)  # Same as robot spawn position
    
    # Robot Local X-axis (ORANGE/YELLOW) - robot's "forward" direction
    robot_local_x = scene.add_entity(
        morph=gs.morphs.Box(
            size=(0.6, 0.03, 0.03),
            pos=(robot_base_pos[0] + 0.3, robot_base_pos[1], robot_base_pos[2]),
            quat=spawn_quat,  # Rotate with robot
            fixed=True,
        ),
        surface=gs.surfaces.Default(
            color=(1.0, 0.8, 0.0, 1.0),  # Orange/yellow
            vis_mode='visual',
        ),
    )
    
    # Robot Local Y-axis (CYAN) - robot's "left" direction  
    robot_local_y = scene.add_entity(
        morph=gs.morphs.Box(
            size=(0.03, 0.6, 0.03),
            pos=(robot_base_pos[0], robot_base_pos[1] + 0.3, robot_base_pos[2]),
            quat=spawn_quat,  # Rotate with robot
            fixed=True,
        ),
        surface=gs.surfaces.Default(
            color=(0.0, 1.0, 1.0, 1.0),  # Cyan
            vis_mode='visual',
        ),
    )
    
    # Robot Local Z-axis (MAGENTA) - robot's "up" direction
    robot_local_z = scene.add_entity(
        morph=gs.morphs.Box(
            size=(0.03, 0.03, 0.6),
            pos=(robot_base_pos[0], robot_base_pos[1], robot_base_pos[2] + 0.3),
            quat=spawn_quat,  # Rotate with robot
            fixed=True,
        ),
        surface=gs.surfaces.Default(
            color=(1.0, 0.0, 1.0, 1.0),  # Magenta
            vis_mode='visual',
        ),
    )
    
    # Build scene
    scene.build()
    
    # Get robot's orientation
    base_quat = robot.get_quat()
    if base_quat.dim() == 2:
        base_quat = base_quat[0]  # Remove batch dimension if present
    
    print("\n" + "="*80)
    print("ROBOT ORIENTATION ANALYSIS")
    print("="*80)
    
    print(f"\nRobot base quaternion: {base_quat}")
    print(f"  (w={base_quat[0]:.4f}, x={base_quat[1]:.4f}, y={base_quat[2]:.4f}, z={base_quat[3]:.4f})")
    
    # Convert quaternion to rotation matrix to see how axes transform
    from genesis.utils.geom import quat_to_xyz, transform_by_quat
    
    # Get euler angles
    if base_quat.dim() == 1:
        base_quat_batch = base_quat.unsqueeze(0)
    else:
        base_quat_batch = base_quat
    euler = quat_to_xyz(base_quat_batch, rpy=True, degrees=True)
    print(f"\nRobot euler angles (degrees): roll={euler[0, 0]:.2f}, pitch={euler[0, 1]:.2f}, yaw={euler[0, 2]:.2f}")
    
    # Define unit vectors in robot's local frame
    import torch
    local_x = torch.tensor([1.0, 0.0, 0.0], device=base_quat.device)  # Robot's "forward"
    local_y = torch.tensor([0.0, 1.0, 0.0], device=base_quat.device)  # Robot's "left"
    local_z = torch.tensor([0.0, 0.0, 1.0], device=base_quat.device)  # Robot's "up"
    
    # Transform these vectors to world frame using the robot's quaternion
    world_x_dir = transform_by_quat(local_x.unsqueeze(0), base_quat.unsqueeze(0))[0]
    world_y_dir = transform_by_quat(local_y.unsqueeze(0), base_quat.unsqueeze(0))[0]
    world_z_dir = transform_by_quat(local_z.unsqueeze(0), base_quat.unsqueeze(0))[0]
    
    print("\n" + "-"*80)
    print("Robot's Local Axes in World Frame:")
    print("-"*80)
    print(f"Robot Local +X (forward) -> World: [{world_x_dir[0]:7.4f}, {world_x_dir[1]:7.4f}, {world_x_dir[2]:7.4f}]")
    print(f"Robot Local +Y (left)    -> World: [{world_y_dir[0]:7.4f}, {world_y_dir[1]:7.4f}, {world_y_dir[2]:7.4f}]")
    print(f"Robot Local +Z (up)      -> World: [{world_z_dir[0]:7.4f}, {world_z_dir[1]:7.4f}, {world_z_dir[2]:7.4f}]")
    
    # Analyze the forward direction
    print("\n" + "-"*80)
    print("FORWARD DIRECTION ANALYSIS:")
    print("-"*80)
    
    # Determine which world axis the robot's forward (+X) aligns with most
    abs_x = abs(world_x_dir[0].item())
    abs_y = abs(world_x_dir[1].item())
    abs_z = abs(world_x_dir[2].item())
    
    if abs_x > abs_y and abs_x > abs_z:
        if world_x_dir[0] > 0:
            forward_dir = "World +X (correct!)"
            status = "✓ GOOD"
        else:
            forward_dir = "World -X (backward!)"
            status = "✗ PROBLEM"
    elif abs_y > abs_x and abs_y > abs_z:
        if world_x_dir[1] > 0:
            forward_dir = "World +Y (sideways left!)"
            status = "✗ PROBLEM"
        else:
            forward_dir = "World -Y (sideways right!)"
            status = "✗ PROBLEM"
    else:
        if world_x_dir[2] > 0:
            forward_dir = "World +Z (upward!)"
            status = "✗ PROBLEM"
        else:
            forward_dir = "World -Z (downward!)"
            status = "✗ PROBLEM"
    
    print(f"Robot's forward direction points toward: {forward_dir}")
    print(f"Status: {status}")
    
    # Analyze the up direction
    abs_x = abs(world_z_dir[0].item())
    abs_y = abs(world_z_dir[1].item())
    abs_z = abs(world_z_dir[2].item())
    
    if abs_z > abs_x and abs_z > abs_y:
        if world_z_dir[2] > 0:
            up_dir = "World +Z (correct!)"
            up_status = "✓ GOOD"
        else:
            up_dir = "World -Z (upside down!)"
            up_status = "✗ PROBLEM"
    else:
        up_dir = f"World [X={world_z_dir[0]:.2f}, Y={world_z_dir[1]:.2f}, Z={world_z_dir[2]:.2f}] (wrong orientation!)"
        up_status = "✗ PROBLEM"
    
    print(f"\nRobot's up direction points toward: {up_dir}")
    print(f"Status: {up_status}")
    
    print("\n" + "="*80)
    print("VISUAL REFERENCE:")
    print("="*80)
    print("World axes at origin (0, 0, 0):")
    print("  RED     = World +X axis (expected forward direction)")
    print("  GREEN   = World +Y axis (expected left direction)")
    print("  BLUE    = World +Z axis (expected up direction)")
    print("\nRobot's LOCAL axes attached to robot base:")
    print("  ORANGE  = Robot Local +X (where robot thinks 'forward' is)")
    print("  CYAN    = Robot Local +Y (where robot thinks 'left' is)")
    print("  MAGENTA = Robot Local +Z (where robot thinks 'up' is)")
    print("\nThe ORANGE arrow should align with RED for correct forward locomotion!")
    print("="*80)
    
    # Set default joint positions
    joint_names = [
        "FL_thigh_joint", "FL_calf_joint",
        "FR_thigh_joint", "FR_calf_joint",
        "RL_thigh_joint", "RL_calf_joint",
        "RR_thigh_joint", "RR_calf_joint",
    ]
    
    default_positions = [0.45, 0.15, 0.45, 0.15, 0.25, 0.15, 0.25, 0.15]
    
    motors_dof_idx = [robot.get_joint(name).dof_start for name in joint_names]
    if motors_dof_idx[0] == 0:
        motors_dof_idx[0] = 6
    
    robot.set_dofs_position(default_positions, motors_dof_idx)
    
    print("\n" + "="*80)
    print("Press ESC or close window to exit")
    print("Rotate the camera to see the coordinate axes clearly")
    print("="*80 + "\n")
    
    # Run simulation
    for i in range(100000):
        robot.control_dofs_position(default_positions, motors_dof_idx)
        scene.step()
        
        if not scene.viewer.is_alive():
            break
    
    print("Visualization complete!")

if __name__ == "__main__":
    main()
