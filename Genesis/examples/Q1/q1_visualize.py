"""
Simple script to visualize the Q1 robot in its default pose.
This helps verify the robot loads correctly before training.
"""

import genesis as gs
import numpy as np
import time

def main():
    # Initialize Genesis
    gs.init()
    
    # Create scene
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01, substeps=2),
        viewer_options=gs.options.ViewerOptions(
            max_FPS=60,
            camera_pos=(2.0, -2.0, 1.5),
            camera_lookat=(0.0, 0.0, 0.3),
            camera_fov=40,
        ),
        show_viewer=True,
    )
    
    # Add plane
    plane = scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))
    
    # Add Q1 robot
    # Apply 90-degree rotation around X-axis to correct orientation
    # quat format: (w, x, y, z) - this rotates the entire robot, not just visuals
    import math
    robot = scene.add_entity(
        gs.morphs.MJCF(
            file="xml/q1/q1_mjx_full.xml",
            pos=(0, 0, 0.5),
            quat=(math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0),  # 90° around X-axis
        ),
    )

    # robot = scene.add_entity(
    #     gs.morphs.URDF(
    #         file="urdf/q1/kutta.urdf",
    #         pos=(0, 0, 0.5),
    #         quat=(math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0),
    #     ),
    # )
    
    # Build scene
    scene.build()
    
    # Print robot information
    print("\n" + "="*60)
    print("Q1 Robot Information")
    print("="*60)
    
    # Check actual orientation
    actual_quat = robot.get_quat()
    print(f"\nRobot actual quaternion: {actual_quat}")
    if actual_quat.dim() == 1:
        print(f"  (w={actual_quat[0]:.4f}, x={actual_quat[1]:.4f}, y={actual_quat[2]:.4f}, z={actual_quat[3]:.4f})")
    else:
        print(f"  (w={actual_quat[0, 0]:.4f}, x={actual_quat[0, 1]:.4f}, y={actual_quat[0, 2]:.4f}, z={actual_quat[0, 3]:.4f})")
    
    # Convert to euler for easier understanding
    from genesis.utils.geom import quat_to_xyz
    if actual_quat.dim() == 1:
        actual_quat = actual_quat.unsqueeze(0)  # Add batch dimension
    euler = quat_to_xyz(actual_quat, rpy=True, degrees=True)
    print(f"Robot euler angles (degrees): roll={euler[0, 0]:.2f}, pitch={euler[0, 1]:.2f}, yaw={euler[0, 2]:.2f}")
    print(f"\nNote: If robot appears rotated visually, it may be due to how the mesh STL files")
    print(f"were exported. The quaternion above shows the actual physics orientation.")
    print(f"Expected: roll≈0, pitch≈0, yaw≈0 for upright orientation")
    
    # Joint names and their default positions
    joint_names = [
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
    ]
    
    default_positions = {
        "FL_thigh_joint": 0.45,
        "FL_calf_joint": 0.15,
        "FR_thigh_joint": 0.45,
        "FR_calf_joint": 0.15,
        "RL_thigh_joint": 0.25,
        "RL_calf_joint": 0.15,
        "RR_thigh_joint": 0.25,
        "RR_calf_joint": 0.15,
    }
    
    print(f"\nNumber of joints: {len(joint_names)}")
    print("\nJoint Configuration:")
    for name in joint_names:
        try:
            joint = robot.get_joint(name)
            print(f"  {name:20s} - DOF index: {joint.dof_start}, Default: {default_positions[name]:6.3f} rad")
        except Exception as e:
            print(f"  {name:20s} - ERROR: {e}")
    
    # Set default pose
    # Get DOF indices - for MuJoCo models with floating base, need to add 6
    # The floating base has 6 DOFs (3 position + 3 orientation)
    motors_dof_idx = [robot.get_joint(name).dof_start for name in joint_names]
    # Fix the first index if it's 0 (should be 6 to account for floating base)
    if motors_dof_idx[0] == 0:
        motors_dof_idx[0] = 6
    print(motors_dof_idx)
    default_dof_pos = [default_positions[name] for name in joint_names]
    
    robot.set_dofs_position(
        position=default_dof_pos,
        dofs_idx_local=motors_dof_idx,
    )
    
    # Step once to apply the positions
    # scene.step()
    
    print(f"\nRobot base position: {robot.get_pos()}")
    print(f"Robot base quaternion: {robot.get_quat()}")
    
    print("\n" + "="*60)
    print("Controls:")
    print("  - Use mouse to rotate view")
    print("  - Scroll to zoom")
    print("  - Press ESC or close window to exit")
    print("\nNOTE: FL and FR legs may appear at slightly different angles")
    print("even with the same joint values due to mirrored kinematics.")
    print("This is normal - left and right legs are mirror images in the XML.")
    print("="*60 + "\n")
    
    # Run simulation
    for i in range(15000):
        # Hold the robot in default pose
        robot.control_dofs_position(default_dof_pos, motors_dof_idx)
        scene.step()
    
    print("Visualization complete!")

if __name__ == "__main__":
    main()
