"""
Diagnostic script to visualize Q1 robot sensor readings in real-time.
This helps verify that sensors are properly oriented and working correctly.
"""

import genesis as gs
import numpy as np
import torch
import math
from genesis.utils.geom import quat_to_xyz, transform_by_quat, inv_quat, transform_quat_by_quat

def print_sensor_readings(robot, step):
    """Print all sensor readings in a formatted way"""
    
    # URDF coordinate correction - rotate sensor readings by this
    # Robot spawns with (0.707, -0.707, 0, 0) to look upright
    # But we need to rotate sensor frame to get correct IMU readings
    sensor_correction_quat = torch.tensor(
        [[math.cos(math.pi/4), math.sin(math.pi/4), 0, 0]],  # +90° rotation around X
        device=gs.device
    )
    
    # Base pose
    base_pos = robot.get_pos()
    base_quat = robot.get_quat()
    
    # Ensure we're working with batched tensors
    if base_pos.dim() == 1:
        base_pos = base_pos.unsqueeze(0)
    if base_quat.dim() == 1:
        base_quat = base_quat.unsqueeze(0)
    
    # Apply sensor coordinate correction to quaternion
    corrected_quat = transform_quat_by_quat(base_quat, sensor_correction_quat)
    
    # Convert quaternion to euler angles for easier understanding
    euler = quat_to_xyz(corrected_quat, rpy=True, degrees=True)
    
    # Base velocities (global frame)
    base_vel_global = robot.get_vel()
    base_ang_global = robot.get_ang()
    
    # Ensure velocities are batched
    if base_vel_global.dim() == 1:
        base_vel_global = base_vel_global.unsqueeze(0)
    if base_ang_global.dim() == 1:
        base_ang_global = base_ang_global.unsqueeze(0)
    
    # Base velocities (local/body frame) - with coordinate correction
    inv_base_quat = inv_quat(corrected_quat)
    base_vel_local = transform_by_quat(base_vel_global, inv_base_quat)
    base_ang_local = transform_by_quat(base_ang_global, inv_base_quat)
    
    # Projected gravity (what the IMU would sense) - with coordinate correction
    global_gravity = torch.tensor([[0.0, 0.0, -1.0]], device=gs.device, dtype=gs.tc_float)
    projected_gravity = transform_by_quat(global_gravity, inv_base_quat)
    
    # Joint states
    joint_names = [
        "FL_thigh_joint", "FL_calf_joint",
        "FR_thigh_joint", "FR_calf_joint",
        "RL_thigh_joint", "RL_calf_joint",
        "RR_thigh_joint", "RR_calf_joint",
    ]
    motors_dof_idx = [robot.get_joint(name).dof_start for name in joint_names]
    dof_pos = robot.get_dofs_position(motors_dof_idx)
    dof_vel = robot.get_dofs_velocity(motors_dof_idx)
    
    # Convert tensors to numpy for printing
    def t2n(tensor):
        if isinstance(tensor, torch.Tensor):
            t = tensor.cpu().numpy()
            return t[0] if t.ndim > 1 else t
        return tensor
    
    base_pos = t2n(base_pos)
    euler = t2n(euler)
    base_vel_global = t2n(base_vel_global)
    base_ang_global = t2n(base_ang_global)
    base_vel_local = t2n(base_vel_local)
    base_ang_local = t2n(base_ang_local)
    projected_gravity = t2n(projected_gravity)
    dof_pos = t2n(dof_pos)
    dof_vel = t2n(dof_vel)
    
    print("\n" + "="*80)
    print(f"STEP {step:5d} - Q1 SENSOR READINGS")
    print("="*80)
    
    print("\n📍 BASE POSE (World Frame)")
    print(f"  Position (x,y,z):  [{base_pos[0]:7.3f}, {base_pos[1]:7.3f}, {base_pos[2]:7.3f}] m")
    print(f"  Orientation (r,p,y): [{euler[0]:7.2f}, {euler[1]:7.2f}, {euler[2]:7.2f}] deg")
    print(f"    Expected: roll≈0°, pitch≈0°, yaw≈any (for upright robot)")
    
    print("\n🌐 VELOCITIES - Global Frame (World)")
    print(f"  Linear  (x,y,z): [{base_vel_global[0]:7.3f}, {base_vel_global[1]:7.3f}, {base_vel_global[2]:7.3f}] m/s")
    print(f"  Angular (x,y,z): [{base_ang_global[0]:7.3f}, {base_ang_global[1]:7.3f}, {base_ang_global[2]:7.3f}] rad/s")
    
    print("\n🤖 VELOCITIES - Local Frame (Body) [USED IN OBSERVATIONS]")
    print(f"  Linear  (x,y,z): [{base_vel_local[0]:7.3f}, {base_vel_local[1]:7.3f}, {base_vel_local[2]:7.3f}] m/s")
    print(f"    x = forward/back, y = left/right, z = up/down")
    print(f"  Angular (x,y,z): [{base_ang_local[0]:7.3f}, {base_ang_local[1]:7.3f}, {base_ang_local[2]:7.3f}] rad/s")
    print(f"    x = roll rate, y = pitch rate, z = yaw rate")
    
    print("\n🎯 PROJECTED GRAVITY (IMU Reading) [USED IN OBSERVATIONS]")
    print(f"  (x,y,z): [{projected_gravity[0]:7.3f}, {projected_gravity[1]:7.3f}, {projected_gravity[2]:7.3f}]")
    print(f"    Expected when upright: [0, 0, -1] (gravity points down in body frame)")
    print(f"    Current magnitude: {np.linalg.norm(projected_gravity):.3f} (should be ≈1.0)")
    if abs(projected_gravity[2] + 1.0) < 0.1 and abs(projected_gravity[0]) < 0.1 and abs(projected_gravity[1]) < 0.1:
        print(f"    ✅ Orientation looks correct!")
    else:
        print(f"    ⚠️  Robot might be tilted or orientation incorrect")
    
    print("\n🦿 JOINT POSITIONS (rad)")
    print(f"  {'Joint':15s}  {'Position':>10s}  {'Velocity':>10s}")
    print(f"  {'-'*15}  {'-'*10}  {'-'*10}")
    for name, pos, vel in zip(joint_names, dof_pos, dof_vel):
        print(f"  {name:15s}  {pos:10.3f}  {vel:10.3f}")
    
    print("="*80)

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
    # Try different orientations to find the correct one
    # Option 3: 90° rotation around Y-axis
    robot = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/q1/kutta.urdf",
            pos=(0, 0, 0.5),
            quat=(math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0),  # +90° around Y-axis
        ),
    )
    
    # Build scene
    scene.build()
    
    # Set default pose
    joint_names = [
        "FL_thigh_joint", "FL_calf_joint",
        "FR_thigh_joint", "FR_calf_joint",
        "RL_thigh_joint", "RL_calf_joint",
        "RR_thigh_joint", "RR_calf_joint",
    ]
    
    default_positions = [0.45, 0.15, 0.45, 0.15, 0.25, 0.15, 0.25, 0.15]
    motors_dof_idx = [robot.get_joint(name).dof_start for name in joint_names]
    
    # Initialize robot in default standing pose using the motor indices
    robot.set_dofs_position(default_positions, motors_dof_idx)
    
    print("\n" + "="*80)
    print("Q1 SENSOR DIAGNOSTICS")
    print("="*80)
    print("\nTesting different base orientations to find correct coordinate system:")
    print("  Current: quat=(cos(π/4), 0, sin(π/4), 0) - +90° around Y-axis")
    print("\nThis script displays real-time sensor readings to verify:")
    print("  1. Robot orientation is correct (projected gravity ≈ [0, 0, -1])")
    print("  2. Velocities are in the correct reference frame")
    print("  3. Joint angles are being read correctly")
    print("\nIf projected gravity is NOT [0, 0, -1], we need to adjust base quaternion")
    print("\nPress Ctrl+C or close window to exit")
    print("\nStarting in 2 seconds...")
    print("="*80)
    
    # Let it settle for a moment
    for _ in range(100):
        robot.control_dofs_position(default_positions, motors_dof_idx)
        scene.step()
    
    # Main loop - print readings every 50 steps (1 second at 50Hz)
    step = 0
    print_interval = 50
    
    try:
        while scene.viewer.is_alive():
            # Hold robot in standing pose
            robot.control_dofs_position(default_positions, motors_dof_idx)
            scene.step()
            
            # Print sensor readings periodically
            if step % print_interval == 0:
                print_sensor_readings(robot, step)
            
            step += 1
            
            # Exit after 500 steps (10 seconds)
            if step >= 500:
                print("\n✅ Diagnostic complete!")
                break
                
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    print("\nSensor diagnostic finished.")

if __name__ == "__main__":
    main()
