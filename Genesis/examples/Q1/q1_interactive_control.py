import genesis as gs
import numpy as np
import threading
from pynput import keyboard

# Keyboard handler
class KeyboardController:
    def __init__(self):
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
    
    def on_press(self, key):
        with self.lock:
            self.pressed_keys.add(key)
    
    def on_release(self, key):
        with self.lock:
            self.pressed_keys.discard(key)
    
    def is_pressed(self, key):
        with self.lock:
            return key in self.pressed_keys
    
    def get_pressed_keys(self):
        with self.lock:
            return self.pressed_keys.copy()

# Initialize Genesis
gs.init(backend=gs.cuda)

# Create scene
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(2.0, 0.0, 1.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
        max_FPS=60,
    ),
    show_viewer=True,
    rigid_options=gs.options.RigidOptions(
        dt=0.01,
        constraint_solver=gs.constraint_solver.Newton,
    ),
)

# Add plane
plane = scene.add_entity(gs.morphs.Plane())

# Load Q1 robot from URDF
robot = scene.add_entity(
    gs.morphs.URDF(
        file="urdf/q1/kutta.urdf",
        pos=(0, 0, 0.5),
        quat=(0.7071, -0.7071, 0.0, 0.0),  # 90 degree rotation around X-axis
    ),
)

# Build scene
scene.build()

# Joint names in order
joint_names = [
    "FL_thigh_joint", "FL_calf_joint", 
    "FR_thigh_joint", "FR_calf_joint",
    "RL_thigh_joint", "RL_calf_joint", 
    "RR_thigh_joint", "RR_calf_joint"
]

# Get DOF indices for motors using get_joint()
motors_dof_idx = [robot.get_joint(name).dof_start for name in joint_names]

# NOTE: Not setting PD gains explicitly - using URDF/Genesis defaults
# The visualize script works without setting gains, so we match that behavior

print(f"Found {len(joint_names)} controllable joints")
print("Joint names:")
for i, name in enumerate(joint_names):
    print(f"  {i}: {name} (dof_idx: {motors_dof_idx[i]})")

# Initialize joint positions (default standing pose)
joint_positions = np.array([
    0.45,   # FL_thigh
    0.15,  # FL_calf
    0.45,   # FR_thigh
    0.15,  # FR_calf
    0.25,   # RL_thigh
    0.15,  # RL_calf
    0.25,   # RR_thigh
    0.15,  # RR_calf
], dtype=float)

# IMPORTANT: Initialize the robot to the default pose before starting control loop
# This prevents the robot from collapsing at the start
print("\nInitializing robot to default pose...")
robot.set_dofs_position(joint_positions, motors_dof_idx)

# Step the simulation multiple times with active control to let it stabilize
print("Stabilizing robot (holding pose with PD control)...")
for step in range(100):
    robot.control_dofs_position(joint_positions, motors_dof_idx)
    scene.step()
    if step % 20 == 0:
        actual = robot.get_dofs_position(motors_dof_idx).cpu().numpy()
        error = np.abs(joint_positions - actual).max()
        print(f"  Step {step}: max error = {error:.4f} rad")

print("Robot initialized and stabilized!")

# Joint limits (from URDF/XML - check these match your robot)
joint_limits = {
    'thigh': (-0.9, 0.67),
    'calf': (-0.35, 0.5),  # Note: calf joints typically have negative ranges
}

# UI state
selected_joint = 0
increment = 0.05

# Joint display names (shorter)
joint_display_names = [
    "FL_thigh", "FL_calf", "FR_thigh", "FR_calf",
    "RL_thigh", "RL_calf", "RR_thigh", "RR_calf"
]

print("\n" + "="*60)
print("INTERACTIVE Q1 ROBOT CONTROLLER")
print("="*60)
print("\nControls:")
print("  [1-8]     : Select joint (1=FL_thigh, 2=FL_calf, etc.)")
print("  [+] / [-] : Increase/Decrease selected joint angle")
print("  [W] / [S] : Increase/Decrease by 0.01 rad")
print("  [A] / [D] : Increase/Decrease by 0.1 rad")
print("  [R]       : Reset to default standing pose")
print("  [P]       : Print current joint angles")
print("  [H]       : Show this help")
print("  [ESC]     : Exit")
print("="*60)

def print_joint_status():
    # Get actual joint positions from simulation
    actual_positions = robot.get_dofs_position(motors_dof_idx).cpu().numpy()
    
    print("\n" + "-"*80)
    print("Current Joint Angles:")
    print(f"{'':5} {'Joint':12} {'Commanded':>12} {'Actual':>12} {'Error':>12}")
    print("-"*80)
    for i, (name, cmd_angle, act_angle) in enumerate(zip(joint_display_names, joint_positions, actual_positions)):
        marker = ">>> " if i == selected_joint else "    "
        error = abs(cmd_angle - act_angle)
        print(f"{marker}{i+1}. {name:12s}: {cmd_angle:7.3f} rad  {act_angle:7.3f} rad  {error:7.4f} rad")
    print("-"*80)

def reset_to_default():
    global joint_positions
    joint_positions = np.array([
        0.45,   # FL_thigh
        0.15,  # FL_calf
        0.45,   # FR_thigh
        0.15,  # FR_calf
        0.25,   # RL_thigh
        0.15,  # RL_calf
        0.25,   # RR_thigh
        0.15,  # RR_calf
    ], dtype=float)
    print("\n✓ Reset to default standing pose")
    print_joint_status()

def apply_limits(joint_idx, value):
    """Apply joint limits"""
    if 'thigh' in joint_display_names[joint_idx]:
        return np.clip(value, joint_limits['thigh'][0], joint_limits['thigh'][1])
    else:  # calf
        return np.clip(value, joint_limits['calf'][0], joint_limits['calf'][1])

# Show initial status
print_joint_status()

# Initialize keyboard controller
kb = KeyboardController()

# Track last key press to avoid repeated triggers
last_action_step = -10
key_cooldown = 3  # Steps between key presses

# Main control loop
for i in range(100000):
    # Control joint positions using PD control
    robot.control_dofs_position(joint_positions, motors_dof_idx)
    
    # Get keyboard input with cooldown
    if i - last_action_step >= key_cooldown:
        pressed = kb.get_pressed_keys()
        
        action_taken = False
        
        # Number keys to select joint
        for j in range(1, 9):
            if keyboard.KeyCode.from_char(str(j)) in pressed:
                selected_joint = j - 1
                print(f"\n>>> Selected: {joint_display_names[selected_joint]}")
                action_taken = True
                break
        
        # Plus/Minus keys for 0.05 rad increments
        if keyboard.KeyCode.from_char('=') in pressed or keyboard.KeyCode.from_char('+') in pressed:
            old_val = joint_positions[selected_joint]
            joint_positions[selected_joint] = apply_limits(
                selected_joint, 
                joint_positions[selected_joint] + 0.05
            )
            actual = robot.get_dofs_position(motors_dof_idx).cpu().numpy()[selected_joint]
            print(f"{joint_display_names[selected_joint]}: {old_val:.3f} -> {joint_positions[selected_joint]:.3f} rad (actual: {actual:.3f})")
            action_taken = True
        
        if keyboard.KeyCode.from_char('-') in pressed or keyboard.KeyCode.from_char('_') in pressed:
            old_val = joint_positions[selected_joint]
            joint_positions[selected_joint] = apply_limits(
                selected_joint,
                joint_positions[selected_joint] - 0.05
            )
            actual = robot.get_dofs_position(motors_dof_idx).cpu().numpy()[selected_joint]
            print(f"{joint_display_names[selected_joint]}: {old_val:.3f} -> {joint_positions[selected_joint]:.3f} rad (actual: {actual:.3f})")
            action_taken = True
        
        # W/S for small increments (0.01 rad)
        if keyboard.KeyCode.from_char('w') in pressed or keyboard.KeyCode.from_char('W') in pressed:
            joint_positions[selected_joint] = apply_limits(
                selected_joint,
                joint_positions[selected_joint] + 0.01
            )
            print(f"{joint_display_names[selected_joint]}: {joint_positions[selected_joint]:.3f} rad")
            action_taken = True
        
        if keyboard.KeyCode.from_char('s') in pressed or keyboard.KeyCode.from_char('S') in pressed:
            joint_positions[selected_joint] = apply_limits(
                selected_joint,
                joint_positions[selected_joint] - 0.01
            )
            print(f"{joint_display_names[selected_joint]}: {joint_positions[selected_joint]:.3f} rad")
            action_taken = True
        
        # A/D for large increments (0.1 rad)
        if keyboard.KeyCode.from_char('a') in pressed or keyboard.KeyCode.from_char('A') in pressed:
            joint_positions[selected_joint] = apply_limits(
                selected_joint,
                joint_positions[selected_joint] - 0.1
            )
            print(f"{joint_display_names[selected_joint]}: {joint_positions[selected_joint]:.3f} rad")
            action_taken = True
        
        if keyboard.KeyCode.from_char('d') in pressed or keyboard.KeyCode.from_char('D') in pressed:
            joint_positions[selected_joint] = apply_limits(
                selected_joint,
                joint_positions[selected_joint] + 0.1
            )
            print(f"{joint_display_names[selected_joint]}: {joint_positions[selected_joint]:.3f} rad")
            action_taken = True
        
        # Reset
        if keyboard.KeyCode.from_char('r') in pressed or keyboard.KeyCode.from_char('R') in pressed:
            reset_to_default()
            action_taken = True
        
        # Print status
        if keyboard.KeyCode.from_char('p') in pressed or keyboard.KeyCode.from_char('P') in pressed:
            print_joint_status()
            action_taken = True
        
        # Help
        if keyboard.KeyCode.from_char('h') in pressed or keyboard.KeyCode.from_char('H') in pressed:
            print("\n" + "="*60)
            print("Controls:")
            print("  [1-8]     : Select joint")
            print("  [+] / [-] : Increase/Decrease by 0.05 rad")
            print("  [W] / [S] : Increase/Decrease by 0.01 rad")
            print("  [A] / [D] : Increase/Decrease by 0.1 rad")
            print("  [R]       : Reset to default")
            print("  [P]       : Print current angles")
            print("  [H]       : Show this help")
            print("  [ESC]     : Exit")
            print("="*60)
            action_taken = True
        
        # Exit
        if keyboard.Key.esc in pressed:
            print("\nExiting...")
            break
        
        if action_taken:
            last_action_step = i
    
    scene.step()
    
    if not scene.viewer.is_alive():
        break

print("Done!")
