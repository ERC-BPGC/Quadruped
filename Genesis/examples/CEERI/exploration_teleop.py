"""
Interactive teleoperation script for testing the exploration environment.
Allows manual control of the drone to verify LiDAR sensor and environment setup.
"""
import argparse
import os
import threading
import numpy as np
import torch
import genesis as gs

from exploration_env import ExplorationEnv

IS_PYNPUT_AVAILABLE = False
try:
    from pynput import keyboard
    IS_PYNPUT_AVAILABLE = True
except ImportError:
    pass

# Movement increments for keyboard control
KEY_VEL_INCREMENT = 0.1  # m/s
KEY_YAW_INCREMENT = 0.1  # rad/s


class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)

    def start(self):
        self.listener.start()

    def stop(self):
        try:
            self.listener.stop()
        except NotImplementedError:
            pass
        self.listener.join()

    def on_press(self, key: "keyboard.Key"):
        with self.lock:
            self.pressed_keys.add(key)

    def on_release(self, key: "keyboard.Key"):
        with self.lock:
            self.pressed_keys.discard(key)

    def get_pressed_keys(self):
        with self.lock:
            return self.pressed_keys.copy()


def get_env_cfgs():
    """Get configuration for exploration environment."""
    env_cfg = {
        "num_actions": 3,  # x_vel, y_vel, yaw_rate
        
        # Episode settings
        "episode_length_s": 120.0,
        
        # Termination conditions
        "termination_if_pitch_greater_than": 45,  # degrees
        "termination_if_roll_greater_than": 45,
        "termination_if_close_to_ground": 0.15,
        "termination_if_too_high": 2.5,
        "collision_threshold": 0.3,  # meters
        
        # Initial pose - spawn in open area away from walls
        "base_init_pos": [-3.0, 3.0, 1.0],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "randomize_init_pos": False,
        
        # Control parameters
        "max_x_vel": 1.5,  # m/s
        "max_y_vel": 1.5,  # m/s
        "max_yaw_rate": 1.57,  # rad/s
        "clip_actions": 1.0,
        
        # Controller gains
        "kp_vel": 3000.0,
        "kp_yaw": 5000.0,
        
        # LiDAR settings
        "lidar_n_rays": 128,
        "lidar_max_range": 6.0,
        
        # Visualization
        "visualize_lidar": True,
        "visualize_camera": False,
        "max_visualize_FPS": 60,
    }
    
    obs_cfg = {
        "num_obs": 15,  # F(1) + R(1) + L(1) + lin_vel(3) + euler(3) + ang_vel(3) + last_actions(3)
        "obs_scales": {
            "lidar": 1.0 / 6.0,  # normalize by max range
            "lin_vel": 1.0 / 3.0,
            "euler": 1.0 / 180.0,  # degrees to normalized
            "ang_vel": 1.0 / 3.14159,
        },
    }
    
    reward_cfg = {
        "reward_scales": {
            "variable_pitch": 0.0,  # Disabled for teleop
            "obstacle_avoidance": 0.0,
            "center_alignment": 0.0,
            "exploration": 0.0,
            "collision": 0.0,
            "smooth": 0.0,
            "angular": 0.0,
        },
    }
    
    return env_cfg, obs_cfg, reward_cfg


def main():
    parser = argparse.ArgumentParser(description="Teleop testing for exploration environment")
    parser.add_argument("--cpu", action="store_true", help="Run on CPU instead of GPU")
    args = parser.parse_args()

    if IS_PYNPUT_AVAILABLE:
        kb = KeyboardDevice()
        kb.start()
        print("\n" + "="*60)
        print("KEYBOARD CONTROLS:")
        print("="*60)
        print("  ↑ (Up Arrow)    : Move Forward (+X)")
        print("  ↓ (Down Arrow)  : Move Backward (-X)")
        print("  ← (Left Arrow)  : Move Left (+Y)")
        print("  → (Right Arrow) : Move Right (-Y)")
        print("  A               : Rotate Left (CCW)")
        print("  D               : Rotate Right (CW)")
        print("  Space           : Stop (zero velocity)")
        print("  R               : Reset drone")
        print("  ESC             : Quit")
        print("="*60)
        print("\nLiDAR Debug Visualization is ENABLED")
        print("Green rays = no hit, Red rays = hit obstacle")
        print("="*60 + "\n")
    else:
        print("WARNING: pynput not installed. Keyboard control disabled.")
        print("Install with: pip install pynput")
        return

    # Initialize Genesis
    gs.init(backend=gs.cpu if args.cpu else gs.gpu, precision="32", logging_level="info")

    # Create environment
    env_cfg, obs_cfg, reward_cfg = get_env_cfgs()
    env = ExplorationEnv(
        num_envs=1,  # Single environment
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        show_viewer=True,
    )

    # Reset environment
    obs, _ = env.reset()

    # Control state
    x_vel_target = 0.0
    y_vel_target = 0.0
    yaw_rate_target = 0.0

    print("Environment ready! Use keyboard to control the drone.")
    print(f"Initial LiDAR readings - F: {env.F[0].item():.2f}m, R: {env.R[0].item():.2f}m, L: {env.L[0].item():.2f}m\n")

    step_count = 0
    try:
        while True:
            if IS_PYNPUT_AVAILABLE:
                pressed = kb.get_pressed_keys()
                
                # Check for quit
                if keyboard.Key.esc in pressed:
                    break
                
                # Reset
                if keyboard.KeyCode.from_char('r') in pressed or keyboard.KeyCode.from_char('R') in pressed:
                    obs, _ = env.reset()
                    x_vel_target = 0.0
                    y_vel_target = 0.0
                    yaw_rate_target = 0.0
                    print("Environment reset!")
                    continue
                
                # Stop (zero velocity)
                if keyboard.Key.space in pressed:
                    x_vel_target = 0.0
                    y_vel_target = 0.0
                    yaw_rate_target = 0.0
                
                # Forward/backward
                if keyboard.Key.up in pressed:
                    x_vel_target = min(1.0, x_vel_target + KEY_VEL_INCREMENT)
                if keyboard.Key.down in pressed:
                    x_vel_target = max(-1.0, x_vel_target - KEY_VEL_INCREMENT)
                
                # Left/right
                if keyboard.Key.left in pressed:
                    y_vel_target = min(1.0, y_vel_target + KEY_VEL_INCREMENT)
                if keyboard.Key.right in pressed:
                    y_vel_target = max(-1.0, y_vel_target - KEY_VEL_INCREMENT)
                
                # Yaw rotation
                if keyboard.KeyCode.from_char('a') in pressed or keyboard.KeyCode.from_char('A') in pressed:
                    yaw_rate_target = min(1.0, yaw_rate_target + KEY_YAW_INCREMENT)
                if keyboard.KeyCode.from_char('d') in pressed or keyboard.KeyCode.from_char('D') in pressed:
                    yaw_rate_target = max(-1.0, yaw_rate_target - KEY_YAW_INCREMENT)
                
                # Damping (gradually reduce velocities when no input)
                if keyboard.Key.up not in pressed and keyboard.Key.down not in pressed:
                    x_vel_target *= 0.95
                if keyboard.Key.left not in pressed and keyboard.Key.right not in pressed:
                    y_vel_target *= 0.95
                if (keyboard.KeyCode.from_char('a') not in pressed and 
                    keyboard.KeyCode.from_char('A') not in pressed and
                    keyboard.KeyCode.from_char('d') not in pressed and
                    keyboard.KeyCode.from_char('D') not in pressed):
                    yaw_rate_target *= 0.95

            # Create action vector [x_vel, y_vel, yaw_rate] (normalized to [-1, 1])
            action = torch.tensor([[x_vel_target, y_vel_target, yaw_rate_target]], 
                                 dtype=torch.float32, device=gs.device)
            
            # Step environment
            obs, reward, done, info = env.step(action)

            # Print debug info every 50 steps
            step_count += 1
            if step_count % 50 == 0:
                pos = env.base_pos[0].cpu().numpy()
                euler = env.base_euler[0].cpu().numpy()
                print(f"Step {step_count:5d} | Pos: [{pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f}] | "
                      f"Yaw: {euler[2]:6.1f}° | "
                      f"LiDAR - F: {env.F[0].item():4.2f}m R: {env.R[0].item():4.2f}m L: {env.L[0].item():4.2f}m | "
                      f"Cmd: [{x_vel_target:5.2f}, {y_vel_target:5.2f}, {yaw_rate_target:5.2f}]")
            
            # Check for reset
            if done[0].item():
                print(f"\nEpisode ended at step {step_count}")
                if env.collision_condition[0].item():
                    print("Reason: COLLISION!")
                elif env.crash_condition[0].item():
                    print("Reason: CRASH (attitude/height violation)")
                else:
                    print("Reason: Timeout")
                print(f"Final LiDAR - F: {env.F[0].item():.2f}m, R: {env.R[0].item():.2f}m, L: {env.L[0].item():.2f}m")
                
                obs, _ = env.reset()
                x_vel_target = 0.0
                y_vel_target = 0.0
                yaw_rate_target = 0.0
                step_count = 0
                print("\nAuto-reset complete. Continue flying!\n")

            # For pytest
            if "PYTEST_VERSION" in os.environ:
                break

    except KeyboardInterrupt:
        print("\n\nTeleop interrupted by user.")
    finally:
        if IS_PYNPUT_AVAILABLE:
            kb.stop()
        print("\nTeleop session ended.")
        print(f"Total steps: {step_count}")


if __name__ == "__main__":
    main()

"""
Usage:
    python exploration_teleop.py
    python exploration_teleop.py --cpu
"""
