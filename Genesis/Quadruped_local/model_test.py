import mujoco
from mujoco import viewer
import time
import numpy as np
import torch
import pickle
import os
import sys

import csv

with open("/home/ritwik/NUS/Genesis/q1_eval_actions_200.csv", "r") as f:
    reader = csv.reader(f)
    action_data = np.array([[float(val) for val in row] for row in reader])

# Add the parent directory to Python path to import from Genesis examples
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'examples', 'Q1'))

from rsl_rl.runners import OnPolicyRunner

# Configuration
xml_path = "scene.xml"
exp_name = "q1-walking"  # Your experiment name
ckpt = 200  # Checkpoint number to load
log_dir = f"../logs/{exp_name}"

# Load model and data
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Reset to 'home' keyframe
mujoco.mj_resetDataKeyframe(model, data, 0)

# Load Genesis training configuration
print(f"Loading trained policy from {log_dir}...")
env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"{log_dir}/cfgs.pkl", "rb"))

# Load the trained policy
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
resume_path = os.path.join(log_dir, f"model_{ckpt}.pt")
checkpoint = torch.load(resume_path, map_location=device)

# Extract the actor network
actor_critic = checkpoint['model_state_dict']
# Build a simple wrapper to run inference
from torch import nn

class PolicyWrapper(nn.Module):
    def __init__(self, checkpoint, num_obs, num_actions):
        super().__init__()
        self.num_obs = num_obs
        self.num_actions = num_actions
        
        # Load actor network architecture from rsl_rl
        # These dimensions match the actual checkpoint: [512, 256, 128]
        actor_hidden_dims = [512, 256, 128]
        actor_layers = []
        actor_layers.append(nn.Linear(num_obs, actor_hidden_dims[0]))
        actor_layers.append(nn.ELU())
        for l in range(len(actor_hidden_dims)):
            if l == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                actor_layers.append(nn.ELU())
        self.actor = nn.Sequential(*actor_layers)
        
        # Load weights
        actor_state = {k.replace('actor.', ''): v for k, v in checkpoint.items() if k.startswith('actor.')}
        self.actor.load_state_dict(actor_state)
        self.actor.eval()
    
    def forward(self, obs):
        return self.actor(obs)

policy = PolicyWrapper(actor_critic, obs_cfg["num_obs"], env_cfg["num_actions"]).to(device)
print("✅ Policy loaded successfully!")

# Joint names (must match your URDF/XML)
joint_names = env_cfg["joint_names"]
default_joint_angles = np.array([env_cfg["default_joint_angles"][name] for name in joint_names])

# Get joint indices in MuJoCo (joint ids) and actuator ids (ctrl indices)
joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]

# In the MJCF used here the actuator names are the joint name without the trailing
# "_joint" suffix (e.g. joint="FL_thigh_joint" -> actuator name="FL_thigh").
# Build actuator names accordingly and look them up. If lookup fails, fall back
# to searching for actuators by the 'joint' attribute (robust but a bit slower).
actuator_names = [name.replace("_joint", "") for name in joint_names]
actuator_ids = []
for jname, aname in zip(joint_names, actuator_names):
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
    if aid == -1:
        # fallback: try lookup using the joint name directly (some XMLs name actuators differently)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jname)
    actuator_ids.append(aid)

print(f"Joint IDs: {joint_ids}")
print(f"Actuator names attempted: {actuator_names}")
print(f"Actuator IDs (ctrl indices): {actuator_ids}")

# Observation scales
obs_scales = obs_cfg["obs_scales"]
commands_scale = np.array([obs_scales["lin_vel"], obs_scales["lin_vel"], obs_scales["ang_vel"]])

# Command (desired velocity) - adjust these as needed
# [forward_vel, lateral_vel, yaw_rate]
commands = np.array([0, 0.65, 0.0])  # Walk forward at 0.65 m/s

# Initialize last actions
last_actions = np.zeros(env_cfg["num_actions"])

def get_observations(data, model, joint_ids, commands, last_actions, default_joint_angles, obs_scales, commands_scale):
    """Extract observations from MuJoCo data matching Genesis format"""
    
    # Get base orientation (quaternion)
    base_quat = data.qpos[3:7].copy()  # Assuming floating base: [pos(3), quat(4), joints...]
    
    # Apply sensor correction if needed (matching Genesis q1_env.py)
    # Sensor correction quat from config: (0.707, 0.707, 0, 0) - 90° around X
    sensor_correction_quat = np.array([0.707, 0.707, 0, 0])
    
    # Transform quaternion by sensor correction
    corrected_quat = quaternion_multiply(base_quat, sensor_correction_quat)
    
    # Get rotation matrix from quaternion
    rot_mat = quat_to_rotation_matrix(corrected_quat)
    
    # Get angular velocity in base frame
    base_ang_vel_world = data.qvel[3:6].copy()  # Angular velocity in world frame
    base_ang_vel = rot_mat.T @ base_ang_vel_world  # Transform to base frame
    
    # Get projected gravity in base frame
    gravity_world = np.array([0, 0, -1])
    projected_gravity = rot_mat.T @ gravity_world
    
    # Get joint positions and velocities
    dof_pos = data.qpos[7:7+len(joint_ids)]  # Joint positions after base (pos+quat)
    dof_vel = data.qvel[6:6+len(joint_ids)]  # Joint velocities after base (lin+ang vel)
    
    # Build observation vector (matching Genesis q1_env.py)
    # Note: q1_env uses the current actions (self.actions) in the obs (not last_actions).
    obs = np.concatenate([
        base_ang_vel * obs_scales["ang_vel"],  # 3
        projected_gravity,  # 3
        commands * commands_scale,  # 3
        (dof_pos - default_joint_angles) * obs_scales["dof_pos"],  # 8
        last_actions,  # placeholder here; caller will pass the appropriate actions buffer
    ])
    
    return obs

def quaternion_multiply(q1, q2):
    """Multiply two quaternions (w, x, y, z format)"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_to_rotation_matrix(q):
    """Convert quaternion to rotation matrix (w, x, y, z format)"""
    w, x, y, z = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])

print("\n" + "="*60)
print("Running trained policy in MuJoCo")
print("="*60)
print(f"Command: forward={commands[0]:.2f} m/s, lateral={commands[1]:.2f} m/s, yaw={commands[2]:.2f} rad/s")
print("Press ESC to quit.")
print("="*60 + "\n")

with viewer.launch_passive(model, data) as gui:
    step = 0
    
    while gui.is_running():
        # Get observations from MuJoCo
        obs_np = get_observations(
            data, model, joint_ids, commands, last_actions, 
            default_joint_angles, obs_scales, commands_scale
        )
        
        # Convert to torch and run policy
        obs_torch = torch.from_numpy(obs_np).float().unsqueeze(0).to(device)

        # print(obs_torch)
        
        # Clip actions
        if step % 1 == 0:
            with torch.no_grad():
                actions = policy(obs_torch).cpu().numpy()[0]
            actions = np.clip(actions, -env_cfg["clip_actions"], env_cfg["clip_actions"])

        # actions = action_data[step//8]

        if step < 100:
            actions = np.zeros_like(actions)
            print("Standddding")
        
        # Compute target joint positions
        target_dof_pos = actions * env_cfg["action_scale"] + default_joint_angles

        # target_dof_pos = np.zeros_like(target_dof_pos)

        # print(target_dof_pos) 
        
        # Apply to MuJoCo (use position control) --- write into actuator ctrl indices
        for i, act_id in enumerate(actuator_ids):
            if act_id < 0 or act_id >= model.nu:
                # actuator not found or out of range
                print(f"Warning: actuator id for '{joint_names[i]}' is {act_id} (out of range). Skipping")
                continue
            # Set position target via ctrl (actuator index)
            data.ctrl[act_id] = target_dof_pos[i]
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Update last actions
        last_actions = actions.copy()
        
        # Sync viewer
        gui.sync()
        
        # Control loop timing (50 Hz like Genesis)
        time.sleep(0.002)
        
        step += 1
        if step % 50 == 0:
            print(f"Step {step}: Base pos: [{data.qpos[0]:.2f}, {data.qpos[1]:.2f}, {data.qpos[2]:.2f}]")

print("\n✅ Simulation complete!")
