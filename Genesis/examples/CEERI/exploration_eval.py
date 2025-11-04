import argparse
import os
import pickle
from importlib import metadata

import torch

try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e
from rsl_rl.runners import OnPolicyRunner

import genesis as gs

from exploration_env import ExplorationEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="drone-exploration")
    parser.add_argument("--ckpt", type=int, default=5000)
    parser.add_argument("--record", action="store_true", default=False)
    parser.add_argument("--num_episodes", type=int, default=10, help="Number of episodes to evaluate")
    args = parser.parse_args()

    gs.init()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, train_cfg = pickle.load(open(f"logs/{args.exp_name}/cfgs.pkl", "rb"))
    
    # Disable all reward scales for clean evaluation
    reward_cfg["reward_scales"] = {k: 0.0 for k in reward_cfg["reward_scales"].keys()}

    # Enable visualization
    env_cfg["visualize_lidar"] = True
    env_cfg["visualize_camera"] = args.record
    env_cfg["max_visualize_FPS"] = 30
    env_cfg["randomize_init_pos"] = False  # Start from same position for consistency

    env = ExplorationEnv(
        num_envs=1,  # Single environment for evaluation
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        show_viewer=True,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    
    if not os.path.exists(resume_path):
        print(f"Error: Checkpoint not found at {resume_path}")
        print(f"Available checkpoints in {log_dir}:")
        for f in sorted(os.listdir(log_dir)):
            if f.startswith("model_") and f.endswith(".pt"):
                print(f"  - {f}")
        return
    
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=gs.device)

    print(f"\n{'='*70}")
    print(f"Evaluating policy from: {resume_path}")
    print(f"{'='*70}\n")

    # Episode statistics
    episode_rewards = []
    episode_lengths = []
    collision_count = 0
    success_count = 0

    with torch.no_grad():
        if args.record:
            env.cam.start_recording()

        for episode in range(args.num_episodes):
            obs, _ = env.reset()
            episode_reward = 0.0
            episode_length = 0
            
            print(f"\n--- Episode {episode + 1}/{args.num_episodes} ---")
            
            max_steps = int(env_cfg["episode_length_s"] * env_cfg["max_visualize_FPS"])
            
            for step in range(max_steps):
                actions = policy(obs)
                obs, rews, dones, infos = env.step(actions)
                
                episode_reward += rews[0].item()
                episode_length += 1
                
                # Print debug info every 100 steps
                if step % 100 == 0:
                    pos = env.base_pos[0].cpu().numpy()
                    euler = env.base_euler[0].cpu().numpy()
                    print(f"  Step {step:4d} | Pos: [{pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f}] | "
                          f"Yaw: {euler[2]:6.1f}° | "
                          f"LiDAR - F: {env.F[0].item():4.2f}m R: {env.R[0].item():4.2f}m L: {env.L[0].item():4.2f}m")
                
                if args.record:
                    env.cam.render()
                
                if dones[0].item():
                    break
            
            # Episode summary
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            
            if env.collision_condition[0].item():
                collision_count += 1
                print(f"  Result: COLLISION at step {episode_length}")
            elif episode_length >= max_steps:
                success_count += 1
                print(f"  Result: SUCCESS (completed full episode)")
            else:
                print(f"  Result: CRASH (attitude/height violation) at step {episode_length}")
            
            print(f"  Episode reward: {episode_reward:.2f}")
            print(f"  Episode length: {episode_length} steps ({episode_length * env.dt:.1f}s)")

        if args.record:
            video_name = f"{args.exp_name}_ckpt{args.ckpt}.mp4"
            env.cam.stop_recording(save_to_filename=video_name, fps=env_cfg["max_visualize_FPS"])
            print(f"\nVideo saved to: {video_name}")

    # Print overall statistics
    print(f"\n{'='*70}")
    print(f"EVALUATION SUMMARY ({args.num_episodes} episodes)")
    print(f"{'='*70}")
    print(f"Average reward:        {sum(episode_rewards) / len(episode_rewards):.2f}")
    print(f"Average length:        {sum(episode_lengths) / len(episode_lengths):.1f} steps "
          f"({sum(episode_lengths) / len(episode_lengths) * env.dt:.1f}s)")
    print(f"Success rate:          {success_count}/{args.num_episodes} "
          f"({100 * success_count / args.num_episodes:.1f}%)")
    print(f"Collision rate:        {collision_count}/{args.num_episodes} "
          f"({100 * collision_count / args.num_episodes:.1f}%)")
    print(f"{'='*70}\n")


if __name__ == "__main__":
    main()

"""
# Evaluation commands:

# Basic evaluation
python exploration_eval.py -e drone-exploration --ckpt 5000

# Evaluation with video recording
python exploration_eval.py -e drone-exploration --ckpt 5000 --record

# Evaluate more episodes
python exploration_eval.py -e drone-exploration --ckpt 5000 --num_episodes 20

# Evaluate different checkpoint
python exploration_eval.py -e drone-exploration --ckpt 2500

Note: If you experience slow performance or encounter other issues
during evaluation, try removing the --record option.
"""
