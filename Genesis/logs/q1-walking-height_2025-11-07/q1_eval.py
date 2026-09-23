import argparse
import os
import pickle
from importlib import metadata
import csv

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

from q1_env import Q1Env


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="q1-walking")
    parser.add_argument("--ckpt", type=int, default=200)
    args = parser.parse_args()

    gs.init()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"logs/{args.exp_name}/cfgs.pkl", "rb"))
    reward_cfg["reward_scales"] = {}

    env = Q1Env(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=gs.device)

    data = []
    iter = 0

    obs, _ = env.reset()
    with torch.no_grad():
        while True:
            actions = policy(obs)
            # print(actions)
            
            act = torch.clip(actions, -env_cfg["clip_actions"], env_cfg["clip_actions"])
            # convert default joint angles (python list) to a tensor matching `act` dtype/device
            default_angles = torch.tensor(
                [env_cfg["default_joint_angles"][name] for name in env_cfg["joint_names"]],
                dtype=act.dtype,
                device=act.device,
            )
            target_dof_pos = act * env_cfg["action_scale"] + default_angles

            # print("Data", data, len(data))

            if iter == 250:
                with open(f"q1_eval_actions_{args.ckpt}.csv", "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerows(data)

            obs, rews, dones, infos = env.step(actions)
            iter += 1
            


if __name__ == "__main__":
    main()

"""
# evaluation
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500
"""
