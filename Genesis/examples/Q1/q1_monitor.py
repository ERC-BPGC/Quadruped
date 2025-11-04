"""
Simple training monitor script that displays training progress in real-time.
Run this in a separate terminal while training to monitor progress.
"""

import os
import argparse
import time
from pathlib import Path

def monitor_training(exp_name="q1-walking", refresh_interval=5):
    """
    Monitor training progress by reading log files.
    
    Args:
        exp_name: Name of the experiment to monitor
        refresh_interval: How often to refresh (seconds)
    """
    log_dir = Path(f"logs/{exp_name}")
    
    if not log_dir.exists():
        print(f"Error: Log directory not found: {log_dir}")
        print("Make sure training has started and the experiment name is correct.")
        return
    
    print(f"Monitoring training progress for: {exp_name}")
    print(f"Log directory: {log_dir}")
    print(f"Refresh interval: {refresh_interval} seconds")
    print("-" * 80)
    
    last_mtime = 0
    iteration_count = 0
    
    try:
        while True:
            # Check for model checkpoints
            model_files = sorted(log_dir.glob("model_*.pt"))
            
            if model_files:
                latest_model = model_files[-1]
                iteration_num = latest_model.stem.split("_")[1]
                
                # Check if there's new data
                current_mtime = latest_model.stat().st_mtime
                if current_mtime > last_mtime:
                    last_mtime = current_mtime
                    iteration_count = int(iteration_num)
                    
                    print(f"\n[{time.strftime('%H:%M:%S')}] Training Progress:")
                    print(f"  Latest checkpoint: model_{iteration_num}.pt")
                    print(f"  Total iterations: {iteration_count}")
                    
                    # Try to read tensorboard logs if available
                    tb_files = list(log_dir.glob("events.out.tfevents.*"))
                    if tb_files:
                        print(f"  Tensorboard logs: {len(tb_files)} file(s)")
                        print(f"  View with: tensorboard --logdir {log_dir}")
            else:
                print(f"[{time.strftime('%H:%M:%S')}] Waiting for training to start...")
            
            time.sleep(refresh_interval)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")
        print(f"Final iteration count: {iteration_count}")

def main():
    parser = argparse.ArgumentParser(description="Monitor Q1 training progress")
    parser.add_argument("-e", "--exp_name", type=str, default="q1-walking",
                        help="Experiment name to monitor")
    parser.add_argument("--interval", type=int, default=5,
                        help="Refresh interval in seconds")
    args = parser.parse_args()
    
    monitor_training(args.exp_name, args.interval)

if __name__ == "__main__":
    main()
