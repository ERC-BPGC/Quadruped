#!/usr/bin/env python3
"""
Test script to check motor configuration generation
"""

import sys
import yaml
from quadruped_enhanced import QuadrupedEnhanced, LegConfig

def test_motor_config():
    print("Testing motor configuration generation...")
    
    # Load config
    with open("config.yaml", 'r') as f:
        config = yaml.safe_load(f)
    
    # Create a leg config
    leg_config = LegConfig.from_config(config)
    
    # Test hip motor config generation
    print("\n=== HIP MOTOR CONFIG ===")
    from quadruped_enhanced import Leg
    leg = Leg(None, leg_config, config.get('motor_config', {}), simulation_mode=True)
    hip_config = leg._build_motor_config('hip')
    
    print("\n=== KNEE MOTOR CONFIG ===")
    knee_config = leg._build_motor_config('knee')
    
    print("\n=== CONFIG VALUES FROM YAML ===")
    print("Hip current_lim from config:", config['motor_config']['hip']['current_lim'])
    print("Knee current_lim from config:", config['motor_config']['knee']['current_lim'])
    print("Hip calibration_lockin_current from config:", config['motor_config']['hip']['calibration_lockin_current'])
    print("Knee calibration_lockin_current from config:", config['motor_config']['knee']['calibration_lockin_current'])

if __name__ == "__main__":
    test_motor_config()