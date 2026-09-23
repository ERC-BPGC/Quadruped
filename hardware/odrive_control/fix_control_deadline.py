#!/usr/bin/env python3
"""
Fix Control Deadline Missed Error
Quick script to apply settings that should resolve CONTROL_DEADLINE_MISSED errors
"""

import odrive
import time
import sys

def fix_control_deadline_for_odrive(odrv):
    """Apply settings to fix control deadline issues"""
    print(f"Fixing control deadline for ODrive {odrv.serial_number}")
    
    # First, check current motor state and errors
    print("\nChecking current motor states:")
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        print(f"  Axis {axis_num}: state={axis.current_state}, error={axis.error}")
        print(f"    Motor error: {axis.motor.error}")
        print(f"    Controller error: {axis.controller.error}")
        print(f"    Current effective_current_lim: {axis.motor.effective_current_lim}")
    
    # Focus on knee motor (axis1 typically)
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        motor_type = "hip" if axis_num == 0 else "knee"
        
        print(f"\nConfiguring {motor_type} motor (axis{axis_num}):")
        
        # Increase current control bandwidth for faster response
        current_bandwidth = 2000 if motor_type == "knee" else 1500
        axis.motor.config.current_control_bandwidth = current_bandwidth
        print(f"  Set current_control_bandwidth = {current_bandwidth}")
        
        # Also increase velocity control bandwidth
        if hasattr(axis.controller.config, 'vel_control_bandwidth'):
            axis.controller.config.vel_control_bandwidth = 1000
            print(f"  Set vel_control_bandwidth = 1000")
        
        # Improve velocity tracking
        if motor_type == "knee":
            # More aggressive gains for knee motors
            axis.controller.config.pos_gain = 200  # Increased from 150
            axis.controller.config.vel_gain = 0.05  # Increased from 0.02
            axis.controller.config.vel_integrator_gain = 0.2  # Increased from 0.1
            axis.controller.config.vel_limit_tolerance = 1.5  # More tolerance
            
            # Remove torque ramping for immediate response
            axis.controller.config.torque_ramp_rate = 0.1  # Faster ramp
            
            # Disable overspeed protection temporarily
            axis.controller.config.enable_overspeed_error = False
            
            # Less aggressive trajectory but ensure it completes
            axis.trap_traj.config.vel_limit = 15  # Further reduced
            axis.trap_traj.config.accel_limit = 20  # Further reduced
            axis.trap_traj.config.decel_limit = 20  # Further reduced
            
            print(f"  Updated knee control parameters for aggressive tracking")
            print(f"    pos_gain = 200, vel_gain = 0.05")
            print(f"    Reduced trajectory limits for reliability")
        
        # Ensure current limits are proper and aggressive
        if motor_type == "hip":
            axis.motor.config.current_lim = 25.0  # Increased from 20
            axis.motor.config.requested_current_range = 30.0
        else:  # knee
            axis.motor.config.current_lim = 80.0  # Increased from 60
            axis.motor.config.requested_current_range = 90.0
            
        # Optional: Switch to current control if position control keeps failing
        if motor_type == "knee":
            print(f"  Option: Switch to current control mode")
            print(f"    axis.controller.config.control_mode = 1  # TORQUE_CONTROL")
            print(f"    axis.controller.config.input_mode = 1   # PASSTHROUGH")
            
        print()

def switch_to_current_control(odrv, axis_num):
    """Emergency switch to current control for problematic motors"""
    axis = getattr(odrv, f"axis{axis_num}")
    print(f"Switching axis {axis_num} to current control mode...")
    
    # Switch to current control
    axis.controller.config.control_mode = 1  # TORQUE_CONTROL
    axis.controller.config.input_mode = 1    # PASSTHROUGH
    
    # Set a reasonable current limit
    axis.controller.input_torque = 0.0  # Start with no torque
    
    print(f"Axis {axis_num} now in current control mode")
    print("Use axis.controller.input_torque to control (range: -current_lim to +current_lim)")

def diagnose_control_issues(odrv):
    """Diagnose what's causing control deadline issues"""
    print(f"\nDiagnosing ODrive {odrv.serial_number}:")
    
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        motor_type = "hip" if axis_num == 0 else "knee"
        
        print(f"\n=== {motor_type.upper()} MOTOR (Axis {axis_num}) ===")
        print(f"Current State: {axis.current_state}")
        print(f"Motor Error: {axis.motor.error}")
        print(f"Controller Error: {axis.controller.error}")
        print(f"Encoder Error: {axis.encoder.error}")
        
        # Control loop timing
        print(f"Current Control Bandwidth: {axis.motor.config.current_control_bandwidth}")
        print(f"Effective Current Limit: {axis.motor.effective_current_lim}")
        print(f"Actual Current Limit: {axis.motor.config.current_lim}")
        
        # Controller settings
        print(f"Position Gain: {axis.controller.config.pos_gain}")
        print(f"Velocity Gain: {axis.controller.config.vel_gain}")
        print(f"Velocity Limit: {axis.controller.config.vel_limit}")
        
        # Trajectory settings
        print(f"Trap Traj Vel Limit: {axis.trap_traj.config.vel_limit}")
        print(f"Trap Traj Accel Limit: {axis.trap_traj.config.accel_limit}")
        
        # Current measurements
        print(f"Current Measured Ph B: {axis.motor.current_meas_phB}")
        print(f"Current Measured Ph C: {axis.motor.current_meas_phC}")
        
        # Position/velocity
        print(f"Position Estimate: {axis.encoder.pos_estimate:.3f}")
        print(f"Velocity Estimate: {axis.encoder.vel_estimate:.3f}")
        print(f"Position Setpoint: {axis.controller.pos_setpoint:.3f}")
        print(f"Velocity Setpoint: {axis.controller.vel_setpoint:.3f}")
        
        # Remove or increase current ramp rate limits
        axis.motor.config.current_lim_margin = axis.motor.config.current_lim - 2.0
        print(f"  Set current_lim_margin = {axis.motor.config.current_lim_margin}")

def main():
    print("ODrive Control Deadline Fix Script")
    print("==================================")
    
    # Connect to all ODrives
    print("Searching for ODrives...")
    odrives = []
    
    try:
        # Try to find ODrives
        for i in range(4):  # Try to find up to 4 ODrives
            try:
                odrv = odrive.find_any(timeout=2)
                if odrv and odrv not in odrives:
                    odrives.append(odrv)
                    print(f"Found ODrive: {odrv.serial_number}")
            except:
                break
                
        if not odrives:
            print("No ODrives found!")
            return
            
        # Apply fixes to all ODrives
        for odrv in odrives:
            fix_control_deadline_for_odrive(odrv)
            
        print(f"\nFixed {len(odrives)} ODrives")
        
        # Run diagnostics
        print("\n" + "="*50)
        print("RUNNING DIAGNOSTICS")
        print("="*50)
        for odrv in odrives:
            diagnose_control_issues(odrv)
        
        print("\nSettings applied - test your motors now!")
        
        # Ask about switching to current control if needed
        problem_axis = input("\nIf you still have issues, which axis has problems? (e.g., '0.1' for ODrive 0 Axis 1, or 'none'): ").strip()
        if problem_axis != 'none' and '.' in problem_axis:
            try:
                odrv_idx, axis_num = map(int, problem_axis.split('.'))
                if 0 <= odrv_idx < len(odrives):
                    switch_to_current_control(odrives[odrv_idx], axis_num)
            except:
                print("Invalid format. Use format like '0.1' for ODrive 0 Axis 1")
        
        # Optional: Save configuration
        save = input("\nSave configuration to ODrive? (y/n): ").lower().strip()
        if save == 'y':
            for odrv in odrives:
                try:
                    odrv.save_configuration()
                    print(f"Configuration saved for ODrive {odrv.serial_number}")
                except:
                    print(f"Could not save configuration for ODrive {odrv.serial_number}")
                    
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()