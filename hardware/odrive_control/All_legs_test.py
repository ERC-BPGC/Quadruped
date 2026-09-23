from __future__ import print_function
import time
import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter
import Quadruped_leg_test_algo as qlt

# Constants
NUM_ODRIVES = 4
L_1 = 24
L_2 = 37.5

# Helper to configure a single motor and encoder
def configure_axis(axis, motor_config, encoder_config, ctrl_config, traj_config):
    # Motor config
    axis.motor.config.pole_pairs = motor_config['pole_pairs']
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.resistance_calib_max_voltage = 6
    axis.motor.config.calibration_current = 12
    axis.motor.config.current_lim = motor_config['current_lim']
    axis.motor.config.requested_current_range = motor_config['requested_range']

    # Encoder config
    axis.encoder.config.mode = encoder_config['mode']
    axis.encoder.config.cpr = encoder_config['cpr']
    axis.encoder.config.calib_range = 0.1

    # Control mode config
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.pos_gain = ctrl_config['pos_gain']
    axis.controller.config.vel_gain = ctrl_config['vel_gain']
    axis.controller.config.vel_integrator_gain = ctrl_config['vel_integrator_gain']

    # Trajectory limits
    axis.trap_traj.config.vel_limit = traj_config['vel']
    axis.trap_traj.config.accel_limit = traj_config['accel']
    axis.trap_traj.config.decel_limit = traj_config['decel']
    axis.controller.config.vel_limit = traj_config['ctrl_vel']
    axis.controller.config.input_mode = InputMode.TRAP_TRAJ

# Calibration and closed loop state
def calibrate_axis(axis):
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(7.5)
    axis.motor.config.pre_calibrated = True

    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(17)
    axis.encoder.config.pre_calibrated = True

    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.input_pos = 0.0
    axis.controller.input_vel = 0.0
    axis.controller.input_torque = 0.0

# Connect to all ODrives
print("Finding all ODrives...")
odrives = []
for i in range(NUM_ODRIVES):
    # print(f"Connecting to ODrive {i+1}...")
    odrives.append(odrive.find_any())
    # print(f"Connected: {odrives[-1]}")

# Global ODrive config (can be adjusted per drive if needed)
for odrv in odrives:
    odrv.config.brake_resistance = 0.5
    odrv.config.dc_max_positive_current = 70
    odrv.config.dc_max_negative_current = -2.0
    odrv.config.max_regen_current = 0

# Motor/encoder/ctrl/traj configs for each axis (can be adjusted as needed)
motor_configs = [
    {'pole_pairs': 20, 'current_lim': 20.0, 'requested_range': 25.0},  # Hip
    {'pole_pairs': 7,  'current_lim': 60.0, 'requested_range': 70.0}   # Knee
]
encoder_configs = [
    {'mode': ENCODER_MODE_INCREMENTAL, 'cpr': 2400},  # Hip
    {'mode': ENCODER_MODE_HALL, 'cpr': 42}            # Knee
]
ctrl_configs = [
    {'pos_gain': 50.0, 'vel_gain': 0.05, 'vel_integrator_gain': 0.0},  # Hip
    {'pos_gain': 20.0, 'vel_gain': 0.02, 'vel_integrator_gain': 0.1}   # Knee
]
traj_configs = [
    {'vel': 10, 'accel': 5, 'decel': 5, 'ctrl_vel': 15.0},  # Hip
    {'vel': 20, 'accel': 10, 'decel': 10, 'ctrl_vel': 20.0} # Knee
]

# Configure and calibrate all axes
# zero_offsets = []

for odrv in odrives:
    for axis_num in [0, 1]:
        axis = getattr(odrv, f'axis{axis_num}')
        # joint_type = "hip" if axis_num == 0 else "knee"
        # print(f"Configuring {joint_type} motor on ODrive {i+1}")
        configure_axis(axis, motor_configs[axis_num], encoder_configs[axis_num], ctrl_configs[axis_num], traj_configs[axis_num])
        calibrate_axis(axis)
        # zero_offsets.append(axis.encoder.pos_estimate)

# Initialize calibration positions
calib_positions = [0.0] * 8  # 8 motors (0-7)
zero_offsets = [0.0] * 8     # To store encoder offsets later

# Loop for offset input
while True:

    leg_id = float(input("\nEnter motor index (0-7) to update or 69 to finish: "))
    if leg_id == 69:
        break

    if 0 <= leg_id < 8:
        pos = float(input(f"Enter new position for Motor {int(leg_id)}: "))
        calib_positions[int(leg_id)] = pos
    else:
        print("Invalid motor index.")

    # Update all motors with current calibration positions
    for i in range(4):  # 4 ODrives
        odrv = odrives[i]
        odrv.axis0.controller.input_pos = calib_positions[i * 2]
        odrv.axis1.controller.input_pos = calib_positions[i * 2 + 1]

# Store encoder zero positions
for i in range(4):
    odrv = odrives[i]
    zero_offsets[i * 2] = odrv.axis0.encoder.pos_estimate
    zero_offsets[i * 2 + 1] = odrv.axis1.encoder.pos_estimate

leg_x = 0.0
final_alpha = 0.0 
final_beta = 0.0

while leg_x != 69.0:

    for i in range(4):  # 4 ODrives
        odrv = odrives[i]
        odrv.axis0.controller.input_pos = zero_offsets[i * 2] + final_alpha 
        odrv.axis1.controller.input_pos = zero_offsets[i * 2 + 1] + final_beta

    height = float(input("Enter Height Pweaseee... : "))
    leg_x = float(input("Enter Leg X Pweaseee... : "))

    is_not_bakchod = False

    while not is_not_bakchod:
        try:
            alpha, beta = qlt.solve_M(L_1, L_2, leg_x, height)
            is_not_bakchod = True
        except Exception as e:
            print("Dhang se likh saale")
            height = float(input("Enter Height Pweaseee... : "))
            leg_x = float(input("Enter Leg X Pweaseee... : "))

    final_alpha = (((1.57 - alpha)*(1))/6.28)*10.5 
    final_beta = ((1.57 - alpha + beta - 1.12)*4.58)

for odrv in odrives:
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE