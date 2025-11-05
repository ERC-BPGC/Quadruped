from __future__ import print_function
import matplotlib
import time
import odrive
from odrive.enums import *
import time
import math
import odrive
from odrive.utils import start_liveplotter

# Connect to your ODrive
odrv0 = odrive.find_any()
print(odrv0)
# Start liveplotter
# start_liveplotter(lambda: [odrv0.axis1.motor.current_control.final_v_alpha, odrv0.axis1.motor.current_control.final_v_beta])

# print(odrv0.config.dc_max_positive_current)
# print(odrv0.axis1.motor.config.current_lim)
# print(odrv0.axis1.motor.config.requested_current_range)

start_liveplotter(lambda: [
odrv0.axis1.motor.current_control.Iq_setpoint,
odrv0.axis1.motor.current_control.Iq_measured])

odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
time.sleep(7.5)
print('done calib 1')

odrv0.axis1.motor.config.pre_calibrated = True

odrv0.axis1.encoder.config.calib_range = 0.1

odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
time.sleep(20)
print('done calib 2')

odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis1.controller.config.pos_gain = 30.0
odrv0.axis1.controller.input_vel = 0.0
odrv0.axis1.controller.input_torque = 0.0
# odrv0.axis1.controller.input_torque = 0.0
# odrv0.axis1.controller.config.enable_current_mode_vel_limit = False
# odrv0.axis1.controller.input_vel = 0.0
odrv0.axis1.trap_traj.config.vel_limit = 20.0
odrv0.axis1.trap_traj.config.accel_limit = 30.0
odrv0.axis1.trap_traj.config.decel_limit = 30.0

# odrv0.axis1.config.motor.current_soft_max = 40.0
odrv0.axis1.controller.config.vel_limit = 30.0
odrv0.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


vel = float(input("Enter Velocity Pweaseee... : "))
while vel != 69.0:
    odrv0.axis1.controller.input_pos = float(vel)
    vel = float(input("Enter Velocity Pweaseee... : "))

odrv0.axis1.requested_state = AXIS_STATE_IDLE