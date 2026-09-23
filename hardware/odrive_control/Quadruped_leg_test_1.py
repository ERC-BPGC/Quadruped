from __future__ import print_function
# import matplotlib
import time
import odrive
from odrive.enums import *
import time
import math
import odrive
from odrive.utils import start_liveplotter
import Quadruped_leg_test_algo as qlt
# Connect to your ODrive
odrv0 = odrive.find_any()
print(odrv0)
# Start liveplotter
# start_liveplotter(lambda: [odrv0.axis1.encoder.shadow_count])

# start_liveplotter(properties=[
#     odrv0.axis1.encoder._shadow_count_property,])

# start_liveplotter(lambda: [
#     odrv0.axis0.motor.current_control.Iq_setpoint,
#     odrv0.axis0.motor.current_control.Iq_measured])


odrv0.config.brake_resistance = 0.5

odrv0.config.dc_max_positive_current = 70

odrv0.config.dc_max_negative_current = -2.0

odrv0.config.max_regen_current = 0

# odrv0.save_configuration()

#hip
odrv0.axis0.motor.config.pole_pairs = 20
odrv0.axis0.motor.config.motor_type = MotorType.HIGH_CURRENT
odrv0.axis0.motor.config.resistance_calib_max_voltage = 10
odrv0.axis0.motor.config.calibration_current = 18
odrv0.axis0.motor.config.current_lim = 20.0
odrv0.axis0.motor.config.requested_current_range = 22.0

odrv0.axis0.encoder.config.mode = EncoderMode.INCREMENTAL
odrv0.axis0.encoder.config.cpr = 2400

#knee
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.motor_type = MotorType.HIGH_CURRENT
odrv0.axis1.motor.config.resistance_calib_max_voltage = 12
odrv0.axis1.motor.config.calibration_current = 22
odrv0.axis1.config.calibration_lockin.current = 33
odrv0.axis1.motor.config.current_lim = 60.0
odrv0.axis1.motor.config.requested_current_range = 70.0

odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.encoder.config.mode = EncoderMode.HALL
odrv0.axis1.encoder.config.cpr = 42

#hip
odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
time.sleep(7.5)
print('done calib 1')
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.calib_range = 0.1
odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
time.sleep(20)
print('done calib 2')

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.pos_gain = 50.0
odrv0.axis0.controller.config.vel_gain = 0.05
odrv0.axis0.controller.config.vel_integrator_gain = 0.0
odrv0.axis0.controller.input_vel = 0.0
odrv0.axis0.controller.input_pos = 0.0
odrv0.axis0.controller.input_torque = 0.0

#knee
odrv0.axis1.requested_state = AxisState.MOTOR_CALIBRATION
time.sleep(7.5)
print('done calib 1')
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.calib_range = 0.1
odrv0.axis1.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
time.sleep(17)
print('done calib 2')

odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis1.controller.config.pos_gain = 20
odrv0.axis1.controller.config.vel_gain = 0.02
odrv0.axis1.controller.config.vel_integrator_gain = 0.1
odrv0.axis1.controller.input_vel = 0.0
odrv0.axis1.controller.input_pos = 0.0
odrv0.axis1.controller.input_torque = 0.0

#hip
odrv0.axis0.trap_traj.config.vel_limit = 10
odrv0.axis0.trap_traj.config.accel_limit = 5
odrv0.axis0.trap_traj.config.decel_limit = 5
odrv0.axis0.controller.config.vel_limit = 15.0
odrv0.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ

#knee
odrv0.axis1.trap_traj.config.vel_limit = 20
odrv0.axis1.trap_traj.config.accel_limit = 10
odrv0.axis1.trap_traj.config.decel_limit = 10
odrv0.axis1.controller.config.vel_limit = 20.0
odrv0.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ

#Closed loop control
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

Calib_pos_hip = 0
Calib_pos_knee = 0

while Calib_pos_hip != 69.0:
    odrv0.axis0.controller.input_pos = float(Calib_pos_hip)
    odrv0.axis1.controller.input_pos = float(Calib_pos_knee)
    Calib_pos_hip = float(input("Enter Calib_pos_hip Pweaseee... : "))
    Calib_pos_knee = float(input("Enter Calib_Calib_pos_knee Pweaseee... : "))

zero_hip = odrv0.axis0.encoder.pos_estimate
zero_knee = odrv0.axis1.encoder.pos_estimate


final_alpha = float(input("Enter final_alpha Pweaseee... : "))
final_beta = float(input("Enter final_beta Pweaseee... : "))

L_1 = 25
L_2 = 33

leg_x = 3

while leg_x != 69.0:
    odrv0.axis1.controller.input_pos = float(zero_knee + final_beta)
    odrv0.axis0.controller.input_pos = float(zero_hip - final_alpha)

    height = float(input("Enter Height Pweaseee... : "))
    leg_x = float(input("Enter Leg X Pweaseee... : "))
    is_not_bakchod = False

    if leg_x == 69.0:
        break

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

odrv0.axis0.requested_state = AxisState.IDLE
odrv0.axis1.requested_state = AxisState.IDLE