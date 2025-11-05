from __future__ import print_function
import odrive
import time

# Connect to ODrive
print("Connecting to ODrive...")
odrv0 = odrive.find_any()
print("Connected!")

# Print general settings
print("\n--- General ODrive Settings ---")
print(f"DC Max Positive Current: {odrv0.config.dc_max_positive_current} A")
print(f"DC Max Negative Current: {odrv0.config.dc_max_negative_current} A")
print(f"DC Bus Voltage: {odrv0.vbus_voltage} V")

# Select the axis (change to axis0 if needed)
axis = odrv0.axis1  

# Print Motor Config
print("\n--- Motor Configuration ---")
print(f"Motor Type: {axis.motor.config.motor_type}")
print(f"Current Limit: {axis.motor.config.current_lim} A")
print(f"Requested Current Range: {axis.motor.config.requested_current_range} A")
print(f"Phase Resistance: {axis.motor.config.phase_resistance} Ω")
print(f"Phase Inductance: {axis.motor.config.phase_inductance} H")
print(f"Current Control Bandwidth: {axis.motor.config.current_control_bandwidth} Hz")
print(f"Calibration Current: {axis.motor.config.calibration_current} A")

# Print Encoder Config
print("\n--- Encoder Configuration ---")
print(f"Encoder Mode: {axis.encoder.config.mode}")
print(f"Encoder CPR: {axis.encoder.config.cpr}")
print(f"Encoder Bandwidth: {axis.encoder.config.bandwidth}")
print(f"Encoder Calibration Range: {axis.encoder.config.calib_range}")

# Print Controller Config
print("\n--- Controller Configuration ---")
print(f"Control Mode: {axis.controller.config.control_mode}")
print(f"Input Mode: {axis.controller.config.input_mode}")
print(f"Position Gain: {axis.controller.config.pos_gain}")
print(f"Velocity Gain: {axis.controller.config.vel_gain}")
print(f"Velocity Integral Gain: {axis.controller.config.vel_integrator_gain}")
print(f"Torque Gain: {axis.controller.config.torque_gain}")

# Print Thermal Status
print("\n--- Thermal Monitoring ---")
print(f"FET Thermistor Temperature: {axis.motor.fet_thermistor.temperature} °C")
print(f"Motor Thermistor Temperature: {axis.motor.motor_thermistor.temperature} °C")

# Print Live Current Limits
print("\n--- Live Current Monitoring ---")
print(f"Iq Setpoint: {axis.motor.current_control.Iq_setpoint} A")
print(f"Iq Measured: {axis.motor.current_control.Iq_measured} A")
print(f"Bus Current: {odrv0.ibus_}{axis.motor.current_control.final_v_alpha}")

print("\nDone checking parameters!")
