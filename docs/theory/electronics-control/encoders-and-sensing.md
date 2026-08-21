---
title: Encoders and Sensing
---

# Encoders and Sensing

Encoders measure position. In a quadruped, they are one of the most important sensors because the controller needs to know where each motor and joint actually is before it can control torque, velocity, position, or leg kinematics.

Bad encoder data can make a good actuator look terrible. A loose magnet, wrong offset, noisy cable, or reversed direction can cause jitter, heating, instability, or violent motion.

## Common Sensing Technologies

| Type | Basic idea | Notes |
| --- | --- | --- |
| Hall sensors | Detect rotor magnetic pole transitions. | Cheap and robust, but low resolution. Common inside BLDC motors for basic commutation. |
| Magnetic encoder | Uses a magnetic field sensor IC and a small magnet mounted on the rotating shaft. | Compact and common in hobby/robotics builds. Many chips provide 10-12 bit or higher angle readings. |
| Optical encoder | Uses light and slots/marks on a disk to measure rotation. | Can be high resolution, but needs cleaner mechanical alignment and protection from dust/oil. |

<div className="figure-grid figure-grid--three figure-grid--compact">
  <figure>
    <img src={require('../../mechanical/assets/encoders/hall.jpg').default} alt="Hall sensor based motor position sensing" />
    <figcaption>Hall sensors are simple and robust, but usually low resolution.</figcaption>
  </figure>
  <figure>
    <img src={require('../../mechanical/assets/encoders/magnetic.avif').default} alt="Magnetic encoder with diametric magnet and sensor IC" />
    <figcaption>Magnetic encoders are compact and common in robot joints.</figcaption>
  </figure>
  <figure>
    <img src={require('../../mechanical/assets/encoders/optical.jpg').default} alt="Optical rotary encoder disk and sensor" />
    <figcaption>Optical encoders can be precise, but need clean alignment.</figcaption>
  </figure>
</div>

## Incremental vs Absolute

Incremental and absolute describe how the encoder reports position. They are separate from the physical sensing method: magnetic, optical, and other encoder technologies can be designed as either incremental or absolute sensors.

### Incremental Encoders

Incremental encoders report motion as pulses relative to a starting point. They are useful for measuring speed and relative movement, but after power-on the controller does not automatically know the absolute shaft angle.

To recover absolute position, incremental systems usually need:

- a homing routine,
- a limit switch,
- an index pulse,
- or a known startup pose.

### Absolute Encoders

Absolute encoders report the actual angle immediately after power-on. This is very useful for robot joints because the controller can know the leg pose before moving the robot.

Absolute encoders still need calibration. The sensor may know its own shaft angle, but the robot still needs a zero offset that maps that sensor angle to the mechanical joint convention.

## PPR, CPR, and Resolution

Encoder specs can be confusing because manufacturers use different terms.

| Term | Meaning |
| --- | --- |
| `PPR` | Pulses per revolution. Often used for incremental encoders. |
| `CPR` | Counts per revolution. With quadrature decoding, one pulse cycle can produce multiple counts. |
| Resolution | Smallest angle step the sensor can report. |
| Bit depth | For digital absolute encoders, `bits` determine how many discrete positions exist per revolution. |

For an absolute encoder:

```text
counts_per_revolution = 2^bits
```

Examples:

| Bit depth | Counts per revolution | Angle per count |
| --- | --- | --- |
| 10-bit | 1024 | about 0.35 deg |
| 12-bit | 4096 | about 0.088 deg |
| 14-bit | 16384 | about 0.022 deg |

For a quadrature incremental encoder, the effective count rate depends on how the signals are decoded. If an encoder is listed as `PPR`, the controller may count 1x, 2x, or 4x that value depending on whether it counts rising edges only, both edges, or all quadrature transitions.

## Motor Angle vs Joint Angle

An encoder can measure either:

- motor shaft angle,
- gearbox output angle,
- joint angle directly.

These are not always the same. If there is a gearbox, belt, linkage, or push rod between the motor and joint, motor angle must be converted to joint angle using the mechanism ratio and geometry.

Motor-side encoders are usually easier to package and useful for motor commutation. Joint-side encoders give better knowledge of the actual joint position, especially when there is backlash, belt stretch, compliance, or linkage nonlinearity.

## Common Failure Points

### Bad Mechanical Mounting

This is one of the most common mistakes in hobby and student robot builds.

For magnetic encoders:

- the magnet must be centered over the sensor,
- the air gap must be within the sensor's recommended range,
- the magnet must be mounted flat and coaxial with the rotating shaft,
- the magnet must not wobble, slip, or tilt during motion,
- the sensor PCB must be rigidly mounted,
- glue alone is risky if the magnet can shift under vibration.

For optical encoders:

- the disk must be concentric with the shaft,
- the sensor gap and alignment must be correct,
- the disk must not rub or wobble,
- dust, grease, and scratches can corrupt readings.

If the encoder is mechanically misaligned, the software may still receive numbers, but those numbers can be nonlinear, noisy, delayed, or completely wrong.

### Wrong Direction

If motor current produces positive torque but the encoder angle decreases, the control loop may run away instead of stabilizing. Always check encoder sign before enabling high current.

### Bad Zero Offset

Absolute encoders still need a joint zero convention. If the offset is wrong, the robot may think a safe pose is actually outside its limits, or inverse kinematics may command the wrong motion.

### Noise and Wiring Issues

Encoder signals can be corrupted by:

- long unshielded wires,
- routing sensor wires near motor phase wires,
- poor grounding,
- loose connectors,
- bad crimping,
- weak pull-ups,
- low supply voltage,
- communication bus errors.

### Insufficient Resolution

Low-resolution sensors can make control rough, especially for slow joint motion or velocity estimation. A motor may sound noisy or feel jerky simply because the controller cannot measure position smoothly enough.

### Backlash or Compliance

If the encoder is on the motor side, backlash or flex after the encoder will not be measured. The controller may think the motor is positioned correctly while the actual joint is slightly behind or oscillating.

## Practical Checks

- Rotate the joint by hand and plot encoder angle.
- Check that the angle changes smoothly without jumps.
- Verify direction/sign against the joint convention.
- Mark a physical zero position and confirm the reported offset.
- Wiggle cables and connectors while watching readings.
- Run motors at low current first.
- Recheck encoder mounting after impacts or transport.
