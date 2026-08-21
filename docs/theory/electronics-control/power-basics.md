---
title: Power Basics
---

# Power Basics

Robot power is not just about choosing a battery with the right voltage. A quadruped has motors that draw large changing currents, electronics that need clean regulated power, and safety risks that become very real when many actuators can move at once.

This page is a quick background reference for the power concepts that matter before working on the robot.

## Voltage, Current, and Power

Voltage is the electrical potential available to push current through a circuit. Current is the flow of charge. Power is the rate at which electrical energy is delivered:

$$
P = VI
$$

For a robot, this means:

- higher voltage can deliver the same power with lower current,
- high current causes heating in wires, connectors, batteries, and motor drivers,
- every wire, connector, switch, fuse, PCB trace, and driver must be rated for the expected current,
- motors can draw much more current during startup, hard acceleration, collision, or stall than they do during light motion.

## Batteries

Legged robots commonly use lithium battery packs because they have high power density. The important battery parameters are:

| Parameter | Why it matters |
| --- | --- |
| Nominal voltage | Sets the main bus voltage for motor drivers and regulators. |
| Capacity | Roughly determines runtime. |
| Discharge rating | Determines how much current the battery can safely supply. |
| Internal resistance | Causes voltage sag and heating under high current. |
| Connector rating | A weak connector can heat up or fail even if the battery itself is fine. |

Battery voltage is not constant. It changes with charge level, load current, and battery health. A motor controller must tolerate the full battery voltage range, not just the nominal value.

## Power Distribution

A quadruped usually has at least two power domains:

- **high-power motor bus:** battery voltage feeding motor drivers,
- **low-voltage electronics rails:** regulated power for compute, sensors, microcontrollers, and communication hardware.

Keep these ideas in mind:

- Motor current paths should be short, thick, and mechanically secure.
- Logic power should be regulated and protected from motor noise.
- Grounds must be planned carefully so high motor currents do not corrupt sensor or communication signals.
- Current return paths matter as much as positive supply paths.

## Protection and Safety

Useful safety elements include:

- **Fuses or resettable protection:** disconnect power during severe overcurrent faults.
- **Emergency stop:** provides a fast way to disable actuator power.
- **Reverse-polarity protection:** protects electronics if a connector is accidentally reversed.
- **Pre-charge or soft-start:** limits inrush current into large capacitors.
- **Current limits in motor controllers:** prevent motors and drivers from pulling destructive current.
- **Clear power-up sequence:** avoids enabling motors before sensors and controllers are ready.

Before applying power, always check polarity, continuity, expected resistance between supply rails, connector orientation, and whether anything conductive is loose inside the robot.

## Capacitors and Voltage Spikes

Motors and drivers create fast current changes. Wiring inductance resists sudden current changes, which can create voltage spikes on the power bus. Capacitors near motor drivers help absorb short spikes and supply quick bursts of current.

Common capacitor roles:

- **bulk capacitance:** supports the main bus during short current bursts,
- **decoupling capacitance:** filters high-frequency noise near electronics,
- **driver input capacitance:** stabilizes the motor controller supply.

Capacitors are useful, but they also create inrush current when the robot is first connected to a battery. Large capacitor banks may need pre-charge or soft-start circuitry.

## Safe Measurement Habits

- Start with a current-limited bench supply when testing new electronics.
- Use a multimeter to verify voltage before plugging in sensitive boards.
- Never assume connector polarity from color alone.
- Keep one hand away from the circuit when probing high-current systems.
- Secure the robot mechanically before enabling actuators.
- Disable motors before changing wiring.

## Video Resource

- [Electricity basics for robotics/electronics](https://www.youtube.com/watch?v=Iye4uVLmj8o)
