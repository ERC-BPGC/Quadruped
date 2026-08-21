---
title: BLDC Motor Control
---

# BLDC Motor Control

BLDC motors are brushless DC motors: permanent magnets sit on the rotor, and coils on the stator are energized electronically to create a rotating magnetic field. They are common in robots because they have high power density, good efficiency, fast torque response, and no brush wear.

In a BLDC motor, the controller switches current through different stator phases so the magnetic field keeps pulling the rotor forward. To do this well, the controller needs to know or estimate rotor position, then energize the phases with the correct timing. Better timing gives smoother torque, higher efficiency, and better low-level control.

<div className="figure-grid figure-grid--two figure-grid--compact">
  <figure>
    <img src={require('../../mechanical/assets/bldc/bldc.gif').default} alt="Animated BLDC motor phase switching" />
    <figcaption>BLDC phase switching creates a rotating magnetic field.</figcaption>
  </figure>
  <figure>
    <img src={require('../../mechanical/assets/bldc/bldc1.gif').default} alt="Animated BLDC motor rotor following stator magnetic field" />
    <figcaption>The rotor follows the stator field as the controller commutates the phases.</figcaption>
  </figure>
</div>

## Video Resources

- [How does a brushless DC motor work?](https://www.youtube.com/watch?v=bCEiOnuODac)
- [BLDC motor control explanation](https://www.youtube.com/watch?v=IkRoEfy-PPQ)
