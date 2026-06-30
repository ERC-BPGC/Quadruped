---
title: Gearboxes
---

# Gearboxes

Gearboxes sit between the motor and joint output. They reduce speed and multiply torque:

```text
output_torque = motor_torque * reduction_ratio * efficiency
output_speed  = motor_speed / reduction_ratio
```

For quadrupeds, the gearbox choice affects torque density, backdrivability, impact tolerance, backlash, efficiency, cost, and how easy the joint is to control.

## Common gearbox types

| Type | Basic idea | Pros | Cons |
| --- | --- | --- | --- |
| Planetary | Sun gear drives planet gears inside a ring gear. | Compact, common, good torque density, easy to source. | Backlash depends on quality, high ratios reduce backdrivability, multiple stages add complexity. |
| Cycloidal | Eccentric input drives a cycloidal disc against pins/rollers. | High reduction in small volume, good shock tolerance, strong torque capacity. | More complex geometry, can have vibration/ripple, precision manufacturing matters. |
| Harmonic | Wave generator flexes a spline against a circular spline. | Very high reduction, compact, low backlash. | Expensive, less backdrivable, flexspline fatigue risk under shock loads. |
| Worm | Screw-like worm drives a worm wheel. | Simple high reduction, compact right-angle drive, can be self-locking. | Low efficiency, heat generation, poor backdrivability, sliding wear. |

### Planetary gearbox

Planetary gearboxes are common in robotics because they package reduction compactly around the motor axis.

<div className="figure-grid figure-grid--two figure-grid--compact">
  <figure>
    <img src={require('./assets/gearboxes/planetary-gearbox.webp').default} alt="Planetary gearbox labeled with sun gear, planet gears, ring gear, and carrier" />
    <figcaption>Planetary gearbox layout: sun gear, planet gears, ring gear, and carrier.</figcaption>
  </figure>
  <figure>
    <img src={require('./assets/gearboxes/planetary_drive.gif').default} alt="Animated planetary gear motion" />
    <figcaption>Planetary gear motion.</figcaption>
  </figure>
</div>

### Cycloidal drive

Cycloidal drives use an eccentric input to move a cycloidal disc against pins or rollers, producing a large reduction in a compact volume.

<div className="figure-grid figure-grid--two figure-grid--compact">
  <figure>
    <img src={require('./assets/gearboxes/cycloidal-gearbox.jpg').default} alt="Exploded cycloidal drive with input shaft, eccentric bearing, cycloidal disk, ring gear, and output shaft" />
    <figcaption>Cycloidal drive components: eccentric input, cycloidal disc, ring pins/rollers, and output pins.</figcaption>
  </figure>
  <figure>
    <img src={require('./assets/gearboxes/cycloidal_drive.gif').default} alt="Animated cycloidal drive motion" />
    <figcaption>Cycloidal drive motion.</figcaption>
  </figure>
</div>

### Harmonic drive

Harmonic drives use a wave generator to flex a spline into a circular spline, creating high reduction with very low backlash.

<div className="figure-grid figure-grid--two figure-grid--compact">
  <figure>
    <img src={require('./assets/gearboxes/harmonic-gearbox.jpg').default} alt="Harmonic drive showing circular spline, flex spline, and wave generator" />
    <figcaption>Harmonic drive components: circular spline, flex spline, and wave generator.</figcaption>
  </figure>
  <figure>
    <img src={require('./assets/gearboxes/harmonic_drive.gif').default} alt="Animated harmonic drive motion" />
    <figcaption>Harmonic drive motion.</figcaption>
  </figure>
</div>

### Worm gearbox

Worm gearboxes use a screw-like worm to drive a worm wheel. They can produce high reductions and may be self-locking, but the sliding contact makes efficiency and heat important concerns.

<div className="figure-grid figure-grid--two figure-grid--compact">
  <figure>
    <img src={require('./assets/gearboxes/worm-gearbox.webp').default} alt="Cutaway worm gearbox showing worm and worm wheel" />
    <figcaption>Worm gearbox layout with worm and worm wheel.</figcaption>
  </figure>
  <figure>
    <img src={require('./assets/gearboxes/worm_drive.gif').default} alt="Animated worm gear motion" />
    <figcaption>Worm gear motion.</figcaption>
  </figure>
</div>

## Quadruped-specific tradeoff

Dynamic quadrupeds usually benefit from joints that can feel and react to ground impacts. That is why many modern electric quadrupeds avoid very high reductions and instead use direct or quasi-direct-drive actuators, belts, or moderate-ratio gearboxes.

Higher reduction helps torque, but it usually hurts:

- backdrivability,
- reflected inertia,
- impact tolerance,
- force-control bandwidth,
- mechanical transparency.

Lower reduction helps dynamic behavior, but demands a stronger motor and higher phase current.

## References and tools

### Planetary

- [Planetary gears explained](https://youtu.be/Ho4AniHtgxM?si=m1kBRgKtOiZX1tb_)
- [Planetary gearbox animation](https://youtu.be/9CDH4NMT_Pc?si=lIJQrVTVulQ4AMoQ)
- [Gear Generator](https://geargenerator.com/beta/) for quick visual gear sketches.

### Cycloidal

- [Cycloidal disc design video](https://m.youtube.com/watch?v=guvatctnjww&pp=ygUcSG93IHRvIGRlc2lnbiBjeWNsb2lkYWwgZGlzaw%3D%3D)
- [Cycloidal drive explanation](https://youtu.be/OsS9-FzKN6s?si=6UzUr6US2mjOdzF2)
- [Cycloidal drive animation](https://youtu.be/eM-8IO_mNIM?si=bPmpjyQm1xUHWZJ3)
- [Building a Cycloidal Drive with SOLIDWORKS](https://blog-assets.solidworks.com/uploads/sites/3/Building-a-Cycloidal-Drive-with-SOLIDWORKS.pdf)

### Harmonic

- [Harmonic drive explanation](https://youtu.be/IXmCze1GsGU?si=k866qfZEKOZKAqwM)
