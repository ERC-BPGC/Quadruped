---
title: Leg Design
---

# Leg Design

The leg design evolved through prototyping. Since this was the first time most of the team was working on a system this large, we explored multiple actuation mechanisms before settling on the current architecture.

The goal of this page is to preserve that design history: what we tried, why we chose it, what worked, and why the next version changed.

## Image Folder

Put leg design images here:

```text
docs/mechanical/assets/leg-design/
```

Suggested filenames:

- `v0-leg-cad.png`
- `v0-ballscrew-knee.png`
- `v0-planetary-hip.png`
- `v1-leg-cad.png`
- `v1-cycloidal-tilt.png`
- `v1-pushrod-knee.png`

## Design Evolution

The first major decision was not just which motor to use, but how to transmit motor torque to each joint. Quadruped legs can use belts, gearboxes, ballscrews, linkages, direct drive, or combinations of these. Each option has tradeoffs in manufacturability, backlash, packaging, efficiency, mass, and repairability.

For our first robot, we prioritized mechanisms that we could actually manufacture, assemble, and iterate on with the tools and vendors available to us.

## V0: 2DOF Leg Prototype

The first prototype leg, V0, was a 2DOF leg with:

- hip actuation,
- knee actuation,
- no tilt/abduction-adduction axis.

This let us focus on the sagittal-plane leg mechanics before attempting a full 3DOF quadruped leg.

### Knee: Ballscrew Mechanism

The knee mechanism was the first major subsystem we tackled. After looking through quadruped actuator examples and available parts, we chose a ballscrew-based knee mechanism inspired by Spot-style linear actuator legs.

We chose this because:

- the mechanism was relatively easy to understand and manufacture,
- it avoided designing a compact custom gearbox for the knee,
- it avoided belt tensioning,
- ballscrews and linear motion parts were easier to source,
- the layout gave a clean way to actuate the calf link through push rods.

The actuation path was:

```text
motor rotation -> ballscrew linear motion -> 6 mm SS push rods -> calf link rotation
```

The knee used a Flipsky 5065 skateboard BLDC motor. It was chosen because the rough speed/torque estimate looked feasible, the long thin form factor fit the design well, and the motor had a built-in encoder.

TODO: Add the ballscrew-to-calf speed/torque ratio calculation here.

TODO: Add V0 knee CAD image or interactive CAD view here.

### Hip: Planetary Gearbox

For the hip/thigh link actuator, V0 used an Eaglepower 8308 pancake BLDC motor with a two-stage planetary gearbox. The total reduction was approximately 10:1.

This architecture was chosen because a pancake BLDC plus gearbox is a common setup for legged robot joints, and a planetary gearbox was one of the easiest reductions for us to start designing and manufacturing with 3D printing.

We used a two-stage design because a single-stage planetary with the required reduction would have made the 3D printed planet gears very small. That would have made them more likely to fracture under load.

TODO: Add V0 hip gearbox CAD image here.

TODO: Add gear ratio breakdown and expected output torque here.

## V1: Current 3DOF Leg

For the next version, V1, we moved to a full 3DOF leg while reusing whatever we could from V0.

The current leg has:

- tilt/abduction-adduction actuation,
- hip actuation,
- knee actuation.

### Tilt: Dual-Disc Cycloidal Actuator

The new tilt axis needed a compact actuator with enough torque for a full 3DOF leg. The old two-stage planetary gearbox was bulky and mechanically weak, so we wanted to move toward metal manufacturing.

The original idea was to make a single-stage metal planetary gearbox. However, we could not find suitable vendors for custom gear manufacturing at the time. We then moved to a cycloidal drive because its main profiles could be laser or plasma cut from metal sheet.

The V1 tilt actuator uses a dual-disc cycloidal drive. The dual-disc layout helps balance the drive compared to a single cycloidal disc.

Key design constraints:

- parts should be manufacturable from metal sheet,
- maximum practical sheet thickness was about 5 mm,
- the mechanism had to fit within the leg packaging,
- tolerances mattered a lot.

Cycloidal drives are sensitive to manufacturing tolerances, but after a few iterations we got the mechanism working.

TODO: Add cycloidal actuator CAD image and iteration notes here.

TODO: Add reduction ratio, eccentricity, pin count, and tolerance notes here.

### Hip: Reused Planetary Actuator

The V1 hip actuator reused the V0 hip setup:

```text
Eaglepower 8308 pancake BLDC -> two-stage planetary gearbox -> hip/thigh link
```

This was not necessarily the ideal final actuator, but reusing it helped us move toward a full 3DOF leg faster.

TODO: Add notes on observed weakness, backlash, or failures in the 3D printed planetary gearbox.

### Knee: Push/Pull Rod Mechanism

For V1, we initially tested the old ballscrew knee mechanism. It worked as a prototype, but it had two major drawbacks:

- it was heavy,
- it limited the final leg speed.

We then switched to a Go2-like push/pull rod mechanism. In the current version, the knee uses:

```text
Eaglepower 8308 pancake BLDC -> small pinion gear set -> push/pull rod -> calf link
```

This removed the heavy ballscrew and gave us a faster, more compact knee actuation path.

TODO: Add pinion ratio, linkage geometry, and knee torque/speed calculation here.

TODO: Add current V1 knee mechanism CAD image here.

## Current V1 Actuator Layout

| Joint | Mechanism | Motor | Notes |
| --- | --- | --- | --- |
| Tilt | Dual-disc cycloidal drive | Eaglepower 8308 BLDC | Metal sheet-manufacturable actuator for the new 3DOF leg. |
| Hip | Two-stage planetary gearbox | Eaglepower 8308 BLDC | Reused from V0; compact enough, but mechanically limited. |
| Knee | Pinion gear set + push/pull rod | Eaglepower 8308 BLDC | Replaced the heavier ballscrew mechanism. |

## Details To Add

- V0 and V1 CAD screenshots.
- Mechanism photos.
- Ballscrew speed/torque conversion calculation.
- Planetary gearbox ratio breakdown.
- Cycloidal reduction and tolerance notes.
- Push/pull rod linkage ratio and speed/torque estimate.
- Failure modes from testing.
- What we would redesign in the next revision.
