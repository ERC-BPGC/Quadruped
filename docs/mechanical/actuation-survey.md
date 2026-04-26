---
title: Actuation Survey
---

# Actuation Survey

Quadruped actuation is a tradeoff between torque density, speed, impact tolerance, efficiency, cost, controllability, and mechanical complexity. This page is the background survey for choosing and explaining the actuator architecture of this robot.

## Quick terminology

In legged-robot documentation, **proximal** means closer to the robot body and **distal** means farther from the body. The hip is proximal, the foot is distal, and the knee sits between the upper and lower leg links.

<figure className="figure--small">
  <img src={require('./assets/actuation/terminology.jpg').default} alt="Quadruped leg terminology showing body, fore leg, hind leg, hip joint, thigh, knee joint, calf, and footpad" />
  <figcaption>Basic quadruped body and leg terminology. Source: figure from <a href="https://www.sciencedirect.com/science/article/abs/pii/S0022489825000151">this ScienceDirect paper</a>.</figcaption>
</figure>

For most quadruped legs:

- **Hip abduction/adduction** swings the leg sideways.
- **Hip flexion/extension** swings the leg forward and backward.
- **Knee flexion/extension** folds or extends the lower leg.
- **Upper leg / thigh / femur** usually means the link between hip pitch and knee.
- **Lower leg / calf / shank** usually means the link between knee and foot.

## Actuator families

| Actuation type | Typical idea | Strengths | Tradeoffs | Examples / references |
| --- | --- | --- | --- | --- |
| Direct electric drive | High-torque motor drives the joint with little or no reduction. | Backdrivable, simple torque control, low reflected inertia. | Heavy motors for high torque; often expensive. | Research actuators and quasi-direct-drive designs. |
| Quasi-direct-drive electric | High torque-density BLDC motor plus low-ratio transmission. | Good balance of torque, speed, backdrivability, and impact tolerance. | Needs careful motor, gearbox, thermal, and current-control design. | MIT Mini Cheetah and Ben Katz's modular actuator thesis. |
| High-ratio geared electric | Motor with large gearbox reduction. | Compact torque amplification and easy position holding. | Lower backdrivability, higher reflected inertia, poorer impact behavior unless torque sensing/compliance is added. | Many industrial/mobile robot joints. |
| Series elastic actuation | Elastic element between motor/transmission and output. | Force sensing through spring deflection, shock tolerance, compliance. | Extra mechanical volume, bandwidth limits, more modeling/control work. | ANYmal-style SEA literature, older legged platforms. |
| Hydraulic actuation | Hydraulic cylinders or rotary hydraulic actuators drive joints. | Very high force density and rugged dynamic motion. | Pumps, valves, fluid management, noise, maintenance, leakage risk, lower accessibility for a student build. | IIT HyQ hydraulic quadruped. |
| Pneumatic actuation | Compressed air drives cylinders or artificial muscles. | Naturally compliant and light at the actuator. | Harder precision control, air supply complexity, lower stiffness. | Soft robots and experimental legs. |

For our quadruped, the strongest practical candidate is usually **electric quasi-direct-drive**: BLDC motor, low-ratio transmission and current/torque control. This is the direction argued strongly by Ben Katz's thesis and the MIT Cheetah actuator literature.

## Leg transmission mechanisms

### Direct or coaxial rotary joints

The simplest architecture places a rotary actuator at each joint. It is easy to model and package conceptually, but knee and distal actuators can add leg inertia if they sit far from the body.

<figure>
  <img src={require('./assets/actuation/coaxial_actuation.webp').default} alt="Quadruped CAD model with direct or coaxial rotary leg actuation" />
  <figcaption>Direct/coaxial rotary joint layout, where actuators sit near the corresponding leg joints.</figcaption>
</figure>

### Push-rod knee mechanism

In a push-rod leg, the knee motor is placed closer to the hip/body and drives the knee through a rod-linkage transmission. This keeps the heavy actuator mass proximal while still moving the lower leg. The tradeoff is that the knee torque and speed depend on linkage geometry, so the mechanical advantage changes through the workspace.

**Examples:** Unitree Go1 and Go2 style legs. The Observable inverse-kinematics page and Sketchfab model are especially useful visual references for the Go2 leg layout.

<figure>
  <img src={require('./assets/actuation/pushrod_actuation.png').default} alt="Push-rod quadruped leg mechanism with proximal knee actuation" />
  <figcaption>Push-rod style knee actuation, with the motor placed proximally and a linkage transmitting motion to the lower leg.</figcaption>
</figure>

### Ball screw / screw actuator mechanism

A rotary motor drives a screw, converting rotation into linear motion. That linear actuator then drives a linkage at the knee. This can produce large forces in a compact leg, but it adds screw/nut friction, efficiency losses, speed limits, and wear considerations.

**Example:** Boston Dynamics Spot-style screw actuator architecture. The Boston Dynamics patent describes a motor-driven screw shaft and translating nut/carrier inside the upper leg, coupled by a linkage to rotate the lower leg about the knee.

<figure>
  <img src={require('./assets/actuation/ball_screw_actuation.webp').default} alt="Ball screw actuator driving a robot leg linkage" />
  <figcaption>Ball-screw style linear actuation converting motor rotation into linkage motion at the leg.</figcaption>
</figure>

### Belt-driven knee mechanism

A belt transfers torque from a motor near the hip to a pulley at the knee. This keeps mass proximal and can be efficient, compact, and backdrivable, but belt tensioning, skipped teeth, pulley sizing, backlash, and belt wear become design concerns.

**Examples:** MIT Mini Cheetah-style legs and several Deep Robotics quadruped designs use compact electric actuators with belt/transmission layouts to keep distal inertia low.

<figure>
  <img src={require('./assets/actuation/belt_actuation.webp').default} alt="Belt-driven quadruped leg transmission" />
  <figcaption>Belt-driven leg transmission with proximal actuation and routed torque to the lower leg.</figcaption>
</figure>

### Four-bar linkage leg

A four-bar leg uses a closed linkage to shape the foot path and transmit motion. It can package actuation neatly and create useful passive or compliant behavior, but the kinematics are less direct than a simple serial two-link leg and the torque transmission varies with pose.

**Examples:** four-bar research legs, including compliant/spring-assisted concepts.

## Mechanism examples

| Mechanism | Example robots / resources | Notes |
| --- | --- | --- |
| Push rod | [Unitree Go2 IK notebook](https://observablehq.com/@christophe-yamahata/inverse-kinematics-go2-robot), [Go2 leg Sketchfab model](https://sketchfab.com/models/d84607a0dc864921826cb092364d3990/embed) | Good visual reference for compact proximal actuation and knee linkage geometry. |
| Ball screw | [Spot video](https://www.youtube.com/watch?v=tfWbE_1eCZk), [Boston Dynamics screw actuator patent](https://patents.google.com/patent/US10253855B2/en), [ball screw leg video](https://www.youtube.com/watch?v=YrpHYPPWpIk) | Strong force packaging; useful to compare against Spot-style architecture. |
| Belt drive | [Ben Katz thesis](https://dspace.mit.edu/bitstream/handle/1721.1/118671/1057343368-MIT.pdf), [MIT Mini Cheetah article](https://news.mit.edu/2019/mit-mini-cheetah-first-four-legged-robot-to-backflip-0304), [Deep Robotics joints](https://www.deeprobotics.us/products/joints/) | Keeps moving mass close to the body and works well with quasi-direct-drive electric actuators. |
| Four bar | [Four-bar kinematic modeling paper](https://www.semanticscholar.org/paper/KINEMATIC-MODELING-OF-QUADRUPED-ROBOT-WITH-FOUR-BAR-Oak-Narwane/af2b230710d164cd3fc85b4caa0c05ced5aafb04), [4-bar compliant leg video](https://www.youtube.com/watch?v=bDxItdyQ3jc) | Useful when the linkage path/compliance is part of the design goal. |


## Core references

- Benjamin G. Katz, [A Low Cost Modular Actuator for Dynamic Robots](https://dspace.mit.edu/bitstream/handle/1721.1/118671/1057343368-MIT.pdf), MIT S.M. thesis, 2018.
- Patrick M. Wensing et al., [Proprioceptive Actuator Design in the MIT Cheetah](https://biomimetics.mit.edu/publications/4ebd1209-954e-4f9e-b64e-473e80b11318/), IEEE Transactions on Robotics, 2017.
- Jun He and Feng Gao, [Mechanism, Actuation, Perception, and Control of Highly Dynamic Multilegged Robots: A Review](https://cjme.springeropen.com/articles/10.1186/s10033-020-00485-9), Chinese Journal of Mechanical Engineering, 2020.
- Boston Dynamics, [Screw actuator for a legged robot](https://patents.google.com/patent/US10253855B2/en), US10253855B2.
- Christophe Yamahata, [Inverse Kinematics: Go2 Robot's Leg](https://observablehq.com/@christophe-yamahata/inverse-kinematics-go2-robot).
