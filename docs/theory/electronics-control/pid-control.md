---
title: PID Control
---

# PID Control

PID control is one of the most common control ideas in robotics. It shows up anywhere we want a system to follow a target: motor velocity, joint position, joint torque/current, body attitude, leg height, or even higher-level balance behavior.

For legged robots, PID-style feedback is especially important at the low level. Even if the robot eventually uses advanced gait planning, model predictive control, reinforcement learning, or whole-body control, the actuators still need stable feedback loops that can track commands quickly and reject disturbances.

This page is only a resource hub for now. The best way to understand PID is to watch it, tune it, and see how the response changes.

## Video Resources

- [Understanding PID control - MATLAB Tech Talks playlist](https://www.youtube.com/watch?v=wkfEZmsQqiA&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)
- [PID control explanation](https://www.youtube.com/watch?v=qKy98Cbcltw)
- [PID controller basics](https://www.youtube.com/watch?v=JFTJ2SS4xyA)
- [PID tuning / control demo](https://www.youtube.com/watch?v=dMRDzicSvXk&t=799s)

## Interactive Demos

- [PID controller interactive demo](https://codepen.io/oscarsaharoy/full/LYbmVma)
- [PID drone control demo](https://pid-drone-control.vercel.app/)

## Where PID Shows Up On A Quadruped

- motor current/torque loops,
- motor velocity loops,
- joint position loops,
- leg endpoint tracking,
- body roll/pitch stabilization,
- balancing and disturbance rejection.

The main lesson: PID is simple, but not optional. A poorly tuned low-level loop can make even a good mechanical design look unstable.
