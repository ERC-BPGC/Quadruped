---
id: intro
title: Project Overview
sidebar_position: 1
slug: /intro
---

# ERC Quadruped

This documentation is the handoff point for the ERC-BPGC quadruped project: the hardware, electronics, code, experiments, and open questions needed for the next team to continue from the current state instead of rediscovering it from scratch.

The site is intentionally split by subsystem so a new contributor can enter from the part they care about:

- **Mechanical:** CAD, frame architecture, links, joints, assembly notes, manufacturing constraints.
- **Electronics:** power, wiring, motor drivers, compute, sensors, connectors, safety checks.
- **Software:** code map, scripts, dependencies, runtime flow, logging, test utilities.
- **Controls:** kinematics, gait logic, motor control, simulation-to-hardware notes.
- **Simulation:** Genesis/MJCF/URDF assets and training or evaluation workflows.

The first pass of these docs is a scaffold. The goal is to keep filling each page with source links, photos, diagrams, commands, known failure modes, and design reasoning.

## Repository layout

```text
Quadruped/
  docs/                  Documentation source for this site
  src/                   Docusaurus homepage and styling
  static/                Images and static assets for the site
  Genesis/               Simulation and project source material
  README.md              GitHub landing page
```

## What good looks like

Every mature page should eventually answer five questions:

1. What is this subsystem responsible for?
2. Where are the source files, CAD files, schematics, or scripts?
3. How do I build, assemble, run, or test it?
4. What is known to work, and what is still unreliable?
5. What should the next contributor try first?
