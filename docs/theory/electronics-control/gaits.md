---
title: Gaits
---

# Gaits

A gait is the repeated footfall pattern a legged robot uses to move. It defines which legs are in stance, which legs are in swing, and how the body shifts while the feet move.

In quadrupeds, the gait affects stability, speed, efficiency, ground reaction forces, and how hard the controller has to work.

## Common Gaits

| Gait | Basic idea | Notes |
| --- | --- | --- |
| Walk | Legs move one at a time. | Slow, stable, and easy to reason about. |
| Crawl | Very slow walking with a large stability margin. | Useful for careful motion or rough terrain. |
| Trot | Diagonal leg pairs move together. | Common for quadruped robots because it balances speed and simplicity. |
| Pace | Same-side legs move together. | Can be fast, but body roll control becomes important. |
| Bound | Front legs and rear legs move as pairs. | Dynamic gait used for faster motion. |
| Gallop | Asymmetric high-speed gait. | More dynamic and harder to control. |

<figure className="figure--small">
  <img src={require('../../mechanical/assets/gaits/all_gaits.gif').default} alt="Animation comparing common quadruped gait patterns" />
  <figcaption>Common quadruped gait patterns and their leg timing.</figcaption>
</figure>

## Useful Terms

- **Stance phase:** the foot is on the ground and supporting/pushing the body.
- **Swing phase:** the foot is in the air moving to the next contact point.
- **Duty factor:** fraction of the gait cycle that a foot spends in stance.
- **Support polygon:** area formed by the stance feet; the robot is more statically stable when the center of mass projection stays inside it.

## Resource

- [Animator Notebook: Quadrupeds Gaits](https://www.animatornotebook.com/learn/quadrupeds-gaits)
