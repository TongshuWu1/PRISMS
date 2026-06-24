# Wall Tool Controller Notes

## Current Controller Stack

The simulator uses a hybrid cable-efficient controller:

- The reel is a velocity actuator. It controls radial cable length and carries most of the vertical load through bounded cable tension.
- The docked drone pair is an acceleration/force actuator. It mainly controls tangential swing and docked-body tilt, with limited radial rescue only when the cable-length error becomes large.
- The hold controller does not force zero tilt. It regulates the cable-supported equilibrium tilt after the position and velocity are settled.
- The reference layer is cable-aware. It slows down under high tracking error, high thrust use, allocation residual, or poor cable geometry.

## Planners Implemented

- `direct`: straight reference to the target.
- `center_setup`: robust hand-designed cable-friendly setup waypoint before hard high-side targets.
- `predictive`: constrained cable-aware route selection over candidate setup waypoints, using cable geometry, angular change, cable-length change, detour, and hard-target safety margins.
- `learned_ranked`: static learned cost map with robust fallback to `predictive` where a non-simulated learned route is risky.
- `sim_refined`: learned candidate ranking followed by dynamic simulation of route candidates, seeded with the robust predictive route.

The interactive simulator exposes the robust route:

```powershell
python wall_tool_sim\wall_tool_ui.py --planner predictive
```

## Research Basis

- MPC for CDPRs is attractive because tension and actuator limits can be part of the controller rather than repaired afterward.
- Flexible/suspended cable robots need smooth tension commands and bounded tension-rate behavior; otherwise the cable dynamics dominate.
- Underactuated cable systems cannot be judged only by quasi-static geometry. Dynamic feasibility and positive cable tension matter.
- Input shaping is a practical next layer for residual swing reduction.
- RL/model-free control is promising near difficult workspace boundaries, but should be trained and evaluated behind safety constraints, not trusted as the first controller.

Useful starting papers:

- Santos, Chemori, and Gouttefarde, "Model Predictive Control of Large-Dimension Cable-Driven Parallel Robots", 2019.
- Bettega et al., "Model predictive control for path tracking in cable driven parallel robots with flexible cables", 2023.
- Sreenath and Kumar, "Dynamics, Control and Planning for Cooperative Manipulation of Payloads Suspended by Cables from Multiple Quadrotor Robots", 2013.
- Baklouti et al., "Input-Shaping for Feed-Forward Control of Cable-Driven Parallel Robots", 2020.
- Dhakate et al., "CaRoSaC: A Reinforcement Learning-Based Kinematic Control of Cable-Driven Parallel Robots by Addressing Cable Sag through Simulation", 2025.

## Latest Benchmark Takeaway

The non-zero hold-equilibrium model removed the old upper-left instability: direct upper-left now scores `0.735` instead of the earlier `5.404` failure. The predictive setup route is still more cable-efficient on hard upper-side targets: upper-right improves from `direct score=0.646` to `predictive score=0.618`, and upper-left improves from `direct score=0.735` to `predictive score=0.626`.

The learned static-cost route initially found a numerically brittle waypoint near the upper-right case. That route could fail from tiny waypoint perturbations, so the planner now snaps generated candidate waypoints and uses the constrained predictive route as a safety prior unless dynamic simulation explicitly proves a better route.

## Best Next Step

Upgrade the current constrained predictive planner into a short-horizon MPC/reference optimizer over the pendulum coordinates:

- State: cable length, cable angle, body tilt, and rates.
- Inputs: spool velocity command and tangential drone force/torque request.
- Constraints: positive cable tension, spool velocity, drone thrust, tilt torque, cable geometry margin, and tension-rate limits.
- Cost: final tracking error, induced-power proxy, spool work, tension variation, and residual swing.

This should sit above the existing low-level hybrid controller first. After it works, use RL or model learning as a route/terminal-cost improver, not as the only safety-critical controller.
