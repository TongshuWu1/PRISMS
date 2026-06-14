# Controller

This folder is for control algorithms only.

- `low_level/rate_control.py`: simulator-independent Crazyflie-style body-rate PI controller, motor mixer, motor dynamics, and motor angular-velocity output.
- `low_level/body_rate.py`: CoppeliaSim plant wrapper for the low-level controller, including simulated force/torque application and propeller visualization.
- `high_level/position.py`: position/yaw controller that outputs collective thrust plus roll, pitch, and yaw-rate commands.
- `high_level/assembly_geometric.py`: docked-assembly COM/attitude controller that outputs a desired world-frame wrench.
- `allocation/geometry.py`: reconstructs each motor position/thrust axis relative to the docked assembly center of mass.
- `allocation/wrench_allocator.py`: maps the desired assembly wrench to motor angular velocity commands.
- `common/telemetry.py`: shared controller telemetry logging.

UI code and CoppeliaSim experiment loops live in `scripts/experiments/`.
Magnetic docking and contact/latch physics live in `simulation/`.
