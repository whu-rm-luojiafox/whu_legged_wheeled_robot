# Chassis model tools

Build commands and architecture: [project README](../../README.md).

| Path | Purpose |
| --- | --- |
| `model/export_fixed_leg_model.m` | Export the fixed-leg continuous/discrete model |
| `model/generated/` | Exported model data |
| `mpc/generate_solver.py` | Generate MPC parameters and runtime adaptations |
| `mpc/requirements.txt` | Dependencies for the offline parameter generator |

Standalone tests, host validation, simulation and board probes have been removed.
The supported firmware presets are `chassis` and `gimbal`.

The MPC algorithm source is retained for reference and parameter generation; it
is not connected to the vehicle control loop. Its existing fixed-leg model uses
0.24 m legs, a 4 ms sample period, 10 states and 4 inputs.

The follow adapter is `chassis_update_yaw_follow()` in
`src/chassis/module/chassis_motion.c`. Control flow and tuning are documented in
[the chassis guide](../../docs/chassis-control.md).
