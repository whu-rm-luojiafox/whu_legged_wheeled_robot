# Chassis model and validation tools

Build commands and architecture: [project README](../../README.md).

| Path | Purpose |
| --- | --- |
| `model/export_fixed_leg_model.m` | Export a fixed-leg continuous/discrete model |
| `model/generated/` | Exported model data |
| `mpc/generate_solver.py` | Generate MPC caches and reproducible runtime patches |
| `mpc/run_host_checks.py` | Build native targets through root CMake and run CTest |
| `mpc/host_probe.cpp` | Diagnostic executable with solver internals |
| `mpc/board_probe.c/.h` | DWT measurement record for the standalone MPC firmware |
| `simulation/compare_lqr_mpc.py` | Independent QP comparison, failure guards and linear rollout |

Run from the project root:

```powershell
python tools/chassis/mpc/run_host_checks.py
# Only after changing model/solver parameters:
python tools/chassis/mpc/run_host_checks.py --regenerate
```

Builds and reports go into `build/`, including `build/host/reports/comparison.json`.
The obsolete MATLAB LQR replay is in the sibling legacy archive, as it
depends on firmware parameters that no longer exist.

Current MPC model: both legs 0.24 m, prediction sample 4 ms, 10 states,
4 inputs, 8 state knots, 7 input intervals, rho=1, 200 iterations and 0.001
tolerance. Model torque bounds are +/-6 N*m for wheels and +/-12 N*m for
virtual leg torques. The latter are not physical joint motor limits.
Terminal weight is Pinf-rho*I, not the model export's Pd.

The native checks cover 24 double/float, bounded/unbounded QP cases,
5 non-convergence guards and a 2-second linear rollout. Host timings do not
represent F407 timings. The C API is single-instance, non-reentrant and
requires heap allocation. A failed solve sets valid=0 and clears command;
candidate is diagnostic only. Live state adaptation, moving references,
variable leg lengths, VMC integration and on-vehicle MPC control remain
outside this experiment.
