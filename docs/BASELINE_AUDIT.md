# Baseline audit — 2026-10-11

The inherited source was protected by a complete external backup before changes. Git metadata recovery and remote reconciliation are recorded by the coordinating agent. The inherited code already includes prediction (CV/CA), TTC/THW, FSM, IDM/MOBIL, Frenet lattice, polynomial trajectories, ST graph/DP, OSQP speed smoothing, CasADi longitudinal MPC, Stanley/LQR/PID, actuator delay and HAL.

## Executed baseline

- Windows Python 3.11: `python -m pytest -q`: **483 passed**, 2.06 s.
- `MPLBACKEND=Agg python -m demos.w07_dp_qp_speed_demo`: solved, dense ST safety PASS, max jerk 2.26226 m/s³.
- `MPLBACKEND=Agg python -m demos.w07_longitudinal_mpc_demo`: executed ST safety PASS, position RMSE 0.0521 m, speed RMSE 0.0297 m/s, max jerk 2.6019 m/s³.
- GUI demos require an interactive matplotlib backend; automated runs use Agg.

## Gaps at baseline

No CARLA adapter or real server regression, no C++ library, no ROS package, no unified safety supervisor, no dynamic bicycle / first-order longitudinal actuator, no training evidence. Existing HAL state has SI units but no timestamp; a timestamp envelope is required rather than changing established state constructors. Existing MPC checks discrete knot bounds; inter-knot ST checks remain the caller's responsibility. VehicleControlStack's trajectory-end branch coasts rather than stops; this requires correction.

## Environment / priority

CARLA 0.9.15 server installed on Windows; matching client is installed into WSL Python 3.10 because the Windows Python 3.11 wheel is unavailable. Ubuntu 22.04 / ROS Humble / GCC / CMake and GoogleTest sources are available. Python Windows has NumPy, SciPy, OSQP and CasADi. P0 order: safe timestamp contract, C++ numerical parity, actual CARLA I/O and multi-scenario evidence, ROS software linkage, dynamics/actuator perturbations, repeat regression and honest documentation. PPO / game experiments are P1 and do not substitute for any P0 acceptance.
