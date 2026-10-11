# FalconPlan — verified software stage, 2026-10-11

GitHub: https://github.com/Leo-7GOAT/FalconPlan. Preserved checkpoint at report preparation: `e1718ac354949e80a18cc69e91800ed64c7017d0`. The coordinator commits/pushes subsequent verified work; use `git rev-parse HEAD` for the final revision. This is a stage report, not closure of all acceptance gates.

The original matrix retains all **16 equally weighted weeks: W01–W12 and W16–W19**. It has **9 PASS / 7 PARTIAL = 56.25% week-level PASS**. This counts accepted week groups, not engineering effort or the entire project's detailed requirement completion. The independent [full requirement ledger](REMAINING_ACCEPTANCE_REVIEW.md) records 127/149 individual checks PASS at its review checkpoint. Historical W01–W05 assignments are inferred from repository history because the task does not define those weeks individually. A component pass never closes its broader week automatically.

| Week | Status | Evidence / remaining boundary |
|---|---|---|
| W01 foundations | PASS_UNIT | Inherited vehicle/frame/control regression; historical mapping inferred |
| W02 prediction/HAL | PASS_UNIT / PASS_MOCK | Prediction/risk tests and SI rear-axle contract; historical mapping inferred |
| W03 behavior | PASS_UNIT | CV/CA, risk, FSM, IDM, MOBIL; live truth actor orchestration |
| W04 path/control | PASS_UNIT | Inherited lateral path/controller demos and tests; historical mapping inferred |
| W05 integration | PASS_UNIT | Inherited planning/integration tests; historical mapping inferred |
| W06 ST-DP | PASS_UNIT | Analytical demo, live dynamic ST projection, corridor and dense checks |
| W07 QP/MPC | PASS_REAL_SIM algorithm execution | DP-QP/rolling MPC numerical acceptance and real online scenes; plant constraints remain W09/W16 gates |
| W08 C++17 | PASS_UNIT / PASS_REAL_SIM | 9 GoogleTests Release + ASan/UBSan; native controller, model, data structures and actual candidate cost/collision/selection; 2,000 candidate parity cases and CARLA online execution. The remaining DP/QP/MPC executive is Python. |
| W09 CARLA/estimation | PARTIAL | Real physics/I/O and corrected nominal GNSS/IMU diagnostics; jerk, noisy/dropout sensor acceptance, sensor feedback and full real-time deadlines open |
| W10 safety | PASS_REAL_SIM software I/O | Native ROS-CARLA brake-to-zero, geofence/freshness/manual/latching faults; physical E-stop pending |
| W11 PPO | PARTIAL | No trained result claimed |
| W12 game decisions | PARTIAL | No evaluated game experiment claimed |
| W16 dynamics/actuators | PARTIAL | Dynamic/actuator unit checks and real throttle sweeps; high-speed low-friction control FAIL and complete identification open |
| W17 learning/game experiments | PARTIAL | No accepted seeded policy experiment |
| W18 policy OOD | PARTIAL | Model OOD failure exists; learned policy OOD experiment remains |
| W19 engineering | PARTIAL | Packaging/replay/profiling/release tools and remote CI; broader engineering/deployment acceptance remains |

Physical vehicle / Pi HIL is **PENDING_HARDWARE**: protocol, geometry, wiring, true E-stop and target timing are unmeasured. It is reported separately and does not shrink the 16-week denominator.

## Executed checkpoints

Inherited Python: 483 passed. Earlier controller-binding checkpoint: Windows **524 passed, 2 Linux-only skips**; WSL **526 passed in 6.82 s**, with raw profiler/stdout evidence in `artifacts/final_stage_regression*`. Independent coordinator rerun also passed 526 tests (`artifacts/independent_release_regression*`). C++ Release **6/6**; ASan/UBSan **6/6**, including null/NaN/invalid-dt shared C ABI tests. Python/C++ parity max error 1.42e-14; 10,000 ctypes Stanley calls have exact parity and explicit boundary checks. Wheel built and imports verified from a non-source directory in an isolated system-site-packages venv; package discovery previously omitted planning/control.

Real CARLA client/server 0.9.15, Town10HD_Opt, synchronous .05 s. Initial PID regression: 7 scenarios × 2 seeds, 12 s/run. Online v4 following/static moved 53.97/61.61 m, CTE .000127/.0802 m, zero collisions. Physical acceleration constraints did not pass that checkpoint. Actual step tests diagnosed automatic clutch transients; manual first gear removed the large spike. No-motion and native Ackermann collision failures are preserved.

Final raw seven-scene × two-seed regression has **14/14 motion and ideal-estimator gates PASS, zero collisions, but 14/14 overall PARTIAL**. Normal PID runs have acceleration approximately 3.10–3.70 m/s² and jerk 87–106 m/s³; timeout emergency stops reach 22.82–26.88 m/s² deceleration and jerk 316–508 m/s³. These are unclipped physical measurements, not solver bounds.

First in-process C++ full-chain static seed 7 failed during tiny negative-speed contact rollback. That retained failure is distinct from the corrected `carla_cpp_recovery_verified` rerun: true reverse below −.1 m/s is rejected, while slight standstill rollback is projected to zero only for forward planner initialization. Following/static seed 7 then moved 54.175/62.096 m, CTE .000088/.096685 m, no collisions or solver rejection, and passed the strict +3/−6 acceleration envelope. Jerk 87.021/78.646 remains PARTIAL. Planner P95 327/87 ms misses a 50 ms cycle; the whole executive remains Python.

Native ROS Humble chain passed acceleration then brake-to-zero, topics, services, TF and rosbag. Final ROS checkpoint: 802 samples, .641/.950/1.350 ms latency and .348 ms timer-jitter standard deviation. Project-local CARLA bridge is not an official bridge installation claim.

## Delivered materials and open gates

Subsequent four-run `carla_sensor_v2` evidence improves the v4 boundary: actual acceleration 2.885–2.921 m/s², zero collisions, GNSS/IMU position RMSE .073–.104 m and 292–307 valid GNSS updates. Initial estimator divergence in 3/4 prior runs is retained; rejecting impossible first-spawn IMU derivative packets fixes it in this rerun. Measured jerk 78–87 m/s³ remains PARTIAL; ideal-sensor diagnostics do not close noise/dropout or sensor-feedback gates. An optional in-process C++ C ABI / ctypes controller is implemented; 10,000 direct-call parity cases pass, but scalar FFI overhead shows no speedup. The earlier binding checkpoint had no native candidate evaluation; the accepted planning increment below now closes W08. Full-stack deadlines remain PARTIAL; DP/QP/MPC stays Python.

[README](../README.md), [benchmark](SIMULATION_BENCHMARK.md), [hardware guide](HARDWARE_READINESS.md), [interview guide](INTERVIEW_TECHNICAL_GUIDE.md), [commissioning tools](../tools/hardware/README.md), real camera screenshots/GIFs and versioned raw traces. Reproduction commands are in README and ROS scripts.

Open P0 gates: full CARLA physical constraints and planning/control deadlines, sensor-noise/dropout and sensor-feedback acceptance. P1 must generate actual model weights and seeded training/evaluation results. Physical hardware, true E-stop and target Pi timing remain PENDING_HARDWARE. Zero collisions, a mock pass or an interface checklist does not imply final acceptance. First physical week is detailed in HARDWARE_READINESS.

## Independent release and cloud checkpoints

Independent host regression: 526 passed in 6.75 s; wall 7.125 s, kernel child CPU 3.848 s, sampled tree RSS max 108,056,576 bytes (`artifacts/independent_release_regression*`). [Cloud software matrix](https://github.com/Leo-7GOAT/FalconPlan/actions/runs/38146268266) and [ROS2 acceptance](https://github.com/Leo-7GOAT/FalconPlan/actions/runs/38146268280) passed at `793a3ed24819ecf4b22d8548ade9d4c0801923b9`. Later source revisions have their own workflows; a prior pass is not a badge for the current revision.

## Accepted native planning increment

The native planning increment at `e1718ac354949e80a18cc69e91800ed64c7017d0` closes the originally requested planning-core migration category: actual candidate cost, continuous segment-circle collision and stable best-candidate selection run inside OnlinePlanningStack. Independent whole-library regression with that native library: **535 passed in 5.26 s**; profiler wall 9.604 s, kernel child CPU 5.130 s, sampled tree RSS max 108,720,128 bytes (`artifacts/final_native_release_regression*`). Windows: 524 passed, 11 explicitly unavailable native/Linux skips. **9 Release + 9 ASan/UBSan GoogleTests** pass. For 2,000 candidates the maximum cost difference is 4.55e-13, with zero collision/selection mismatches. Eight-candidate batch P50 Python/native is 203/126 microseconds, including native packing/FFI/result creation; this is not full-stack timing.

Actual CARLA static seed 7: 400 ticks carry native-plan metadata (not 400 new evaluations), displacement 61.235 m, CTE 0.09825 m, no collision/planner failure, acceleration +2.991/-1.990 m/s², nominal estimator RMSE 0.07278 m. Jerk 78.646 m/s³ keeps the scene **PARTIAL**. See `artifacts/carla_cpp_planning/planning_integration.json` for four matching source hashes, reproduction and the overlapping-test latency caveat.

## Actual isolated software release

Release A (`4ecb4e358735fb6e07dea66e7cc8015fb0e9076b`) was exported from committed sources into a separate Linux directory, installed in a fully isolated venv, passed dependency checks, **9 C++ GoogleTests and 535 Python tests (2.53 s)**, then selected with hardware disabled and no process launched. Raw build/test/install logs and resolved dependencies are in `artifacts/deployment/`. Locally cached Linux wheels were verified against official PyPI SHA256 metadata before installation; the manifest is retained. This is developer-host software staging, not a Pi measurement or running-actuator process rollback. The task-owned CARLA server was stopped only after actors were removed and the world restored to asynchronous operation.

The second independent release B (`98f07c3ce758b13fce2d9524e7064c642023ec19`) passed 9 C++ tests and 535 Python tests (2.17 s). Actual selection went A → B → rollback A; both release directories remain present, with hardware disabled and no process started. [Software CI](https://github.com/Leo-7GOAT/FalconPlan/actions/runs/38148371824) and [ROS2 CI](https://github.com/Leo-7GOAT/FalconPlan/actions/runs/38148371816) actually passed for B. The eventual publication revision may contain later documentation-only evidence; obtain its exact SHA using `git rev-parse HEAD` and the final delivery message.
