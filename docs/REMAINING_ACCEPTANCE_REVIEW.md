# Original-scope acceptance review — 2026-10-11

This is a read-only source/evidence audit against the user's original pasted brief, not a replacement scope. Only this review file was written. The current software stage is reviewable, but it does **not** satisfy the original final stopping condition: outstanding software items remain alongside PENDING_HARDWARE.

A PASS accepts only the stated experiment/component and its stated envelope. PASS_UNIT, PASS_MOCK and PASS_REAL_SIM are not interchangeable. PARTIAL means implementation or evidence is incomplete; it is not an environmental BLOCKED. No current missing simulator/component is classified BLOCKED: both native simulator and ROS environments actually ran. Physical vehicle/robot, manufacturer protocol validation, Raspberry Pi HIL and physical E-stop are PENDING_HARDWARE.

Counts below use named acceptance checks as the denominator, splitting compound requirements where their statuses differ. They are not percentages of engineering effort. Repeated requirements in different original sections are intentionally retained. The 16 week rows and the final §12 gates are separate denominators, not added to the detailed ledger. Historical W01–W05 assignments that the brief does not define precisely are inferred from repository history; an inferred mapping is not a new user requirement. Optional Falcon IPPO/MAPPO is not in the mandatory denominator and remains unimplemented.

## Material findings and remaining software

1. Actual CARLA and ROS were used. The seven-scene/two-seed PID smoke suite and the following/static full planning chain are different experiments. Do not imply that every full-stack behavior has been exercised across all seven scenes, or that the full planning chain ran through ROS in the CARLA smoke test. The ROS acceptance independently executes stamped command → safety → CARLA and brake-to-zero.
2. Current sensor evidence supersedes the earlier divergent diagnostic. [carla_sensor_v2](../artifacts/carla_sensor_v2/regression.json) records four 20 s runs with GNSS/IMU position RMSE 0.072594, 0.075493, 0.103840 and 0.084998 m; 292–307 accepted GNSS updates and two rejected initial IMU packets in one run. Earlier [full_stack_final](../artifacts/carla_full_stack_final/regression.json) contained three divergent diagnostics and remains a historical failure. Current ideal-sensor diagnostics pass. Control still uses actor ground truth, so sensor-feedback control, noise, dropout and timestamp-disorder robustness remain PARTIAL.
3. Raw actual acceleration is now measured without clipping, and manual gear fixes clutch transients. Four sensor-v2 runs stay within the tested nominal acceleration envelope (max 2.885–2.921 m/s²), but measured jerk is 78.646–87.021 m/s³ and explicitly PARTIAL. Old clipped 3/6 m/s² summaries are not physical-limit proof. [C++ CARLA runs](../artifacts/carla_cpp_recovery_verified/regression.json) mark motion/acceleration PASS_REAL_SIM and overall physical constraints PARTIAL.
4. W08 now satisfies all four original migration categories. In addition to controllers, domain data and kinematic RK4/constant-jerk kernels, the new C++ module evaluates the actual lattice candidate costs, checks collision continuously along each sampled line segment and selects the first minimum-cost collision-free candidate. Its typed C++ API and bounded C ABI reject invalid numeric/count/offset input; no exception crosses the boundary, all-blocked returns no solution, and invalid planning input leaves output arrays unchanged. The online planner calls this module in process and uses its selected trajectory. Polynomial sampling, feasibility filters, extended-route collision checks, ST-DP, QP and MPC remain Python; the original requirement does not demand migrating those entire modules. [Parity/benchmark](../artifacts/cpp_planning/parity_benchmark.json) covers 2,000 candidates in 250 eight-candidate batches, with zero collision/selection mismatches and maximum absolute cost error 4.547e-13. Batch P50 Python/native is 203.016/126.3705 µs, including packing, ctypes, native allocations and output construction. The earlier scalar Stanley FFI benchmark remains separate and shows no speedup.
   [Actual CARLA integration](../artifacts/carla_cpp_planning/planning_integration.json) and raw state logs contain 400 physics ticks with C++ evaluator plan metadata, zero planner failures, 61.234693 m displacement, 0.098253 m CTE RMSE and zero collisions. Replanning occurs at the configured planning cadence; 400 logged ticks do not imply 400 new C++ ranking calls. All four recorded source SHA-256 hashes match the audited files. This is PASS_REAL_SIM for the planning-evaluator component and W08; the scene remains PARTIAL because jerk is 78.645672 m/s³. That run overlapped independent tests, so its host latency is not an isolated performance comparison.
5. Online planner/controller percentiles are reported, but the planner runs at 2 Hz and some P95/P99 calls exceed 50 ms. Synchronous simulation accepts slow wall-time computation; it does not prove a fixed 20 Hz planning budget. Explicit bounded scheduling, deadline fault behavior and representative real-time performance acceptance remain open.
6. [Analytical dynamics](../artifacts/dynamics/benchmark.json) retains a low-friction 20 m/s delayed run with tracking RMSE 38.760 m and final error about -111.543 m. This is an observed FAIL inside the model experiment, not a false simulator integration failure. Negative linearized eigenvalues across tested speeds do not establish nonlinear robustness. Current throttle/brake sweeps are actual CARLA identification inputs; fitted steering lag/deadzone/tire/mass parameters are not yet accepted.
7. PPO and Level-k/Stackelberg with actual training/evaluation artifacts remain mandatory P1 software gaps. Their priority does not remove them from the original scope. No optional multi-agent experiment is claimed.
8. Hardware commissioning tools, immutable release selection and rollback exist. Selection rollback does not itself stop a running actuator process. Joined safe-stop/deploy/rollback acceptance and a vehicle ros2_control mock hardware plugin remain incomplete. Unknown vendor frames are correctly left unspecified.

## Current evidence and claim boundaries

- Python latest audited checkpoint: [535 passed in 7.75 s](../artifacts/cpp_planning/full_python_tests.txt). Earlier 522/526-test host/profile results remain earlier checkpoints, not the final count.
- C++: [nine release GoogleTests](../artifacts/cpp_planning/release_ctest.txt) and [nine ASan/UBSan passes](../artifacts/cpp_planning/sanitizer_ctest.txt), including analytic cost, sparse-segment tangency, degenerate segment, stable tie, all-blocked, null/invalid input and unchanged-output failure cases. The seeded 1,000-case control probe, 10,000 direct Stanley-binding comparisons and new 2,000-candidate planning study are distinct case sets.
- Native ROS/CARLA acceptance: 802 latency samples, P50/P95/P99 0.640792/0.949817/1.350105 ms; actual timer standard deviation 0.347625 ms. Three seconds of positive acceleration moved 5.009897 m; four seconds of brake reached zero. [ROS report](ROS2_ACCEPTANCE.md) distinguishes host stamp latency from controller-period jitter and physical feedback.
- README now contains current architecture, actual figures, measured results, Chinese navigation and limitations. The original inherited 454-test/kinematic-only homepage is no longer the current finding. Final pushed SHA, sensitive-file scan and cloud CI result remain coordinator-owned publication evidence; local test success is not a cloud CI badge.

## Counts with explicit denominators

Detailed ledger: **127/149 checks have passing evidence**, 22 PARTIAL, 0 current FAIL, 0 BLOCKED. Passing evidence by kind: 69 PASS_UNIT, 58 PASS_REAL_SIM. Historic failed experiment rows remain in raw logs and the findings above; a corrected current check is not counted as a current FAIL. This W08 follow-up changes one detailed check from PARTIAL to PASS_REAL_SIM without reducing the denominator or changing unrelated statuses.

| Original section/check group | Denominator | PASS checks | PARTIAL checks |
| --- | --- | --- | --- |
| §4.1 Planning and live I/O | 17 | 17 | 0 |
| §4.2 Dynamics and actuation | 12 | 9 | 3 |
| §4.3 C++ migration and engineering | 11 | 11 | 0 |
| §4.4 CARLA genuine integration | 20 | 20 | 0 |
| §4.5 Safety | 11 | 11 | 0 |
| §4.6 P1 learning/game requirements (mandatory) | 6 | 0 | 6 |
| §6 stable architecture | 6 | 6 | 0 |
| §7 hardware-preparation software | 16 | 14 | 2 |
| §8 tests for every added core: six required classes | 6 | 4 | 2 |
| §8 vehicle benchmark metrics | 10 | 9 | 1 |
| §8 engineering metrics | 7 | 7 | 0 |
| §9 README / portfolio required outputs | 13 | 13 | 0 |
| §10–13 milestone/Git/final material | 10 | 4 | 6 |
| Additional W09 hard requirement: state estimation | 4 | 2 | 2 |

The requested week matrix has **9/16 PASS and 7/16 PARTIAL** under the stated week-level scope. This count is not a claim that 9/16 of the entire project is complete.

| Week | Current status | Acceptance boundary |
| --- | --- | --- |
| W01 | PASS_UNIT | Inherited vehicle/frame/control foundations; exact historical week assignment inferred |
| W02 | PASS_UNIT | Inherited prediction/risk/HAL foundations; exact historical assignment inferred |
| W03 | PASS_UNIT | Inherited behavior/FSM/IDM/MOBIL; historical assignment inferred |
| W04 | PASS_UNIT | Inherited lateral/path/controller demos; historical assignment inferred |
| W05 | PASS_UNIT | Inherited planning/integration tests; historical assignment inferred |
| W06 | PASS_UNIT | ST-DP existing demo/tests |
| W07 | PASS_REAL_SIM | QP/MPC demos and two real full-stack scenes; GT localization |
| W08 | PASS_REAL_SIM | All four requested categories: controllers, data, model and actual candidate cost/collision/stable selection; nine release/sanitizer tests, 2,000-candidate parity and native CARLA evaluator execution. DP/QP/MPC may remain Python. |
| W09 | PARTIAL | Actual CARLA and4/4 nominal GNSS/IMU diagnostics PASS; GT control, noise/dropout and identification incomplete |
| W10 | PASS_REAL_SIM | Software supervisor/faults/ROS plus physical brake-to-zero; hardware estop pending |
| W11 | PARTIAL | No accepted Falcon PPO experiment at snapshot |
| W12 | PARTIAL | No accepted Level-k/Stackelberg experiment |
| W16 | PARTIAL | Actuator/dynamic models exist; unsafe low-friction envelope and incomplete identification |
| W17 | PARTIAL | Minimal RL/game experiment not accepted |
| W18 | PARTIAL | Minimal RL/game OOD not accepted |
| W19 | PARTIAL | Replay/profiling/local checks PASS; remote CI/portfolio/final delivery not finished |

## Original §12 minimum visible gates

These are exactly the original 10 project gates. A small set of visible gates can pass while detailed §3–§9 software remains open. Count: **9/10 PASS**, 1 PARTIAL.

| Original gate | Status | Evidence/boundary |
| --- | --- | --- |
| C++ compiles/unit tests | PASS_UNIT | Nine release and nine sanitized tests; W08 categories now accepted |
| W07 DP-QP/MPC reproducible | PASS_UNIT | Originalanalyticaldemosandnumericchecks; plantjerkpartial |
| CARLA actual run | PASS_REAL_SIM | 0.9.15actualserver/physics |
| ROS2/HAL chain | PASS_REAL_SIM | NativeHumbleSafety/CARLA |
| Multiple real closed-loop scenes | PASS_REAL_SIM | 7scene2seedPID; twofullstackscenesdistinct |
| Safety/fault injection | PASS_REAL_SIM | Softwareunit/ROS/CARLAfaults |
| Dynamics/actuator error experiments | PASS_UNIT | ActualanalyticalcomparisonandCARLAsweeps; includesfailedOODrow |
| Benchmark/logs | PASS_REAL_SIM | Versionedraw/currentreports |
| Complete README | PASS_UNIT | Rewrittenhomepage; finalrootpublicationcheckseparate |
| Hardware Adapter Ready | PARTIAL | Stableabstractcontract/tools; vehiclemockros2control andjoinedstop/rollbackmissing; physicalpending |

## Detailed acceptance ledger

Each row is the stated check, status and actual evidence/boundary. Passing a unit check does not upgrade it to physical or target-platform validation.

### §4.1 Planning and live I/O

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| CV/CA prediction | PASS_UNIT | Inherited behavior prediction tests |
| TTC/THW risk | PASS_UNIT | Inherited risk/FSM tests |
| FSM | PASS_UNIT | Behavior planner and online stack |
| IDM | PASS_REAL_SIM | Actual following actor state |
| MOBIL | PASS_UNIT | Inherited behavior tests; no broad real lane-change claim |
| Frenet lattice | PASS_REAL_SIM | online_stack actual static-obstacle loop |
| Quintic/Quartic polynomial | PASS_UNIT | Inherited numerical tests |
| ST graph | PASS_REAL_SIM | Live actor ST boundaries |
| Dynamic obstacle projection | PASS_REAL_SIM | nearby_actors plus timestamp validation |
| DP speed planner | PASS_REAL_SIM | online_stack DP search |
| Convex ST corridor | PASS_REAL_SIM | Online DP-derived corridor |
| QP speed optimization | PASS_REAL_SIM | OSQP and dense inter-knot ST safety check |
| Longitudinal MPC | PASS_REAL_SIM | Full-stack runs, solve every fourth 50 ms tick |
| Stanley/LQR/PID | PASS_UNIT | All unit/demo coverage; real CARLA uses Stanley/MPC or PID |
| HAL/Watchdog | PASS_REAL_SIM | Shared HAL, CARLA and ROS fault evidence |
| Algorithm I/O closed loop | PASS_REAL_SIM | Two full-stack scenes × two seeds; not all planner behaviors on ROS |
| Dynamic states from simulator / truth clearly labeled | PASS_REAL_SIM | Actual moving actors, GT identification |

### §4.2 Dynamics and actuation

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Dynamic bicycle | PASS_UNIT | Friction-limited single-track model, RK4 |
| Steering actuator delay | PASS_REAL_SIM | 150 ms real-sim delay scenario; analytical sweep |
| Steering angle/rate limits | PASS_UNIT | Unit and ROS clamps; estop deliberately bypasses rate limit |
| Acceleration/braking actuator model | PASS_REAL_SIM | First-order model and genuine CARLA sweep |
| First-order dynamics | PASS_UNIT | Analytical response test |
| Deadzone and saturation | PASS_UNIT | Config/tests; not target hardware calibration |
| Model parameter identification interface | PARTIAL | Step analysis/sweeps exist; no fitted mass/inertia/tire parameter model |
| Stability at different speeds | PARTIAL | Eigenvalue sweep exists; nonlinear low-friction 20 m/s run FAIL |
| Model error/disturbance experiments | PARTIAL | Friction/delay tests exist; broad mass/tire/inertia/crosswind envelope not accepted |
| Kinematic/dynamic comparison | PASS_UNIT | Both measured in analytical benchmark |
| Configurable delay/deadzone/friction/model parameters | PASS_UNIT | Dataclasses expose those parameters |
| Before/after benchmark | PASS_UNIT | Tuned gain .6 vs 1.5 at 10 m/s; preserve unsafe row |

### §4.3 C++ migration and engineering

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Controller migration | PASS_UNIT | PID and Stanley library functions |
| Trajectory/state data structures | PASS_UNIT | State/Command/PathPoint/Longitudinal |
| Kinematics/dynamics core | PASS_UNIT | RK4 kinematic bicycle and constant jerk |
| Key planning search / trajectory evaluation core | PASS_REAL_SIM | Native lattice candidate costs, continuous segment-circle collision and stable argmin drive actual online trajectory selection; 2,000-candidate parity and 400-tick CARLA integration evidence |
| CMake | PASS_UNIT | Actual compiled static library and probe |
| GoogleTest | PASS_UNIT | Nine cases in core_tests; release_ctest.txt records 9/9 |
| Compile warnings | PASS_UNIT | -Wall -Wextra -Wpedantic -Werror |
| ASan/UBSan | PASS_UNIT | Local Linux sanitizer_ctest.txt records 9/9 |
| Python/C++ numeric parity | PASS_UNIT | Existing 1,000 control cases plus 2,000 planning candidates, seed42; planning max cost error 4.547e-13, zero collision/selection mismatches |
| Clear library interface | PASS_UNIT | core.hpp backend-neutral types/functions |
| Comparable in-process performance | PASS_UNIT | 250 matched eight-candidate batches: Python/native P50 203.016/126.3705 µs including FFI/packing; earlier 10,000 scalar FFI comparisons remain separately scoped |

### §4.4 CARLA genuine integration

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Version/API/map/model/resource/bridge inspection | PASS_REAL_SIM | 0.9.15 server/API; custom bridge; hardware/resource limits separately scoped |
| Actual server/client connection | PASS_REAL_SIM | Local Windows server reached from WSL |
| Spawn/destruction | PASS_REAL_SIM | Owned actors tracked and cleaned |
| Map/reference route | PASS_REAL_SIM | Town10HD_Opt waypoints |
| Ego state acquisition | PASS_REAL_SIM | Rear-axle actor ground truth |
| Dynamic surrounding actors | PASS_REAL_SIM | Following vehicle is real actor |
| Control send | PASS_REAL_SIM | VehicleControl applied |
| Synchronous time management | PASS_REAL_SIM | Fixed50ms tick; restored world settings |
| Physics simulation | PASS_REAL_SIM | Actual CARLA server |
| Trajectory/state logs | PASS_REAL_SIM | JSONL; original and final runs retained |
| Automated scenario regressions | PASS_REAL_SIM | Seven scenes×two seeds smoke; two scenes×two seeds full-stack |
| Straight speed tracking scene | PASS_REAL_SIM | Original real smoke runs |
| Curve tracking scene | PASS_REAL_SIM | Original real smoke runs |
| Static obstacle scene | PASS_REAL_SIM | Final full-stack route changes |
| Slow lead/dynamic obstacle scene | PASS_REAL_SIM | Final full-stack live ST/IDM |
| Actuator delay/model-error scene | PASS_REAL_SIM | Real delay scene plus separately labeled analytical perturbation |
| Sensor/state/control-timeout scene | PASS_REAL_SIM | Real timeout intervention; ideal-sensor estimator corrected in carla_sensor_v2; robustness separate |
| At least five scenes and multiple fixed seeds | PASS_REAL_SIM | Seven smoke scenes; seeds7/19, not five seeds |
| Logs/screenshots/video/config | PASS_REAL_SIM | Actual camera GIF/PNG and JSON config |
| Ground truth/perception boundary | PASS_REAL_SIM | Truth is identified; no camera perception claim |

### §4.5 Safety

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Safety Supervisor | PASS_REAL_SIM | Shared core plus real ROS/CARLA output |
| Command timeout | PASS_REAL_SIM | 200ms ROS policy and real-sim scene |
| State timestamp expiry | PASS_REAL_SIM | ROS and simulation fault tests |
| Speed limit | PASS_UNIT | Overspeed unit fault latch |
| Steering/acceleration constraints | PASS_UNIT | Command constraints; not guaranteed physical jerk |
| Geofence | PASS_UNIT | Unit fault and ROS-configurable fence |
| Watchdog | PASS_REAL_SIM | Independent HAL/ROS command age |
| Software emergency stop | PASS_REAL_SIM | Latched brake; not physical estop |
| Manual override | PASS_REAL_SIM | Fresh-command handshake and same limits |
| Fault injection | PASS_REAL_SIM | Actual timeout, ROS malformed/stale inputs |
| Safety FSM/recovery | PASS_REAL_SIM | Explicit fresh stationary reset; unit/ROS checks |

### §4.6 P1 learning/game requirements (mandatory)

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| PPO vs FSM/IDM/MOBIL baseline | PARTIAL | No completed Falcon training/evaluation artifact at this snapshot |
| Level-k or Stackelberg baseline | PARTIAL | No accepted implementation/experiment found |
| OOD/domain randomization | PARTIAL | Analytical perturbation exists; requested RL/game OOD experiment absent |
| Actual training process | PARTIAL | Not accepted yet |
| Reward curve and weights | PARTIAL | Not accepted yet |
| Seeds and evaluation results | PARTIAL | Not accepted yet |

### §6 stable architecture

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Algorithm layer independent of hardware brand | PASS_UNIT | Domain types and HAL |
| Simulator/HardwareAdapter shared contract | PASS_UNIT | VehicleHAL interface |
| Configuration switching | PASS_UNIT | ROS YAML, backend flags and commissioning config |
| Time/units/frames/errors | PASS_UNIT | Explicit SI and monotonic/ROS envelope conversion |
| Mock/replay/real separation | PASS_UNIT | Tools provenance and evidence labels |
| Ackermann reserve without invented R550 protocol | PASS_UNIT | Acceleration-mode Ackermann transport; vendor fields null |

### §7 hardware-preparation software

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Abstract vehicle hardware adapter | PASS_UNIT | VehicleHAL |
| Mock I/O | PASS_UNIT | MockHardwareAdapter and fixture |
| ROS2/ros2_control Mock Hardware | PARTIAL | Genuine ROS mock-input Ackermann transport; no vehicle ros2_control hardware plugin |
| Command/state interfaces | PASS_UNIT | Stable domain and ROS contracts |
| Config files | PASS_UNIT | Safety/commissioning configuration |
| Calibration scripts | PASS_UNIT | Commissioning tool; real CARLA sweep is not hardware fit |
| Steering/motor step tool | PASS_UNIT | Bounded offline plans; real throttle/brake sweep |
| Encoder/IMU recording tool | PASS_UNIT | Recorder event types/units, actual sensors in CARLA |
| Latency measurement tool | PASS_UNIT | Stamped ROS data and command receipts |
| Log replay | PASS_REAL_SIM | Real rosbag replay plus validated offline JSONL |
| Fault injection tool | PASS_UNIT | Separate provenance and real ROS faults |
| Wiring/comms checklist | PASS_UNIT | tools/hardware/README first-week checklist |
| Deployment script | PASS_UNIT | Stage/select immutable release |
| Rollback and safe-stop mechanism | PARTIAL | Selection rollback does not stop running actuator process; runtime brake logic exists but deployment-stop join not accepted |
| Truthful hardware readiness label | PASS_UNIT | No real-vehicle validation claim |
| Target HIL correctly pending | PASS_UNIT | No target Pi timing attribution |

### §8 tests for every added core: six required classes

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Unit tests | PASS_UNIT | Core and ROS tests exist |
| Numerical tests | PASS_UNIT | MPC/QP/parity/dynamics |
| Integration tests | PASS_UNIT | Real ROS/CARLA and full-stack loop |
| Boundary cases | PARTIAL | Coverage exists but not exhaustive across new sensor/online-stack modules |
| Failure paths | PARTIAL | Initial IMU rejection and divergence corrected; noisy/dropout estimator acceptance not exercised |
| Performance tests | PASS_UNIT | Direct in-process C++ comparison and planner/control timing exist; runtime deadlines remain incomplete |

### §8 vehicle benchmark metrics

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| CTE RMSE | PASS_REAL_SIM | CARLA summaries |
| Heading RMSE | PASS_REAL_SIM | CARLA summaries |
| Speed RMSE | PASS_REAL_SIM | CARLA summaries |
| Max steering | PASS_REAL_SIM | CARLA summaries |
| Steering rate | PASS_REAL_SIM | CARLA summaries with estop caveat |
| Acceleration/jerk | PARTIAL | Final full-stack values unclipped; old smoke values clipped; physical jerk not constrained |
| Collision count | PASS_REAL_SIM | Actual collision sensor |
| Planner runtime | PASS_REAL_SIM | Measured timestamps |
| Controller runtime | PASS_REAL_SIM | Measured timestamps |
| Safety intervention count | PASS_REAL_SIM | Actual supervisor counts |

### §8 engineering metrics

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| P50/P95/P99 | PASS_REAL_SIM | Measured ROS/control/runtime |
| CPU/memory | PASS_UNIT | Host process profiler/time-v; not Pi/server total |
| Actual control-cycle jitter | PASS_REAL_SIM | ROS monotonic timer periods |
| ROS message latency | PASS_REAL_SIM | Same-host stamp-to-callback only |
| Test success rate | PASS_UNIT | artifacts/cpp_planning/full_python_tests.txt: 535 passed in 7.75 s; earlier 522/526 checkpoints retained |
| Fault recovery result | PASS_REAL_SIM | Explicit ROS and simulation tests |
| Hardware/software/seeds/sample/statistical metadata | PASS_UNIT | Latest reports distinguish experiment checkpoints and protocols |

### §9 README / portfolio required outputs

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| English homepage and Chinese navigation | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Problem/architecture/stack/algorithms in first30seconds | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Actual simulator and truthful measured data | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Architecture diagram | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Actual simulation screenshot | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Trajectory/performance plots | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Actual GIF/video link where generated | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Environment setup | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| One-command quick start/tests | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Benchmark and failure cases | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Limitations/software-vs-hardware boundary | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Hardware integration guide | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |
| Engineering lessons and checked paths/commands | PASS_UNIT | Current rewritten README + linked evidence/guides; root final checked-command/publication acceptance separately pending |

### §10–13 milestone/Git/final material

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| M1 baseline/core-loop evidence | PASS_UNIT | Baseline audit and W07 logs |
| M2 simulation/safety evidence | PARTIAL | Actual GT-control/ROS/safety passes; physical jerk and runtime full acceptance incomplete |
| M3 hardware-pre-software enhancement evidence | PARTIAL | Identification/robustness/P1 gaps remain |
| M4 portfolio completion | PARTIAL | Required pages now written; final remote CI/push evidence still coordinator-owned |
| Independent protected Git/reconciliation | PASS_UNIT | Parent restored/backs up, no destructive reset |
| Tested incremental commits and non-force push | PARTIAL | First P0 push being performed; final remote SHA/CI pending |
| Sensitive/oversize/cache check | PARTIAL | Parent final publication scan owns acceptance |
| docs/FINAL_DELIVERY_REPORT.md | PASS_UNIT | Present with checkpoint SHA, evidence and partial statuses |
| docs/INTERVIEW_TECHNICAL_GUIDE.md | PASS_UNIT | Present; parent owns final publication review |
| Latest commit/GitHub/status matrix/evidence/boundaries | PARTIAL | Stage report lists snapshot commit; final pushed SHA/CI pending |

### Additional W09 hard requirement: state estimation

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| GNSS/IMU estimation actually acquired | PASS_REAL_SIM | Real sensor callbacks logged |
| Ideal-sensor estimator accuracy | PASS_REAL_SIM | carla_sensor_v2:4/4 nominal diagnostics .072594–.103840m; older3/4 divergence preserved |
| Estimated-state feedback and sensor robustness | PARTIAL | GT controls plant; added noise/dropout/estimated-feedback experiment absent |
| Calibrated actual simulator actuator model | PARTIAL | Five throttle/brake sweeps exist; fitted steering/lag/deadzone not complete |

## Closure rule

Do not mark the original task complete while any mandatory software PARTIAL above remains. Tonight's verified stage may be published with these boundaries and preserved evidence. Resolve software failures and missing experiments, update exact counts against the same original requirements, then record final remote commit/CI evidence. Hardware work stays PENDING_HARDWARE until performed on the real target.
