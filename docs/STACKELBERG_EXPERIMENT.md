# Stackelberg merge experiment — analytical planning model

Status: **PASS_UNIT** for the implemented game solver, contracts and executed numerical experiment; **PARTIAL** for CARLA/online-planner integration. This module does not command a vehicle or replace SafetySupervisor. Its quintic lateral trajectory is a planning assumption, not measured tire/steering physics.

The leader chooses constant longitudinal acceleration from {-4,-2,0,2} m/s² and whether to merge. For every leader strategy the follower maximizes its own utility over the same acceleration set. The leader then selects its best feasible outcome, using pessimistic leader utility for follower ties. Utilities include progress, desired-speed error, squared acceleration effort, leader merge reward and collision penalty. This is explicit nested best-response enumeration, rather than an if/else policy labelled as a game.

Nonreversing longitudinal motion integrates constant acceleration exactly until a full stop. Lateral merge follows a quintic over 2.5 s, after 0.5 s anticipation, on a 3.5 m lane. Collision uses an ellipse with 5 m longitudinal and 1.8 m lateral center separation. The relative-motion segment minimum catches between-knot crossings; it approximates the sampled curve, and does not certify continuous nonlinear motion. A robust veto rejects merge strategies that collide under any of the four bounded follower actions. This finite action set is not a guarantee against arbitrary time-varying road users.

Executed 2026-10-11 in WSL Python 3.10 on the development CPU. Seeds 7/17/27 each generate 20 scenes per domain; all policies see identical scenes. Each result records state, intent, actual opponent action, payoff matrix and a full time series. The CV baseline greedily optimizes the same leader utility while assuming constant-speed opponent. It is separate from the PPO experiment's FSM/IDM/MOBIL baseline.

| Domain (60 paired scenes each) | CV greedy merge / predicted collision | Stackelberg merge / predicted collision | Robust veto merge / predicted collision |
|---|---:|---:|---:|
| ID: opponent speed 8–14 m/s, desired 14 | 56 / 0 | 60 / 0 | 41 / 0 |
| Fast OOD: opponent speed 15–20, desired 19; leader still assumes 14 | 38 / 0 | 60 / 0 | 2 / 0 |
| Noncooperative OOD: fast opponent ignores leader intent, accelerates +2 | 16 / 22 | 23 / 37 | 2 / 0 |

These are model predictions and completed model merge trajectories, not CARLA collision measurements or a general success-rate guarantee. The observed **37/60 noncooperative collisions** invalidate use of the unguarded Stackelberg policy outside its response assumption. The veto removes those bounded-model collisions at the cost of completing only **2/60** fast-domain merges. No OOD safety or progress gate is relaxed to call this a globally accepted behavior policy. The earlier two-domain v1 remains in the repository; v2 adds the failed cooperation assumption instead of deleting it.

In v2 ID, decision-time P50/P95/P99 for Stackelberg is 2.723/3.409/3.880 ms; these single-host timings overlapped other engineering tasks. Exact domain statistics and every case are in [benchmark.json](../artifacts/game/merge_v2/benchmark.json); [raw trajectories](../artifacts/game/merge_v2/trajectories.jsonl) and [plot](../artifacts/game/merge_v2/benchmark.png) are retained.

```bash
# Run from repository root. Use a new output path: existing evidence is never overwritten.
python -m pytest -q tests/test_stackelberg.py
python -m demos.stackelberg_merge --output artifacts/game/my_run --seeds 7 17 27
```

Fourteen tests cover between-knot collision crossing, adjacent-lane separation, exact stopping without reversal, independent equilibrium enumeration, robust veto and malformed/NaN trajectories. Source: [solver](../behavior/stackelberg.py), [experiment](../demos/stackelberg_merge.py), [tests](../tests/test_stackelberg.py).

Remaining acceptance: receding-horizon integration through the actual planning/control/safety chain, CARLA two-actor negotiation, realistic reaction delays and actuator/lateral constraints, opponent-model identification, and time-varying adversarial behavior. The existing CARLA evidence elsewhere does not validate this new policy.
