# PPO control experiment: measured results and remaining acceptance

**Training and evaluation process: PASS_UNIT. Overall policy robustness: PARTIAL.**
Two independently trained policies pass all 15 nominal held-out episodes each.
Both fail the unknown acceleration-gain and combined perturbation cases. These
are engineering simulations using the repository's analytical dynamic bicycle
model, not CARLA, sensor perception, Raspberry Pi, or physical vehicle evidence.
No learned policy has been validated on the CARLA adapter or authorized for
hardware deployment.

## Execution and model contract

The learned policy chooses acceleration and an absolute target lane. A fixed
quintic lane transition and Stanley controller produce steering. Every policy
then uses the same nominal command bounds, actuator model, SafetySupervisor,
and analytical DynamicBicycleModel. Traffic consists of target-level truth,
scripted braking leaders, and an IDM follower. Swept vehicle rectangle tests
check collisions. No perception model is substituted for measured sensors.

The classical comparison actually calls the existing BehaviorPlanner FSM,
IDM acceleration, and MOBIL lane evaluation. The guarded learned policy also
uses a predictive IDM acceleration cap and front/rear lane gap checks. The
ablation removes these predictive guards while retaining the supervisor,
actuator limits, and lane cooldown. It is not an unrestricted physical policy.
Raw requested actions and guard interventions remain in the traces.

The shared actuator envelope is explicitly **plant-aware**. Its zero-speed
braking filter bounds discrete braking release using known nominal actuator
gain and writes the filtered acceleration back into actuator state. Envelope
v2 adds an analogous speed headroom filter. Envelope v3 uses an internal
11.999 m/s control target to avoid floating-point overshoot of the unchanged
12 m/s hard supervisor limit. These filters are shared by every policy in a
paired comparison. They simplify analytical plant behavior and have not been
validated against CARLA or R550 hardware. Performance changes across v1/v2/v3
cannot be attributed solely to PPO learning: reward and the shared control
contract both changed. Unknown gain deliberately remains unknown to this
filter, exposing measured jerk failures in OOD evaluation.

## Actual training

The actor receives 17 normalized values: ego speed, lateral position, heading,
yaw rate, actuator acceleration/steering, desired speed/lane, both lanes' front
and rear gaps/relative speeds, and the previous acceleration command. Two
64-unit tanh layers feed a Gaussian acceleration head and a categorical lane
head; a separate critic estimates value. Acceleration is transformed with tanh
to [-6, 3] m/s². PPO uses clipped ratios (0.2), terminal-masked GAE (gamma 0.995,
lambda 0.95), four Adam epochs, minibatches of 256, and gradient clipping 0.5.
Action log probabilities include the tanh Jacobian; entropy regularization
uses base Gaussian plus categorical entropy as an explicit approximation.
The sampling/update structure follows the primary
[PPO paper](https://arxiv.org/abs/1707.06347).

Each rollout uses eight environments, 256 time steps, and dt=0.05 s.
Training randomizes desired speed [6,10] m/s, friction [0.8,1.0], actuator delay
[0,0.1] s, and mass scale [0.9,1.1]; acceleration gain remains 1.0.
NumPy and Torch are seeded. Torch runs on CPU with one thread and deterministic
algorithms, even though the installed build is 2.14.0+cu130. Training logs,
optimizer state, weights, reward curves, and SHA256 values are stored locally.

| Artifact directory | Seed | Reward / envelope | Actual transitions | Updates | Completed episodes | Loop time (s) | Actor parameter change L2 |
|---|---:|---|---:|---:|---:|---:|---:|
| ppo_seed7 | 7 | 1 / 1 | 131072 | 64 | 612 | 78.218 | 1.32649 |
| ppo_v2_seed7 | 7 | 2 / 2 | 262144 | 128 | 810 | 158.649 | 1.55145 |
| ppo_v2_seed19 | 19 | 2 / 2 | 262144 | 128 | 838 | 164.558 | 1.09685 |
| ppo_v3_seed7 | 7 | 2 / 3 | 262144 | 128 | 684 | 218.585 | 1.78889 |
| ppo_v3_seed19 | 19 | 2 / 3 | 262144 | 128 | 680 | 218.707 | 1.70648 |
| ci_smoke | 7 | 2 / 3 | 512 | 1 | 0 | 0.786 | 0.07415 |

Five substantive runs contain 1,179,648 actual transitions; the CI smoke adds
512. Actor-only changes above are recomputed against seeded initial weights,
so critic updates alone cannot satisfy the training proof. The manifest
[reproducibility_manifest.json](../artifacts/learning/reproducibility_manifest.json)
checks weight hashes and records final source hashes. Older artifact version
fields select retained reward/envelope branches; final source hashes are not
claimed to be historical source hashes.

The v3 policies were trained concurrently. Process profiles measured elapsed
times 224.355/224.665 s, kernel CPU totals 212.708/213.043 s, and sampled process
tree RSS maxima 899,252,224/961,404,928 bytes (seeds 7/19). These are x86 WSL host
measurements including Python/Torch overhead, not Pi memory or timing evidence.
Concurrent runs are not a controlled single-process performance comparison.

## Paired held-out evaluation

Evaluation seeds are 101, 211, 307, 401, 503, independent of training seeds.
Each of free driving, mixed traffic, and a braking leader is evaluated for
400 samples (20 simulated seconds). All policies receive identical initial
traffic and plant parameters for each seed. Two final reports each contain
225 episodes: five domains × three scenarios × five seeds × three policies.
Earlier v1 and v2 seed-7 reports each retain another 225 episodes, including
failures; CI adds three episodes. Total retained evaluations: 903 episodes.

Every strict episode gate requires a full horizon, displacement >3 m, zero
collision/offroad, CTE RMSE <1 m, steering <=0.5 rad, steering rate <=0.8 rad/s,
measured acceleration within [-6,3] m/s², and measured jerk <=6 m/s³
(numerical tolerance 1e-6). Acceleration and jerk are unbounded measured speed
differences, not command values clipped for reporting. Speed RMSE is reported
without inventing a new passing threshold.

| Nominal policy | Passed episodes | Collision episodes | Mean return | Mean CTE RMSE (m) | Mean speed RMSE (m/s) | Mean guard interventions |
|---|---:|---:|---:|---:|---:|---:|
| FSM / IDM / MOBIL | 14/15 | 1/15 | 104.572 | 0.0591 | 2.6379 | See raw groups |
| PPO guarded, train seed 7 | 15/15 | 0/15 | 115.465 | 0.0960 | 3.7713 | 50.67 |
| PPO guarded, train seed 19 | 15/15 | 0/15 | 105.093 | 0.0718 | 3.7015 | 49.60 |
| PPO without predictive guard, seed 7 | 15/15 | 0/15 | See raw groups | See raw groups | See raw groups | 0 predictive |
| PPO without predictive guard, seed 19 | 10/15 | 5/15 | See raw groups | See raw groups | See raw groups | 0 predictive |

The classical nominal episodes are identical across the two reports and must
not be counted as 30 independent baseline tests. Higher return does not establish
PPO superiority: speed accuracy is worse than the classical baseline, and guard
interventions are substantial. The small held-out set and seed-dependent
ablation failures do not establish a certified collision probability.

| Guarded PPO OOD case | Plant change | Seed 7 pass | Seed 19 pass | Offroad episodes (7 / 19) | Maximum measured jerk (7 / 19), m/s³ |
|---|---|---:|---:|---|---|
| Delay | delay 0.25 s | 13/15 | 15/15 | 2 / 0 | 6.000 / 6.000 |
| Friction | mu 0.4 | 15/15 | 15/15 | 0 / 0 | 5.999 / 6.000 |
| Gain and mass | gain 1.4, mass scale 1.3 | 0/15 | 0/15 | 0 / 0 | 11.573 / 11.573 |
| Combined | speed 12, mu 0.4, delay 0.25, mass 1.3, gain 1.3 | 0/15 | 0/15 | 14 / 9 | 9.725 / 9.953 |

All guarded OOD cases have zero measured collision episodes, but offroad and
jerk failures still make both complete reports **PARTIAL**. The delay row's
rounded jerk values must not replace unrounded JSON acceptance decisions.
See the complete [seed 7 report](../artifacts/learning/evaluation_v3_seed7/evaluation.json)
and [seed 19 report](../artifacts/learning/evaluation_v3_seed19/evaluation.json),
their comparison plots, and representative gzip JSONL traces. Every run retains
its summary; first evaluation seed retains detailed traces for every paired
domain/scenario/policy, rather than claiming all 903 full traces were stored.

## Preserved failures and interpretation

Reward v1's uncapped progress term produces an unconstrained speed optimum of
1.3125 times desired speed when considering its progress/speed-error terms.
Its guarded nominal result passed only 2/15 gates. Reward v2 caps progress at
one and adds a stopped-supervisor penalty, but its envelope-v2 run passed only
10/15 nominal gates: floating-point speed 12.000000000000004 triggered the
strict overspeed latch. The v3 internal headroom fixes this known numerical
boundary without relaxing the supervisor limit. Historical weights, reward
logs, evaluation failures, and version branches remain present.

Unknown gain causes physical jerk above the nominal bound despite bounded
software commands. Combined low friction, delay, gain, and higher target speed
cause frequent offroad events. The predictive guard reduces one learned seed's
collision failures, but does not repair all model uncertainty. Runtime timeout,
command timeout, and estop tests confirm learned actions cannot override the
latched supervisor. Physical actuator delay remains in stopped-mode simulation;
this is not a guarantee of instantaneous physical stopping.

## Reproduction and automated verification

From the project root, in Python 3.10+ with the optional Torch extra:

```bash
python -m pip install -e '.[dev,rl,profiling]'
python -m demos.learning.train_ppo --seed 7 --steps 262144 --output artifacts/learning/reproduce_seed7
python -m demos.learning.train_ppo --seed 19 --steps 262144 --output artifacts/learning/reproduce_seed19
python -m demos.learning.evaluate_ppo --weights artifacts/learning/reproduce_seed7/policy.pt --output artifacts/learning/reproduce_eval7
python -m pytest tests/test_learning_environment.py tests/test_learning_ppo.py -q
```

Full evaluation exits nonzero for a PARTIAL policy result; the JSON and plots
remain available. The API returns the report without hiding performance status:

```python
from demos.learning.train_ppo import train
from demos.learning.evaluate_ppo import evaluate
t = train('artifacts/ci-ppo', seed=7, total_steps=512)
e = evaluate('artifacts/ci-ppo/policy.pt', 'artifacts/ci-ppo/evaluation',
             seeds=(101,), cases=('ID',), scenarios=('free',))
assert t['trained_steps'] == 512 and t['parameter_delta_l2'] > 0
assert e['episodes'] == 3 and e['process_status'] == 'PASS_EVALUATION_PROCESS'
assert e['engine'] == 'ANALYTICAL_DYNAMIC_BICYCLE_NOT_CARLA'
# Retain e['status']; this process smoke does not gate the full OOD policy.
```

The equivalent evaluation CLI takes `--seeds 101 --cases ID --scenarios free
--process-only`; this flag only changes CLI exit policy and does not alter
performance status or failed gates in the saved report. Actual 512-step smoke
changed parameters (combined L2 0.153275; actor L2 0.074148), completed three
evaluation episodes, and passed its tiny free-driving case. It completed zero
training episodes, so it provides no meaningful episodic learning curve.
Default training and every substantive run have actual completed-episode plots.

Local module tests: **23 passed in 6.40 s**, including analytical GAE, policy
density/bounds, real actor gradient updates, save/load round trip, twice-seeded
bit-identical training weights, fault latches, collision failure, envelope
boundaries, and invalid/bool/nondivisible training parameters. Windows without
Torch explicitly skips optional PPO tests. CI must install CPU Torch to exercise
these tests; an optional-dependency skip is not an algorithm test pass.

Latest independent full regression checkpoint with native C++ planning parity
enabled: **578 passed in 13.91 s**, recorded in
[final_full_regression_profile.stdout.txt](../artifacts/learning/tests/final_full_regression_profile.stdout.txt).
The associated profiler measured 15.420 s process wall time, 9.633 s kernel
CPU, and 1,008,119,808 bytes sampled tree RSS. An earlier 562-pass/one-failure
checkpoint is preserved separately; the failing ROS timestamp test double was
subsequently repaired by the ROS owner. Repository-wide final publication checks
may run later after all owners freeze, so this is a dated checkpoint rather than
a claim about future CI.

## Unaccepted deployment work

Calibrated actuator gain and robust uncertainty handling, longer and broader
held-out tests, learned-policy CARLA integration, sensor-estimated observations,
and hardware inference/latency/stop tests remain unaccepted. This experiment
demonstrates actual training, weights, reward curves, numerical checks, classical
comparison, and OOD failure analysis. It does not close those deployment gates
or replace the classical P0 fallback.
