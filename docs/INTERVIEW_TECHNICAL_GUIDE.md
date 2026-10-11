# Interview technical guide / 中文学习导航

## Frenet and lattice

`coordinate_system/reference_line.py` splines then reparameterizes by arc length. `frenet_transform.py`: x=xr-d sin θr, y=yr+d cos θr, d′=(1-κr d)tan(θ-θr). Near 1-κr d≤0 or ±90° heading, reject the mapping. Quintic lateral trajectories fix endpoint position/velocity/acceleration; quartic longitudinal trajectories fix terminal speed/acceleration with free terminal progress. `planning/lattice_planner.py` samples, filters dynamics/curvature, checks segment-circle collision and ranks costs. Understand sampling completeness, low-speed d-dot/s-dot singularity and circular-geometry limitations.

## ST-DP-QP and MPC

`st_graph.py` derives target-level occupancy from live states under CV/CA assumptions. `st_dp.py` stores predecessor/current distance pairs to constrain edge acceleration. DP selects the nonconvex obstacle topology; `speed_corridor.py` retains that free-space side. `speed_qp.py` minimizes tracking and jerk with exact segment equations v1=v0+(a0+a1)dt/2 and s1=s0+v0dt+(2a0+a1)dt²/6. Dense between-knot checks are sampled evidence, not formal continuous proofs.

`online_stack.py` connects live actors, behavior, lattice, ST-DP, corridor, QP and parameterization. A geometry speed floor avoids stationary spatial parameterization singularity; true measured speed remains in QP/MPC. `longitudinal_mpc.py` uses [s,v,a], jerk input and exact dynamics, applying the first control then resolving. Optimizer feasibility does not guarantee plant acceleration when an actuator mapping is wrong. Prepare to explain infeasibility, braking fallback, topology versus smoothing and receding-horizon delay.

## PID / Stanley / LQR / dynamics

PID uses conditional integration against windup. Stanley adds heading error and atan2(k*CTE,v+softening), tracking the front axle. LQR uses quadratic cost around a linearized model with speed/small-error assumptions. Separate curvature feedforward, feedback, command queue and physical actuator response.

`dynamic_bicycle.py`: m(vy_dot+v r)=Fyf cosδ+Fyr; Iz r_dot=lf Fyf cosδ-lr Fyr. Slip uses atan2 and friction/axle-load saturation. RK4 substeps resolve stiffness; low speed transitions to kinematics. Local stable eigenvalues do not guarantee delayed nonlinear closed-loop stability. `first_order_actuator.py`: queue delay, exact 1-exp(-dt/τ), deadzone, saturation and rate limits. The high-speed low-friction experiment failed; explain the operational-envelope implication honestly.

## Real failure cases, estimation, ROS and safety

CARLA wheel positions are world centimeters, requiring actor-frame projection and handedness conversion. Automatic gearbox clutch engagement caused 12–15 m/s² spikes in measured sweeps; manual first gear reduced them to about 2–3. Native Ackermann testing collided and is retained as FAIL. Distinguish intended acceleration, physically measured acceleration and simulator execution.

`state_estimator.py` implements IMU propagation, GNSS updates, Mahalanobis gating and Joseph covariance form. `hal/carla_sensors.py` uses actual GNSS/IMU with an OpenDRIVE georeference Jacobian. The complete planning-chain checkpoints use truth localization; a separate 12-run controller experiment initializes from real GNSS/IMU callbacks and drives control/servo/Safety without Ego truth reads. Consume each sensor timestamp once and preserve original accepted measurement age during extrapolation. Ten feedback cases pass; long-outage whole-window accuracy and all physical envelopes remain unaccepted. A stationary initialization prior avoids noise-driven false reverse without clipping later estimates. Bias, gravity, extrinsics and clock alignment remain physical deployment prerequisites.

`ros2/falcon_ros` carries Ackermann/Odometry, TF and safety services. Separate callback groups and MultiThreadedExecutor fixed command starvation caused by a synchronous simulator timer. `control/safety.py` latches stale state/command, geofence, speed, reverse, collision and software E-stop. Manual commands still obey limits; explicit stationary reset and fresh reevaluation are required. Explain clock domains, queue expiry and why a software brake command is not a hardware E-stop.

## Actual PPO and Stackelberg experiments

The [learning report](LEARNING_EXPERIMENT_REPORT.md) links actual weights, reward curves, seeded training and paired results. PPO uses a tanh-Gaussian acceleration head and categorical lane head, clipped likelihood ratios and terminal-masked GAE. The classical comparison executes FSM/IDM/MOBIL; learned inference uses trained tensor weights. Guard interventions are visible, and every paired policy uses the same actuator-envelope version. A separate unguarded ablation removes predictive collision/lane gates while retaining actuator/Safety contracts. Improvements across versions also reflect reward/envelope changes; do not attribute them solely to the network.

Two final seeds each train 262,144 transitions; each passes 15/15 ID model episodes but both fail gain/combined OOD. Nominal speed tracking is worse than the classical baseline. All this uses analytical dynamic bicycle motion, not CARLA. Explain why parameter changes in the actor, independent evaluation seeds and saved optimizer/weight/hash records distinguish real learning from a rule labelled PPO; none of them certifies road safety.

The [game report](STACKELBERG_EXPERIMENT.md) enumerates follower best responses for each leader action, then chooses the leader's best feasible strategy with pessimistic tie handling. Cooperation assumptions are measurable failure modes: a noncooperative opponent produces 37/60 unguarded predicted collisions; a bounded robust veto removes those model collisions but completes only 2/60 merges. Discuss opponent-model error and the safety/progress tradeoff, rather than claiming a universal game-policy advantage.

## C++ and Sim-to-Real

`cpp/include/falcon/core.hpp` exposes the numerical core and structures. Warnings-as-errors, GoogleTest, ASan/UBSan and 1,000 random parity cases substantiate numerical claims, not end-to-end deadlines. Python planning cost needs pruning, asynchronous execution or further C++ migration before target deadline acceptance. Sim-to-real requires measured actuator parameters, verified SI/frames, actual Pi CPU/jitter/transport evidence, a bounded low-speed ODD and independent stopping mechanisms. Personally understand coordinate transforms, constant-jerk equations, solver feasibility, actuator delay, safety recovery and the scope of each status.
