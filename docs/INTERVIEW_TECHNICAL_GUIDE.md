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

`state_estimator.py` implements IMU propagation, GNSS updates, Mahalanobis gating and Joseph covariance form. `hal/carla_sensors.py` uses actual GNSS/IMU with an OpenDRIVE georeference Jacobian. Bias, gravity, extrinsics and clock alignment are physical deployment prerequisites. Current control uses truth localization.

`ros2/falcon_ros` carries Ackermann/Odometry, TF and safety services. Separate callback groups and MultiThreadedExecutor fixed command starvation caused by a synchronous simulator timer. `control/safety.py` latches stale state/command, geofence, speed, reverse, collision and software E-stop. Manual commands still obey limits; explicit stationary reset and fresh reevaluation are required. Explain clock domains, queue expiry and why a software brake command is not a hardware E-stop.

## C++ and Sim-to-Real

`cpp/include/falcon/core.hpp` exposes the numerical core and structures. Warnings-as-errors, GoogleTest, ASan/UBSan and 1,000 random parity cases substantiate numerical claims, not end-to-end deadlines. Python planning cost needs pruning, asynchronous execution or further C++ migration before target deadline acceptance. Sim-to-real requires measured actuator parameters, verified SI/frames, actual Pi CPU/jitter/transport evidence, a bounded low-speed ODD and independent stopping mechanisms. Personally understand coordinate transforms, constant-jerk equations, solver feasibility, actuator delay, safety recovery and the scope of each status.
