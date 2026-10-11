"""End-to-end W06 DP -> W07 QP -> receding-horizon longitudinal MPC."""
import bisect
import math

import matplotlib.pyplot as plt

from planning.longitudinal_mpc import (
    LongitudinalMPC, LongitudinalMPCConfig,
    LongitudinalState, MPCReferencePoint, propagate_constant_jerk,
)
from planning.speed_corridor import build_speed_corridor
from planning.speed_profile import speed_profile_from_dp_plan
from planning.speed_qp import (
    SpeedQPConstraints, SpeedQPWeights,
    build_speed_qp_problem, solve_speed_qp,
)
from planning.st_dp import DPCostWeights, DPSpeedPlanner
from planning.st_graph import DynamicObstacle, build_constant_velocity_st_boundary
from planning.st_grid import build_st_grid
from planning.st_transition import STKinematicLimits
from planning.speed_validation import is_speed_profile_st_safe
from planning.speed_profile import SpeedProfile, SpeedProfilePoint


def interpolate_qp_state(profile: SpeedProfile, t: float) -> LongitudinalState:
    """Evaluate the W07 piecewise constant-jerk trajectory at arbitrary time."""
    points = profile.points
    if not points[0].t - 1e-8 <= t <= points[-1].t + 1e-8:
        raise ValueError("query time is outside the QP profile")
    if t >= points[-1].t:
        p = points[-1]
        return LongitudinalState(p.s, p.v, p.a)
    k = max(0, bisect.bisect_right(profile.times, t) - 1)
    p, q = points[k], points[k+1]
    segment_dt = q.t - p.t
    jerk = (q.a - p.a) / segment_dt
    if abs(t - p.t) < 1e-12:
        return LongitudinalState(p.s, p.v, p.a)
    return propagate_constant_jerk(
        LongitudinalState(p.s, p.v, p.a), jerk, t-p.t
    )


def make_qp_reference():
    """Use the same moving front-car scenario as the W06/W07 demo."""
    boundary = build_constant_velocity_st_boundary(
        obstacle=DynamicObstacle(
            obstacle_id="slow_front_vehicle", s0=30.0, speed=5.0,
            length=4.0, safety_margin=2.0,
        ),
        horizon=6.2, dt=0.1,
    )
    grid = build_st_grid(horizon=6.0, dt=1.0, max_s=90.0, ds=5.0)
    dp = DPSpeedPlanner(
        grid=grid, boundaries=[boundary],
        limits=STKinematicLimits(
            min_speed=0.0, max_speed=25.0,
            max_accel=3.0, max_decel=6.0,
        ),
        initial_speed=15.0, desired_speed=15.0,
        weights=DPCostWeights(speed_error=1.0, acceleration=0.5),
        edge_sample_dt=0.02,
    ).plan()
    coarse = speed_profile_from_dp_plan(
        plan=dp, initial_speed=15.0, initial_acceleration=0.0,
    )
    corridor = build_speed_corridor(
        reference_profile=coarse, boundaries=[boundary],
        min_s=0.0, max_s=90.0, boundary_epsilon=0.20,
    )
    problem = build_speed_qp_problem(
        reference_profile=coarse, corridor=corridor,
        constraints=SpeedQPConstraints(
            min_speed=0.0, max_speed=25.0,
            max_accel=3.0, max_decel=6.0, max_jerk=4.0,
        ),
        weights=SpeedQPWeights(
            position_tracking=1.0, speed_tracking=1.0,
            acceleration=0.3, jerk=8.0,
        ),
    )
    qp = solve_speed_qp(problem=problem, reference_profile=coarse).profile
    if not is_speed_profile_st_safe(qp, [boundary], sample_dt=0.01):
        raise RuntimeError("QP reference is not ST-safe: abort MPC demonstration")
    return boundary, coarse, qp


def main():
    boundary, dp_profile, qp_profile = make_qp_reference()
    cfg = LongitudinalMPCConfig(
        dt=0.2, horizon_steps=6,
        max_speed=25.0, max_accel=3.0, max_decel=6.0, max_jerk=4.0,
        weight_s=0.8, weight_v=4.0, weight_a=0.2,
        weight_j=0.2, terminal_weight=3.0,
    )
    controller = LongitudinalMPC(cfg)
    state = LongitudinalState(
        qp_profile.points[0].s, qp_profile.points[0].v,
        qp_profile.points[0].a,
    )
    actual = [(0.0, state)]
    applied_jerks = []
    # Stop at t=4.8 so every MPC prediction remains within QP horizon t=6.
    num_steps = round((qp_profile.times[-1] - cfg.horizon_steps * cfg.dt) / cfg.dt)

    print("\nW07 longitudinal MPC: repeated optimization, execute first jerk only")
    print(f"{'t':>5} {'s':>10} {'v':>9} {'a':>9} {'j':>9}")

    for step in range(num_steps):
        t = step * cfg.dt
        ref = []
        corridor = []
        for k in range(cfg.horizon_steps + 1):
            future_t = round(t + k * cfg.dt, 10)
            r = interpolate_qp_state(qp_profile, future_t)
            ref.append(MPCReferencePoint(
                t=future_t, s=r.s, v=r.v, a=r.a,
            ))
            interval = boundary.boundary_at(future_t)
            # W06 selected the following topology: always behind front car.
            upper = 90.0 if interval is None else interval[0] - 0.05
            corridor.append((0.0, upper))

        result = controller.solve(state, ref, corridor)
        jerk = result.jerk_command
        next_state = propagate_constant_jerk(state, jerk, cfg.dt)
        executed = SpeedProfile(points=(
            SpeedProfilePoint(t=t, s=state.s, v=state.v, a=state.a),
            SpeedProfilePoint(t=t+cfg.dt, s=next_state.s, v=next_state.v, a=next_state.a),
        ))
        if not is_speed_profile_st_safe(executed, [boundary], sample_dt=0.005):
            raise RuntimeError(f"MPC executed segment collided at t={t:.2f}")
        print(f"{t:5.2f} {state.s:10.3f} {state.v:9.3f} {state.a:9.3f} {jerk:9.3f}")
        applied_jerks.append(jerk)
        state = next_state
        actual.append((t + cfg.dt, state))

    actual_t = [t for t,_ in actual]
    actual_s = [s.s for _,s in actual]
    actual_v = [s.v for _,s in actual]
    ref_at_actual = [interpolate_qp_state(qp_profile,t) for t in actual_t]
    pos_rmse = math.sqrt(sum((s-r.s)**2 for s,r in zip(actual_s,ref_at_actual))/len(actual_s))
    vel_rmse = math.sqrt(sum((v-r.v)**2 for v,r in zip(actual_v,ref_at_actual))/len(actual_v))
    print(f"\nPosition RMSE: {pos_rmse:.4f} m")
    print(f"Speed RMSE:    {vel_rmse:.4f} m/s")
    print(f"Max |jerk|:    {max(abs(v) for v in applied_jerks):.4f} m/s^3")
    print("Executed ST safety: PASS (dense sampled check)")
    assert pos_rmse<.25 and vel_rmse<.2, 'MPC tracking acceptance failed'
    assert max(abs(v) for v in applied_jerks)<=cfg.max_jerk+1e-5

    plt.figure(figsize=(10, 5))
    plt.fill_between(boundary.times, boundary.lower_s, boundary.upper_s,
                     alpha=.25, label="Dynamic front-car boundary")
    plt.plot(dp_profile.times, dp_profile.distances,"--o",label="W06 DP")
    plt.plot(qp_profile.times, qp_profile.distances,"-o",label="W07 QP")
    plt.plot(actual_t,actual_s,"-",lw=2,label="MPC executed s(t)")
    plt.xlabel("Time [s]");plt.ylabel("Progress [m]");plt.legend();plt.grid()
    plt.title("FalconPlan W07: DP -> QP -> MPC")

    plt.figure(figsize=(10,4))
    plt.plot(qp_profile.times, qp_profile.speeds,"--o",label="QP reference speed")
    plt.plot(actual_t,actual_v,"-",lw=2,label="MPC executed speed")
    plt.xlabel("Time [s]");plt.ylabel("Speed [m/s]");plt.legend();plt.grid()
    plt.title("Longitudinal MPC receding horizon speed tracking")

    plt.figure(figsize=(10,4))
    plt.step(actual_t[:-1],applied_jerks,where="post",label="Executed jerk")
    plt.axhline(cfg.max_jerk,ls=":",label="Max jerk")
    plt.axhline(-cfg.max_jerk,ls=":")
    plt.xlabel("Time [s]");plt.ylabel("Jerk [m/s^3]");plt.legend();plt.grid()
    plt.title("MPC jerk constraints")
    from pathlib import Path
    output=Path('artifacts/w07');output.mkdir(parents=True,exist_ok=True)
    for number in plt.get_fignums():
        plt.figure(number).savefig(output/f'mpc_{number}.png',dpi=150)
    plt.show()


if __name__ == '__main__':
    main()
