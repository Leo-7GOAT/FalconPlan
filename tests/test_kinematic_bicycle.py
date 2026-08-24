import math

import pytest

from vehicle_model import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
    KinematicBicycleModel,
)


# ============================================================
# 公共模型
# ============================================================

@pytest.fixture
def model():

    params = VehicleParams(
        wheel_base=2.8,
        max_steer=math.radians(35),
        max_accel=3.0,
        max_decel=6.0,
        max_speed=40.0,
        min_speed=0.0
    )

    return KinematicBicycleModel(params)


# ============================================================
# 1. derivative 基础测试
# ============================================================

@pytest.mark.parametrize(
    "psi, expected_x_dot, expected_y_dot",
    [
        (0.0, 10.0, 0.0),
        (math.pi / 2, 0.0, 10.0),
        (math.pi, -10.0, 0.0),
        (-math.pi / 2, 0.0, -10.0),
    ]
)
def test_derivative_direction(
    model,
    psi,
    expected_x_dot,
    expected_y_dot
):

    state = VehicleState(
        x=0,
        y=0,
        psi=psi,
        v=10
    )

    command = VehicleCommand(
        delta=0,
        a=0
    )

    dot = model.derivative(
        state,
        command
    )

    assert dot.x == pytest.approx(
        expected_x_dot,
        abs=1e-9
    )

    assert dot.y == pytest.approx(
        expected_y_dot,
        abs=1e-9
    )


# ============================================================
# 2. 直行测试
# ============================================================

@pytest.mark.parametrize(
    "dt, expected_x",
    [
        (0.01, 0.1),
        (0.1, 1.0),
        (0.5, 5.0),
        (1.0, 10.0),
    ]
)
def test_euler_straight(
    model,
    dt,
    expected_x
):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=0,
        a=0
    )

    next_state = model.euler_step(
        state,
        command,
        dt
    )

    assert next_state.x == pytest.approx(
        expected_x
    )

    assert next_state.y == pytest.approx(0)
    assert next_state.psi == pytest.approx(0)
    assert next_state.v == pytest.approx(10)


# ============================================================
# 3. 左右转符号
# ============================================================

def test_left_turn_positive_yaw_rate(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0
    )

    dot = model.derivative(
        state,
        command
    )

    assert dot.psi > 0


def test_right_turn_negative_yaw_rate(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=math.radians(-10),
        a=0
    )

    dot = model.derivative(
        state,
        command
    )

    assert dot.psi < 0


# ============================================================
# 4. 转向公式
# ============================================================

@pytest.mark.parametrize(
    "delta_deg",
    [-20, -10, -5, 5, 10, 20]
)
def test_yaw_rate_formula(
    model,
    delta_deg
):

    v = 10.0

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=v
    )

    delta = math.radians(
        delta_deg
    )

    command = VehicleCommand(
        delta=delta,
        a=0
    )

    dot = model.derivative(
        state,
        command
    )

    expected = (
        v / 2.8
        * math.tan(delta)
    )

    assert dot.psi == pytest.approx(
        expected,
        abs=1e-12
    )


# ============================================================
# 5. 加速度
# ============================================================

@pytest.mark.parametrize(
    "a, expected_v",
    [
        (2.0, 12.0),
        (0.0, 10.0),
        (-2.0, 8.0),
    ]
)
def test_euler_acceleration(
    model,
    a,
    expected_v
):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=0,
        a=a
    )

    next_state = model.euler_step(
        state,
        command,
        dt=1.0
    )

    assert next_state.v == pytest.approx(
        expected_v
    )


# ============================================================
# 6. 零速度
# ============================================================

def test_zero_velocity_euler(model):

    state = VehicleState(
        x=3,
        y=5,
        psi=1.0,
        v=0
    )

    command = VehicleCommand(
        delta=math.radians(20),
        a=0
    )

    next_state = model.euler_step(
        state,
        command,
        dt=0.1
    )

    assert next_state.x == pytest.approx(3)
    assert next_state.y == pytest.approx(5)
    assert next_state.psi == pytest.approx(1.0)
    assert next_state.v == pytest.approx(0)


# ============================================================
# 7. Euler 一步已知结果
# ============================================================

def test_euler_known_case(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0
    )

    result = model.euler_step(
        state,
        command,
        dt=0.5
    )

    assert result.x == pytest.approx(
        5.0,
        abs=1e-10
    )

    assert result.y == pytest.approx(
        0.0,
        abs=1e-10
    )

    assert result.psi == pytest.approx(
        0.3148696084,
        abs=1e-9
    )


# ============================================================
# 8. RK4 一步已知结果
# ============================================================

def test_rk4_known_case(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0
    )

    result = model.rk4_step(
        state,
        command,
        dt=0.5
    )

    assert result.x == pytest.approx(
        4.917806,
        abs=1e-5
    )

    assert result.y == pytest.approx(
        0.780695,
        abs=1e-5
    )

    assert result.psi == pytest.approx(
        0.3148696084,
        abs=1e-9
    )


# ============================================================
# 9. 默认 step 应使用 RK4
# ============================================================

def test_default_step_is_rk4(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0
    )

    default_result = model.step(
        state,
        command,
        dt=0.5
    )

    rk4_result = model.rk4_step(
        state,
        command,
        dt=0.5
    )

    assert default_result.x == pytest.approx(
        rk4_result.x
    )

    assert default_result.y == pytest.approx(
        rk4_result.y
    )

    assert default_result.psi == pytest.approx(
        rk4_result.psi
    )


# ============================================================
# 10. method=euler 路由正确
# ============================================================

def test_step_euler_method(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0
    )

    result = model.step(
        state,
        command,
        dt=0.5,
        method="euler"
    )

    expected = model.euler_step(
        state,
        command,
        dt=0.5
    )

    assert result == expected


# ============================================================
# 11. 非法 method
# ============================================================

def test_invalid_method(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=0,
        a=0
    )

    with pytest.raises(ValueError):

        model.step(
            state,
            command,
            dt=0.1,
            method="suizeier"
        )


# ============================================================
# 12. 非法 dt
# ============================================================

@pytest.mark.parametrize(
    "dt",
    [0.0, -0.1, -1.0]
)
def test_invalid_dt(
    model,
    dt
):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=10
    )

    command = VehicleCommand(
        delta=0,
        a=0
    )

    with pytest.raises(ValueError):

        model.step(
            state,
            command,
            dt=dt
        )

def test_rk4_matches_analytical_circle(model):

    state = VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=10.0
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0.0
    )

    dt = 0.5
    T = 5.0
    steps = int(T / dt)

    current = state

    for _ in range(steps):
        current = model.step(
            current,
            command,
            dt,
            method="rk4"
        )

    omega = (
        state.v / 2.8
        * math.tan(command.delta)
    )

    radius = state.v / omega

    x_exact = (
        state.x
        + radius * (
            math.sin(state.psi + omega * T)
            - math.sin(state.psi)
        )
    )

    y_exact = (
        state.y
        + radius * (
            math.cos(state.psi)
            - math.cos(state.psi + omega * T)
        )
    )

    psi_exact = state.psi + omega * T

    assert current.x == pytest.approx(
        x_exact,
        abs=1e-5
    )

    assert current.y == pytest.approx(
        y_exact,
        abs=2e-4
    )

    assert current.psi == pytest.approx(
        psi_exact,
        abs=1e-10
    )

def euler_position_error(model, dt):

    state = VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=10.0
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0.0
    )

    T = 5.0
    steps = int(T / dt)

    current = state

    for _ in range(steps):
        current = model.step(
            current,
            command,
            dt,
            method="euler"
        )

    omega = (
        state.v / 2.8
        * math.tan(command.delta)
    )

    radius = state.v / omega

    x_exact = radius * math.sin(
        omega * T
    )

    y_exact = radius * (
        1 - math.cos(omega * T)
    )

    error = math.hypot(
        current.x - x_exact,
        current.y - y_exact
    )

    return error


def test_euler_convergence(model):

    error_dt_05 = euler_position_error(
        model,
        0.5
    )

    error_dt_01 = euler_position_error(
        model,
        0.1
    )

    error_dt_001 = euler_position_error(
        model,
        0.01
    )

    assert error_dt_01 < error_dt_05
    assert error_dt_001 < error_dt_01

    # Euler是一阶方法
    # dt缩小约10倍，误差也应明显缩小
    assert error_dt_05 / error_dt_01 > 4
    assert error_dt_01 / error_dt_001 > 4

def test_steering_saturation(model):

    command = VehicleCommand(
        delta=math.radians(90),
        a=0
    )

    limited = model.clamp_command(command)

    assert limited.delta == pytest.approx(
        model.params.max_steer
    )

def test_acceleration_saturation(model):

    command = VehicleCommand(
        delta=0,
        a=100
    )

    limited = model.clamp_command(command)

    assert limited.a == pytest.approx(
        model.params.max_accel
    )

def test_brake_saturation(model):

    command = VehicleCommand(
        delta=0,
        a=-100
    )

    limited = model.clamp_command(command)

    assert limited.a == pytest.approx(
        -model.params.max_decel
    )

def test_speed_upper_limit(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=39
    )

    command = VehicleCommand(
        delta=0,
        a=3
    )

    result = model.step(
        state,
        command,
        dt=1.0
    )

    assert result.v == pytest.approx(
        model.params.max_speed
    )

def test_vehicle_cannot_brake_below_zero(model):

    state = VehicleState(
        x=0,
        y=0,
        psi=0,
        v=1
    )

    command = VehicleCommand(
        delta=0,
        a=-6
    )

    result = model.step(
        state,
        command,
        dt=1.0
    )

    assert result.v == pytest.approx(0.0)