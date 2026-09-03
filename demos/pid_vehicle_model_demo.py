import matplotlib.pyplot as plt

from control.pid import PIDController

from vehicle_model.kinematic_bicycle import (
    KinematicBicycleModel,
)

from vehicle_model.models import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
)


# ============================================================
# Vehicle
# ============================================================

params = VehicleParams(
    wheel_base=2.8,

    max_steer=0.5,

    max_accel=3.0,
    max_decel=6.0,

    max_speed=40.0,
    min_speed=0.0,
)


model = KinematicBicycleModel(
    params=params,
)


# ============================================================
# PID
#
# 注意：
#
# PID output limit 与车辆真实执行限制保持一致。
# ============================================================

pid = PIDController(
    Kp=0.8,
    Ki=0.1,
    Kd=0.05,

    output_min=-params.max_decel,
    output_max=params.max_accel,
)


# ============================================================
# Initial state
# ============================================================

state = VehicleState(
    x=0.0,
    y=0.0,
    psi=0.0,
    v=20.0,
)


target_speed = 25.0

dt = 0.05
total_time = 8.0

steps = int(
    total_time / dt
)


# ============================================================
# Logs
# ============================================================

times = []

speeds = []
speed_errors = []

acceleration_commands = []

positions_x = []


# ============================================================
# Closed loop
# ============================================================

for k in range(
    steps + 1
):

    t = k * dt

    # --------------------------------------------------------
    # 1. Feedback
    #
    # PID reads actual vehicle speed.
    # --------------------------------------------------------

    acceleration_command = pid.update(
        target=target_speed,
        measurement=state.v,
        dt=dt,
    )

    # --------------------------------------------------------
    # 2. Control command
    #
    # Longitudinal-only test:
    # steering = 0
    # --------------------------------------------------------

    command = VehicleCommand(
        delta=0.0,
        a=acceleration_command,
    )

    # --------------------------------------------------------
    # 3. Vehicle dynamics
    #
    # W02 model becomes the controlled plant.
    # --------------------------------------------------------

    state = model.step(
        state=state,
        command=command,
        dt=dt,
        method="rk4",
    )

    # --------------------------------------------------------
    # 4. Logging
    # --------------------------------------------------------

    error = (
        target_speed
        - state.v
    )

    times.append(t)

    speeds.append(
        state.v
    )

    speed_errors.append(
        error
    )

    acceleration_commands.append(
        acceleration_command
    )

    positions_x.append(
        state.x
    )


# ============================================================
# Result
# ============================================================

print(
    "target speed:",
    target_speed,
)

print(
    "final speed:",
    state.v,
)

print(
    "final error:",
    target_speed - state.v,
)

print(
    "final x:",
    state.x,
)

print(
    "final integral:",
    pid.integral,
)

print(
    "max acceleration command:",
    max(acceleration_commands),
)

print(
    "min acceleration command:",
    min(acceleration_commands),
)


# ============================================================
# Speed
# ============================================================

plt.figure()

plt.plot(
    times,
    speeds,
    label="Vehicle speed",
)

plt.axhline(
    target_speed,
    linestyle="--",
    label="Target speed",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Speed [m/s]"
)

plt.title(
    "PID + Kinematic Bicycle Model - Speed Tracking"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# Speed error
# ============================================================

plt.figure()

plt.plot(
    times,
    speed_errors,
)

plt.axhline(
    0.0,
    linestyle="--",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Speed error [m/s]"
)

plt.title(
    "Longitudinal Tracking Error"
)

plt.grid()

plt.show()


# ============================================================
# Acceleration command
# ============================================================

plt.figure()

plt.plot(
    times,
    acceleration_commands,
)

plt.axhline(
    params.max_accel,
    linestyle="--",
    label="Max acceleration",
)

plt.axhline(
    -params.max_decel,
    linestyle="--",
    label="Max deceleration",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Acceleration command [m/s^2]"
)

plt.title(
    "PID Acceleration Command"
)

plt.grid()
plt.legend()

plt.show()