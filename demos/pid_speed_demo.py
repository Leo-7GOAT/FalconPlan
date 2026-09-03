import matplotlib.pyplot as plt

from control.pid import PIDController


# ============================================================
# Controller
# ============================================================

pid = PIDController(
    Kp=0.8,
    Ki=0.1,
    Kd=0.05,

    output_min=-6.0,
    output_max=3.0,
)


# ============================================================
# Simulation settings
# ============================================================

target_speed = 35.0
current_speed = 20.0

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
errors = []
commands = []


# ============================================================
# Closed-loop simulation
# ============================================================

for k in range(steps + 1):

    t = k * dt

    # --------------------------------------------------------
    # Controller sees current feedback.
    # --------------------------------------------------------

    acceleration_command = pid.update(
        target=target_speed,
        measurement=current_speed,
        dt=dt,
    )

    # --------------------------------------------------------
    # Very simple longitudinal plant:
    #
    # v(k+1) = v(k) + a(k) * dt
    # --------------------------------------------------------

    current_speed = (
        current_speed
        + acceleration_command * dt
    )

    error = (
        target_speed
        - current_speed
    )

    times.append(t)
    speeds.append(current_speed)
    errors.append(error)
    commands.append(
        acceleration_command
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
    current_speed,
)

print(
    "final error:",
    target_speed - current_speed,
)

print(
    "final integral:",
    pid.integral,
)


# ============================================================
# Speed
# ============================================================

plt.figure()

plt.plot(
    times,
    speeds,
    label="Actual speed",
)

plt.axhline(
    target_speed,
    linestyle="--",
    label="Target speed",
)

plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.title("PID Speed Tracking")

plt.grid()
plt.legend()
plt.show()


# ============================================================
# Error
# ============================================================

plt.figure()

plt.plot(
    times,
    errors,
)

plt.xlabel("Time [s]")
plt.ylabel("Speed error [m/s]")
plt.title("PID Speed Error")

plt.grid()
plt.show()


# ============================================================
# Control command
# ============================================================

plt.figure()

plt.plot(
    times,
    commands,
)

plt.xlabel("Time [s]")
plt.ylabel("Acceleration command [m/s^2]")
plt.title("PID Acceleration Command")

plt.grid()
plt.show()