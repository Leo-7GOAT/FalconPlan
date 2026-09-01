import numpy as np
import matplotlib.pyplot as plt

from planning.polynomial import QuinticPolynomial


poly = QuinticPolynomial(
    x0=0.0,
    v0=0.0,
    a0=0.0,
    x1=3.5,
    v1=0.0,
    a1=0.0,
    T=3.0,
)


times = np.linspace(
    0.0,
    3.0,
    101,
)

positions = [
    poly.position(t)
    for t in times
]

velocities = [
    poly.velocity(t)
    for t in times
]

accelerations = [
    poly.acceleration(t)
    for t in times
]

jerks = [
    poly.jerk(t)
    for t in times
]


# ------------------------------------------------------------
# Position
# ------------------------------------------------------------

plt.figure()

plt.plot(
    times,
    positions,
)

plt.xlabel("Time [s]")
plt.ylabel("Lateral position d [m]")
plt.title("Quintic Polynomial - Position")
plt.grid()

plt.show()


# ------------------------------------------------------------
# Velocity
# ------------------------------------------------------------

plt.figure()

plt.plot(
    times,
    velocities,
)

plt.xlabel("Time [s]")
plt.ylabel("Lateral velocity [m/s]")
plt.title("Quintic Polynomial - Velocity")
plt.grid()

plt.show()


# ------------------------------------------------------------
# Acceleration
# ------------------------------------------------------------

plt.figure()

plt.plot(
    times,
    accelerations,
)

plt.xlabel("Time [s]")
plt.ylabel("Lateral acceleration [m/s^2]")
plt.title("Quintic Polynomial - Acceleration")
plt.grid()

plt.show()


# ------------------------------------------------------------
# Jerk
# ------------------------------------------------------------

plt.figure()

plt.plot(
    times,
    jerks,
)

plt.xlabel("Time [s]")
plt.ylabel("Lateral jerk [m/s^3]")
plt.title("Quintic Polynomial - Jerk")
plt.grid()

plt.show()