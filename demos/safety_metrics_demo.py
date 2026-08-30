import math

from behavior import (
    compute_ttc,
    compute_thw,
    assess_risk,
)


print("TTC:")
print(compute_ttc(40, 25, 20))
print(compute_ttc(30, 15, 20))
print(compute_ttc(20, 30, 10))
print(compute_ttc(20, 0, 0))
print(compute_ttc(0, 20, 10))


print("\nTHW:")
print(compute_thw(40, 25))
print(compute_thw(30, 15))
print(compute_thw(20, 30))
print(compute_thw(20, 0))
print(compute_thw(0, 20))


print("\nRisk:")
print(assess_risk(10, 3))
print(assess_risk(5, 1.0))
print(assess_risk(1.0, 3))
print(assess_risk(math.inf, 0.5))