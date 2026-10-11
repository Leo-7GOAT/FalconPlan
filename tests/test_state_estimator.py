import pytest,numpy as np
from vehicle_model.state_estimator import PlanarEKF
from vehicle_model.models import VehicleState
def test_gnss_imu_numerical_convergence_and_covariance():
    e=PlanarEKF(VehicleState(0,0,0,2));rng=np.random.default_rng(7);errors=[]
    for k in range(100):
        e.predict(0,0,.05);x=(k+1)*.1;e.update_position(x+rng.normal(0,.05),rng.normal(0,.05),k*.05);errors.append(e.state.x-x)
    assert np.sqrt(np.mean(np.square(errors)))<.05
    assert np.min(np.linalg.eigvalsh(e.cov))>0
    with pytest.raises(ValueError):e.update_position(0,0,.1)
def test_gnss_outlier_gate_and_imu_failure():
    e=PlanarEKF(VehicleState(0,0,0,0))
    assert not e.update_position(100,100,0)
    assert e.state.x==0
    with pytest.raises(ValueError):e.predict(float('nan'),0,.05)
    with pytest.raises(ValueError):e.predict(0,0,0)

def test_compass_wrap_and_covariance():
    import math
    e=PlanarEKF(VehicleState(0,0,math.pi-.01,0))
    e.update_heading(-math.pi+.01)
    assert abs(math.atan2(math.sin(e.state.psi+math.pi-.01),math.cos(e.state.psi+math.pi-.01)))<.01
    assert np.min(np.linalg.eigvalsh(e.cov))>0
    with pytest.raises(ValueError):e.update_heading(float('nan'))
