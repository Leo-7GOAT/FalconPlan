from .models import PredictedState, TimedPredictedState
import math

def predict_cv(
        s0: float,
        v0: float,
        t: float
) -> PredictedState:
    if t < 0:
        raise ValueError(
            "prediction time t must be non-negative"
        )

    if v0 <= 0:
        return PredictedState(
            s=s0,
            v=0.0
        )

    future_s = s0 + v0 * t
    future_v = v0
    return PredictedState(future_s, future_v)

def predict_ca(
        s0: float,
        v0: float,
        a: float,
        t: float
) -> PredictedState:

    if t < 0:
        raise ValueError(
            "prediction time t must be non-negative"
        )

    elif v0 <= 0:
        return PredictedState(
            s=s0,
            v=0.0
        )

    elif a < 0:
        t_stop = - v0 / a
        if t <= t_stop:
            v = v0 + a * t
            s = s0 + v0 * t + 0.5 * a * t**2
        else:
            v = 0
            s_stop = s0 + v0 * t_stop + 0.5 * a * t_stop**2
            return PredictedState(s_stop, v)
    elif a > 0:
        v = v0 + a * t
        s = s0 + v0 * t + 0.5 * a * t ** 2

    elif a == 0:
        return predict_cv(s0, v0, t)

    return PredictedState(s, v)

def predict_cv_horizon(
        s0: float,
        v0: float,
        horizon: float,
        dt: float
) -> list[TimedPredictedState]:
    if horizon < 0:
        raise ValueError(
            "prediction horizon must be non-negative"
        )
    if dt <= 0:
        raise ValueError(
            "dt must be positive"
        )

    predictions = []

    num = math.floor(
        horizon / dt + 1e-12
    ) + 1

    for i in range(num):
        t = i * dt
        predicted_state = predict_cv(s0, v0, t)

        timed_state = TimedPredictedState(t=t, state=predicted_state)

        predictions.append(timed_state)
    return predictions

def predict_ca_horizon(
        s0: float,
        v0: float,
        a: float,
        horizon: float,
        dt: float
) -> list[TimedPredictedState]:
    if horizon < 0:
        raise ValueError(
            "prediction horizon must be non-negative"
        )
    if dt <= 0:
        raise ValueError(
            "dt must be positive"
        )

    predictions = []

    num = math.floor(
        horizon / dt + 1e-12
    ) + 1

    for i in range(num):
        t = i * dt
        predicted_state = predict_ca(s0, v0, a, t)

        timed_state = TimedPredictedState(t=t, state=predicted_state)

        predictions.append(timed_state)
    return predictions
