from pip._internal.models import candidate


class PIDController:
    def __init__(self,
                 Kp:float,
                 Ki:float,
                 Kd:float,
                 output_min: float | None = None,
                 output_max: float | None = None,
                 ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(
            self,
            target: float,
            measurement: float,
            dt: float,
            feedforward: float = 0.0,
    ) -> float:

        if dt <= 0:
            raise ValueError("dt must be positive")

        error = target - measurement

        if self.previous_error is None:
            Dk = 0.0
        else:
            Dk = (error - self.previous_error) / dt

        candidate_integral = (self.integral + error * dt)

        u_raw_candidate = (
                feedforward
                + self.Kp * error
                + self.Ki * candidate_integral
                + self.Kd * Dk
        )
        integrate = True
        if (
                self.output_max is not None
                and u_raw_candidate > self.output_max
                and error > 0.0
        ):
            integrate = False

        if (
                self.output_min is not None
                and u_raw_candidate < self.output_min
                and error < 0.0
        ):
            integrate = False

        if integrate:
            self.integral = candidate_integral

        u_raw = (
                feedforward
                + self.Kp * error
                + self.Ki * self.integral
                + self.Kd * Dk
        )
        u = u_raw

        if self.output_max is not None:
            u = min(
                u,
                self.output_max,
            )

        if self.output_min is not None:
            u = max(
                u,
                self.output_min,
            )


        self.previous_error = error
        return u