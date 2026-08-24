import math

from .models import(
    VehicleState,
    VehicleCommand,
    VehicleParams
)

class KinematicBicycleModel:

    def __init__(self, params:VehicleParams):
        self.params = params

    def derivative(
            self,
            state: VehicleState,
            command: VehicleCommand
    ) -> VehicleState:

        x_dot = state.v * math.cos(state.psi)
        y_dot = state.v * math.sin(state.psi)

        psi_dot = (state.v
                   / self.params.wheel_base
                   * math.tan(command.delta))

        v_dot = command.a
        return VehicleState(
            x=x_dot,
            y=y_dot,
            psi=psi_dot,
            v=v_dot,
        )

    def euler_step(self,
                   state: VehicleState,
                   command: VehicleCommand,
                   dt: float
                   ) -> VehicleState:
        dot = self.derivative(state, command)

        return VehicleState(
            x=state.x + dot.x * dt,
            y=state.y + dot.y * dt,
            psi=state.psi + dot.psi * dt,
            v=state.v + dot.v * dt
        )

    def rk4_step(
            self,
            state: VehicleState,
            command: VehicleCommand,
            dt: float
    ) -> VehicleState:
        # k1
        k1 = self.derivative(
            state,
            command
        )

        # k2
        state_k2 = VehicleState(
            x=state.x + k1.x * dt / 2,
            y=state.y + k1.y * dt / 2,
            psi=state.psi + k1.psi * dt / 2,
            v=state.v + k1.v * dt / 2
        )

        k2 = self.derivative(
            state_k2,
            command
        )

        # k3
        state_k3 = VehicleState(
            x=state.x + k2.x * dt / 2,
            y=state.y + k2.y * dt / 2,
            psi=state.psi + k2.psi * dt / 2,
            v=state.v + k2.v * dt / 2
        )

        k3 = self.derivative(
            state_k3,
            command
        )

        # k4
        state_k4 = VehicleState(
            x=state.x + k3.x * dt,
            y=state.y + k3.y * dt,
            psi=state.psi + k3.psi * dt,
            v=state.v + k3.v * dt
        )

        k4 = self.derivative(
            state_k4,
            command
        )

        # 加权平均
        x_next = state.x + dt / 6 * (
                k1.x + 2 * k2.x + 2 * k3.x + k4.x
        )

        y_next = state.y + dt / 6 * (
                k1.y + 2 * k2.y + 2 * k3.y + k4.y
        )

        psi_next = state.psi + dt / 6 * (
                k1.psi + 2 * k2.psi + 2 * k3.psi + k4.psi
        )

        v_next = state.v + dt / 6 * (
                k1.v + 2 * k2.v + 2 * k3.v + k4.v
        )

        return VehicleState(
            x=x_next,
            y=y_next,
            psi=psi_next,
            v=v_next
        )

    def step(
            self,
            state: VehicleState,
            command: VehicleCommand,
            dt: float,
            method: str = "rk4"
    ) -> VehicleState:

        if dt <= 0:
            raise ValueError("dt must be greater than 0")

        command = self.clamp_command(command)

        if method == "euler":
            next_state = self.euler_step(
                state,
                command,
                dt
            )

        elif method == "rk4":
            next_state = self.rk4_step(
                state,
                command,
                dt
            )

        else:
            raise ValueError(
                f"Unknown integration method: {method}"
            )

        # 速度物理范围保护
        v_limited = max(
            self.params.min_speed,
            min(
                self.params.max_speed,
                next_state.v
            )
        )

        return VehicleState(
            x=next_state.x,
            y=next_state.y,
            psi=next_state.psi,
            v=v_limited
        )

    def clamp_command(
            self,
            command: VehicleCommand
    ) -> VehicleCommand:

        delta = max(
            -self.params.max_steer,
            min(self.params.max_steer, command.delta)
        )

        a = max(
            -self.params.max_decel,
            min(self.params.max_accel, command.a)
        )

        return VehicleCommand(
            delta=delta,
            a=a
        )