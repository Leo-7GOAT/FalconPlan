import math

from .fsm import BehaviorFSM

from .idm import (
    compute_idm_acceleration,
)

from .mobil import (
    evaluate_lane_change_with_idm,
)

from .prediction import (
    predict_ca_horizon,
)

from .safety_metrics import (
    compute_ttc,
    compute_thw,
    assess_risk,
    compute_risk_horizon,
)

from .models import (
    BehaviorInput,
    BehaviorPlanResult,
    IDMParams,
    MOBILParams,
    RiskLevel,
    LaneChangeEvaluation,
    LongitudinalVehicleState,
)


class BehaviorPlanner:
    """
    W03 behavior planner.

    Pipeline:

        current traffic state
                ↓
        TTC / THW
                ↓
        CV / CA prediction
                ↓
        future risk horizon
                ↓
        IDM longitudinal behavior
                ↓
        MOBIL lane-change evaluation
                ↓
        FSM behavior state
    """

    def __init__(
            self,
            idm_params: IDMParams,
            mobil_params: MOBILParams,
            prediction_horizon: float = 2.0,
            prediction_dt: float = 0.5,
    ):

        if prediction_horizon < 0.0:
            raise ValueError(
                "prediction_horizon must be non-negative"
            )

        if prediction_dt <= 0.0:
            raise ValueError(
                "prediction_dt must be positive"
            )

        self.idm_params = idm_params
        self.mobil_params = mobil_params

        self.prediction_horizon = prediction_horizon
        self.prediction_dt = prediction_dt

        self.fsm = BehaviorFSM()

    def _compute_current_risk(
            self,
            ego: LongitudinalVehicleState,
            front: LongitudinalVehicleState | None,
    ) -> tuple[float, float, RiskLevel]:

        # No front vehicle means no current following risk.
        if front is None:
            return (
                math.inf,
                math.inf,
                RiskLevel.SAFE,
            )

        gap = front.s - ego.s

        ttc = compute_ttc(
            gap=gap,
            ego_speed=ego.v,
            front_speed=front.v,
        )

        thw = compute_thw(
            gap=gap,
            ego_speed=ego.v,
        )

        risk = assess_risk(
            ttc=ttc,
            thw=thw,
        )

        return (
            ttc,
            thw,
            risk,
        )

    def _compute_future_risk_horizon(
            self,
            ego: LongitudinalVehicleState,
            front: LongitudinalVehicleState | None,
            ego_acceleration: float,
            front_acceleration: float,
    ):

        if front is None:
            return []

        ego_predictions = predict_ca_horizon(
            s0=ego.s,
            v0=ego.v,
            a=ego_acceleration,
            horizon=self.prediction_horizon,
            dt=self.prediction_dt,
        )

        front_predictions = predict_ca_horizon(
            s0=front.s,
            v0=front.v,
            a=front_acceleration,
            horizon=self.prediction_horizon,
            dt=self.prediction_dt,
        )

        return compute_risk_horizon(
            ego_predictions=ego_predictions,
            front_predictions=front_predictions,
        )

    def _compute_longitudinal_acceleration(
            self,
            ego: LongitudinalVehicleState,
            front: LongitudinalVehicleState | None,
    ) -> float:

        if front is None:

            return compute_idm_acceleration(
                ego_speed=ego.v,
                gap=None,
                front_speed=None,
                params=self.idm_params,
            )

        gap = front.s - ego.s

        if gap <= 0.0:
            raise ValueError(
                "front vehicle must be ahead of ego"
            )

        return compute_idm_acceleration(
            ego_speed=ego.v,
            gap=gap,
            front_speed=front.v,
            params=self.idm_params,
        )

    def _evaluate_target_lane(
            self,
            ego: LongitudinalVehicleState,

            old_leader: LongitudinalVehicleState | None,
            old_follower: LongitudinalVehicleState | None,

            new_leader: LongitudinalVehicleState | None,
            new_follower: LongitudinalVehicleState | None,

    ) -> LaneChangeEvaluation:

        return evaluate_lane_change_with_idm(
            ego=ego,

            old_leader=old_leader,
            old_follower=old_follower,

            new_leader=new_leader,
            new_follower=new_follower,

            idm_params=self.idm_params,
            mobil_params=self.mobil_params,
        )

    @staticmethod
    def _choose_lane(
            left_eval: LaneChangeEvaluation | None,
            right_eval: LaneChangeEvaluation | None,
    ) -> tuple[bool, bool]:
        """
        Convert MOBIL results into FSM lane availability.

        If both sides are beneficial, choose the side
        with the larger incentive.

        Equal incentive currently prefers left.
        """

        left_available = (
            left_eval is not None
            and left_eval.beneficial
        )

        right_available = (
            right_eval is not None
            and right_eval.beneficial
        )

        if left_available and right_available:

            if (
                    left_eval.incentive
                    >= right_eval.incentive
            ):
                right_available = False

            else:
                left_available = False

        return (
            left_available,
            right_available,
        )

    def step(
            self,
            ego: LongitudinalVehicleState,

            front_current_lane:
            LongitudinalVehicleState | None = None,

            rear_current_lane:
            LongitudinalVehicleState | None = None,

            front_left_lane:
            LongitudinalVehicleState | None = None,

            rear_left_lane:
            LongitudinalVehicleState | None = None,

            front_right_lane:
            LongitudinalVehicleState | None = None,

            rear_right_lane:
            LongitudinalVehicleState | None = None,

            left_lane_exists: bool = False,
            right_lane_exists: bool = False,

            lane_change_completed: bool = False,

            ego_prediction_acceleration: float = 0.0,
            front_prediction_acceleration: float = 0.0,

    ) -> BehaviorPlanResult:

        # ====================================================
        # 1. Current risk
        # ====================================================

        ttc, thw, current_risk = (
            self._compute_current_risk(
                ego=ego,
                front=front_current_lane,
            )
        )

        # ====================================================
        # 2. Future risk horizon
        # ====================================================

        risk_horizon = (
            self._compute_future_risk_horizon(
                ego=ego,
                front=front_current_lane,

                ego_acceleration=(
                    ego_prediction_acceleration
                ),

                front_acceleration=(
                    front_prediction_acceleration
                ),
            )
        )

        overall_risk = current_risk

        for predicted_risk in risk_horizon:

            if (
                    predicted_risk.risk.value
                    > overall_risk.value
            ):
                overall_risk = (
                    predicted_risk.risk
                )

        # ====================================================
        # 3. IDM longitudinal acceleration
        # ====================================================

        idm_acceleration = (
            self._compute_longitudinal_acceleration(
                ego=ego,
                front=front_current_lane,
            )
        )

        # ====================================================
        # 4. MOBIL target-lane evaluation
        # ====================================================

        left_eval = None
        right_eval = None

        # Emergency braking has higher priority than
        # lane-change evaluation.
        if overall_risk != RiskLevel.EMERGENCY:

            if left_lane_exists:

                left_eval = (
                    self._evaluate_target_lane(
                        ego=ego,

                        old_leader=front_current_lane,
                        old_follower=rear_current_lane,

                        new_leader=front_left_lane,
                        new_follower=rear_left_lane,
                    )
                )

            if right_lane_exists:

                right_eval = (
                    self._evaluate_target_lane(
                        ego=ego,

                        old_leader=front_current_lane,
                        old_follower=rear_current_lane,

                        new_leader=front_right_lane,
                        new_follower=rear_right_lane,
                    )
                )

        # ====================================================
        # 5. Decide which lane candidate is allowed
        # ====================================================

        left_available, right_available = (
            self._choose_lane(
                left_eval=left_eval,
                right_eval=right_eval,
            )
        )

        # ====================================================
        # 6. FSM
        # ====================================================

        behavior_input = BehaviorInput(
            risk=overall_risk,

            front_vehicle_present=(
                front_current_lane is not None
            ),

            left_lane_available=left_available,
            right_lane_available=right_available,

            lane_change_completed=(
                lane_change_completed
            ),
        )

        state = self.fsm.update(
            behavior_input
        )

        # ====================================================
        # 7. Output
        # ====================================================

        return BehaviorPlanResult(
            state=state,

            current_risk=current_risk,
            risk=overall_risk,

            ttc=ttc,
            thw=thw,

            idm_acceleration=idm_acceleration,

            left_eval=left_eval,
            right_eval=right_eval,

            risk_horizon=tuple(
                risk_horizon
            ),
        )