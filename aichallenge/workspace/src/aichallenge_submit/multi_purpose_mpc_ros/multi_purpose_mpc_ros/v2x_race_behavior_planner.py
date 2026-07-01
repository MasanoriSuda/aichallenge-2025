"""V2X race behavior planner for multi-vehicle follow/yield/overtake.

The planner is intentionally ROS-free. It owns the race-level state machine
and reuses the Gate2 overtake planner for side selection and lateral behavior.
"""

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from multi_purpose_mpc_ros.v2x_overtake_planner import (
    V2XOvertakeConfig,
    V2XOvertakePlanner,
    V2XOvertakeResult,
)


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _stamp_to_seconds(stamp) -> Optional[float]:
    if stamp is None:
        return None
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except AttributeError:
        return None


def _get_attr(obj, name: str, default=None):
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


@dataclass
class V2XRaceBehaviorConfig:
    detection_range_m: float = 40.0
    corridor_half_width_m: float = 2.0
    front_conflict_lateral_window_m: float = 2.6
    front_conflict_gap_m: float = 10.0
    self_ignore_radius_m: float = 0.75
    min_follow_gap_m: float = 3.0
    time_headway_sec: float = 0.8
    follow_kp_gap: float = 0.4
    follow_kd_closing: float = 0.8
    follow_speed_cap_mps: float = 12.0 / 3.6
    emergency_stop_gap_m: float = 2.0
    yield_detect_range_m: float = 12.0
    yield_start_gap_m: float = 8.0
    yield_release_gap_m: float = 14.0
    yield_min_closing_speed_mps: float = 0.4
    yield_speed_cap_mps: float = 8.0 / 3.6
    yield_timeout_sec: float = 4.0
    yield_side_hold_sec: float = 1.0
    catchup_detect_range_m: float = 120.0
    catchup_start_gap_m: float = 18.0
    catchup_release_gap_m: float = 8.0
    catchup_speed_cap_mps: float = 10.0 / 3.6
    catchup_timeout_sec: float = 30.0
    approach_start_gap_m: float = 34.0
    approach_speed_cap_mps: float = 18.0 / 3.6
    min_overtake_start_gap_m: float = 4.0
    max_overtake_start_gap_m: float = 24.0
    abort_gap_m: float = 2.5
    abort_escape_lateral_threshold_m: float = 0.45
    return_clearance_m: float = 8.0
    return_rear_clearance_m: float = 4.0
    preferred_side: str = "right"
    side_selection_policy: str = "largest_margin"
    side_margin_tie_threshold_m: float = 0.3
    lateral_offset_m: float = 1.4
    lateral_offset_rate_mps: float = 1.2
    constraint_half_width_m: float = 0.55
    constraint_transition_horizon_ratio: float = 0.35
    constraint_initial_progress: float = 0.35
    min_lateral_clearance_m: float = 1.6
    min_wall_clearance_m: float = 0.5
    wall_safety_margin_m: float = 0.2
    wall_check_horizon_m: float = 16.0
    vehicle_width_m: float = 1.45
    overtake_speed_cap_mps: float = 12.0 / 3.6
    prepare_speed_cap_mps: float = 9.0 / 3.6
    overtake_steer_rate_max: float = 0.8
    max_overtake_target_speed_mps: float = 12.0 / 3.6
    min_closing_speed_mps: float = 0.4
    min_ttc_sec: float = 0.8
    stale_timeout_sec: float = 2.0
    target_lost_hold_sec: float = 1.0
    circular_path: bool = True
    return_offset_threshold_m: float = 0.25
    overtake_cooldown_sec: float = 2.0
    side_lock_min_sec: float = 1.0
    side_switch_center_threshold_m: float = 0.45

    @classmethod
    def from_config(
        cls,
        cfg,
        vehicle_width_m: float,
    ) -> "V2XRaceBehaviorConfig":
        preferred_side = str(_get_attr(cfg, "preferred_side", "right")).lower()
        if preferred_side not in ("left", "right"):
            preferred_side = "right"
        side_selection_policy = str(
            _get_attr(cfg, "side_selection_policy", "largest_margin")).lower()
        if side_selection_policy not in ("largest_margin", "preferred"):
            side_selection_policy = "largest_margin"

        return cls(
            detection_range_m=float(_get_attr(cfg, "detection_range_m", 40.0)),
            corridor_half_width_m=float(_get_attr(cfg, "corridor_half_width_m", 2.0)),
            front_conflict_lateral_window_m=float(
                _get_attr(cfg, "front_conflict_lateral_window_m", 2.6)),
            front_conflict_gap_m=float(_get_attr(cfg, "front_conflict_gap_m", 10.0)),
            self_ignore_radius_m=float(_get_attr(cfg, "self_ignore_radius_m", 0.75)),
            min_follow_gap_m=float(_get_attr(cfg, "min_follow_gap_m", 3.0)),
            time_headway_sec=float(_get_attr(cfg, "time_headway_sec", 0.8)),
            follow_kp_gap=float(_get_attr(cfg, "follow_kp_gap", 0.4)),
            follow_kd_closing=float(_get_attr(cfg, "follow_kd_closing", 0.8)),
            follow_speed_cap_mps=float(_get_attr(cfg, "follow_speed_cap_kmph", 12.0)) / 3.6,
            emergency_stop_gap_m=float(_get_attr(cfg, "emergency_stop_gap_m", 2.0)),
            yield_detect_range_m=float(_get_attr(cfg, "yield_detect_range_m", 12.0)),
            yield_start_gap_m=float(_get_attr(cfg, "yield_start_gap_m", 8.0)),
            yield_release_gap_m=float(_get_attr(cfg, "yield_release_gap_m", 14.0)),
            yield_min_closing_speed_mps=float(
                _get_attr(cfg, "yield_min_closing_speed_mps", 0.4)),
            yield_speed_cap_mps=float(_get_attr(cfg, "yield_speed_cap_kmph", 8.0)) / 3.6,
            yield_timeout_sec=float(_get_attr(cfg, "yield_timeout_sec", 4.0)),
            yield_side_hold_sec=float(_get_attr(cfg, "yield_side_hold_sec", 1.0)),
            catchup_detect_range_m=float(_get_attr(cfg, "catchup_detect_range_m", 120.0)),
            catchup_start_gap_m=float(_get_attr(cfg, "catchup_start_gap_m", 18.0)),
            catchup_release_gap_m=float(_get_attr(cfg, "catchup_release_gap_m", 8.0)),
            catchup_speed_cap_mps=float(_get_attr(cfg, "catchup_speed_cap_kmph", 10.0)) / 3.6,
            catchup_timeout_sec=float(_get_attr(cfg, "catchup_timeout_sec", 30.0)),
            approach_start_gap_m=float(_get_attr(cfg, "approach_start_gap_m", 34.0)),
            approach_speed_cap_mps=float(_get_attr(cfg, "approach_speed_cap_kmph", 18.0)) / 3.6,
            min_overtake_start_gap_m=float(_get_attr(cfg, "min_overtake_start_gap_m", 4.0)),
            max_overtake_start_gap_m=float(_get_attr(cfg, "max_overtake_start_gap_m", 24.0)),
            abort_gap_m=float(_get_attr(cfg, "abort_gap_m", 2.5)),
            abort_escape_lateral_threshold_m=float(
                _get_attr(cfg, "abort_escape_lateral_threshold_m", 0.45)),
            return_clearance_m=float(_get_attr(cfg, "return_clearance_m", 8.0)),
            return_rear_clearance_m=float(_get_attr(cfg, "return_rear_clearance_m", 4.0)),
            preferred_side=preferred_side,
            side_selection_policy=side_selection_policy,
            side_margin_tie_threshold_m=float(_get_attr(cfg, "side_margin_tie_threshold_m", 0.3)),
            lateral_offset_m=float(_get_attr(cfg, "lateral_offset_m", 1.4)),
            lateral_offset_rate_mps=float(_get_attr(cfg, "lateral_offset_rate_mps", 1.2)),
            constraint_half_width_m=float(_get_attr(cfg, "constraint_half_width_m", 0.55)),
            constraint_transition_horizon_ratio=float(
                _get_attr(cfg, "constraint_transition_horizon_ratio", 0.35)),
            constraint_initial_progress=float(_get_attr(cfg, "constraint_initial_progress", 0.35)),
            min_lateral_clearance_m=float(_get_attr(cfg, "min_lateral_clearance_m", 1.6)),
            min_wall_clearance_m=float(_get_attr(cfg, "min_wall_clearance_m", 0.5)),
            wall_safety_margin_m=float(_get_attr(cfg, "wall_safety_margin_m", 0.2)),
            wall_check_horizon_m=float(_get_attr(cfg, "wall_check_horizon_m", 16.0)),
            vehicle_width_m=float(_get_attr(cfg, "vehicle_width_m", vehicle_width_m)),
            overtake_speed_cap_mps=float(_get_attr(cfg, "overtake_speed_cap_kmph", 12.0)) / 3.6,
            prepare_speed_cap_mps=float(_get_attr(cfg, "prepare_speed_cap_kmph", 9.0)) / 3.6,
            overtake_steer_rate_max=float(_get_attr(cfg, "overtake_steer_rate_max", 0.8)),
            max_overtake_target_speed_mps=float(
                _get_attr(cfg, "max_overtake_target_speed_kmph", 12.0)) / 3.6,
            min_closing_speed_mps=float(_get_attr(cfg, "min_closing_speed_mps", 0.4)),
            min_ttc_sec=float(_get_attr(cfg, "min_ttc_sec", 0.8)),
            stale_timeout_sec=float(_get_attr(cfg, "stale_timeout_sec", 2.0)),
            target_lost_hold_sec=float(_get_attr(cfg, "target_lost_hold_sec", 1.0)),
            circular_path=bool(_get_attr(cfg, "circular_path", True)),
            return_offset_threshold_m=float(_get_attr(cfg, "return_offset_threshold_m", 0.25)),
            overtake_cooldown_sec=float(_get_attr(cfg, "overtake_cooldown_sec", 2.0)),
            side_lock_min_sec=float(_get_attr(cfg, "side_lock_min_sec", 1.0)),
            side_switch_center_threshold_m=float(
                _get_attr(cfg, "side_switch_center_threshold_m", 0.45)),
        )

    def to_overtake_config(self) -> V2XOvertakeConfig:
        return V2XOvertakeConfig(
            detection_range_m=self.detection_range_m,
            corridor_half_width_m=self.corridor_half_width_m,
            self_ignore_radius_m=self.self_ignore_radius_m,
            min_overtake_start_gap_m=self.min_overtake_start_gap_m,
            max_overtake_start_gap_m=self.max_overtake_start_gap_m,
            abort_gap_m=self.abort_gap_m,
            abort_escape_lateral_threshold_m=self.abort_escape_lateral_threshold_m,
            return_clearance_m=self.return_clearance_m,
            preferred_side=self.preferred_side,
            side_selection_policy=self.side_selection_policy,
            side_margin_tie_threshold_m=self.side_margin_tie_threshold_m,
            lateral_offset_m=self.lateral_offset_m,
            lateral_offset_rate_mps=self.lateral_offset_rate_mps,
            constraint_half_width_m=self.constraint_half_width_m,
            min_lateral_clearance_m=self.min_lateral_clearance_m,
            min_wall_clearance_m=self.min_wall_clearance_m,
            wall_safety_margin_m=self.wall_safety_margin_m,
            wall_check_horizon_m=self.wall_check_horizon_m,
            vehicle_width_m=self.vehicle_width_m,
            overtake_speed_cap_mps=self.overtake_speed_cap_mps,
            follow_speed_cap_mps=self.follow_speed_cap_mps,
            max_overtake_target_speed_mps=self.max_overtake_target_speed_mps,
            min_closing_speed_mps=self.min_closing_speed_mps,
            min_ttc_sec=self.min_ttc_sec,
            stale_timeout_sec=self.stale_timeout_sec,
            target_lost_hold_sec=self.target_lost_hold_sec,
            circular_path=self.circular_path,
            return_offset_threshold_m=self.return_offset_threshold_m,
        )


@dataclass
class V2XRaceBehaviorResult:
    active: bool
    state: str
    speed_cap_mps: float
    target_lateral_offset_m: float
    reason: str
    vehicle_id: Optional[str] = None
    target_id: Optional[str] = None
    target_role: Optional[str] = None
    side: Optional[str] = None
    gap_m: Optional[float] = None
    signed_gap_m: Optional[float] = None
    yaw_signed_gap_m: Optional[float] = None
    lateral_offset_m: Optional[float] = None
    relative_speed_mps: Optional[float] = None
    target_speed_mps: Optional[float] = None
    left_wall_margin_m: Optional[float] = None
    right_wall_margin_m: Optional[float] = None
    yield_active: bool = False
    yield_target_id: Optional[str] = None
    cooldown_remaining_sec: Optional[float] = None


@dataclass
class _Snapshot:
    vehicle_id: str
    x: float
    y: float
    stamp_sec: float


@dataclass
class _Projection:
    s_m: float
    lateral_signed_m: float
    path_index: int


@dataclass
class _RaceRelation:
    snapshot: _Snapshot
    forward_gap_m: float
    signed_gap_m: float
    lateral_signed_m: float
    lateral_offset_m: float
    yaw_lateral_signed_m: float
    path_index: int
    yaw_signed_gap_m: float
    relative_speed_mps: Optional[float]
    target_speed_mps: Optional[float]


class _SegmentOvertakePlanner(V2XOvertakePlanner):
    """Gate2 planner with segment projection for sparse waypoint paths."""

    def _path_relation(
        self,
        ego_x: float,
        ego_y: float,
        snapshot,
        path_xy: Sequence[Tuple[float, float]],
    ) -> Optional[Tuple[float, float, float, int]]:
        if len(path_xy) < 2:
            return None

        cum_s, total_s = self._cumulative_distance(path_xy)
        ego_projection = _project_to_path(
            ego_x, ego_y, path_xy, cum_s, self._cfg.circular_path)
        target_projection = _project_to_path(
            snapshot.x, snapshot.y, path_xy, cum_s, self._cfg.circular_path)
        if ego_projection is None or target_projection is None:
            return None

        signed_gap = _signed_gap(
            target_projection.s_m,
            ego_projection.s_m,
            total_s,
            self._cfg.circular_path,
        )
        forward_gap = signed_gap
        if self._cfg.circular_path and forward_gap <= 0.0:
            forward_gap += total_s

        return (
            forward_gap,
            signed_gap,
            target_projection.lateral_signed_m,
            target_projection.path_index,
        )


class V2XRaceBehaviorPlanner:
    CLEAR = "clear"
    FOLLOW = "follow"
    YIELD = "yield"
    CATCHUP_WAIT = "catchup_wait"
    PREPARE = "prepare_overtake"
    OVERTAKING = "overtaking"
    RETURN = "return_to_line"
    COOLDOWN = "cooldown"
    ABORT = "abort"

    def __init__(self, config: V2XRaceBehaviorConfig):
        self._cfg = config
        self._snapshots: Dict[str, _Snapshot] = {}
        self._overtake = _SegmentOvertakePlanner(config.to_overtake_config())
        self._yield_target_id: Optional[str] = None
        self._yield_started_sec: Optional[float] = None
        self._yield_hold_until_sec: Optional[float] = None
        self._catchup_target_id: Optional[str] = None
        self._catchup_started_sec: Optional[float] = None
        self._cooldown_until_sec: Optional[float] = None
        self._forced_abort_reason: Optional[str] = None

    def update_v2x(self, msg, now_sec: Optional[float] = None) -> None:
        msg_stamp = _stamp_to_seconds(_get_attr(_get_attr(msg, "header"), "stamp"))
        for vehicle in getattr(msg, "vehicles", []):
            vehicle_id = str(getattr(vehicle, "vehicle_id", ""))
            position = getattr(vehicle, "position", None)
            if not vehicle_id or position is None:
                continue

            x = float(getattr(position, "x", math.nan))
            y = float(getattr(position, "y", math.nan))
            if not (_finite(x) and _finite(y)):
                continue

            vehicle_stamp = _stamp_to_seconds(
                _get_attr(_get_attr(vehicle, "header"), "stamp"))
            stamp_sec = vehicle_stamp
            if stamp_sec is None:
                stamp_sec = msg_stamp
            if stamp_sec is None:
                stamp_sec = now_sec
            if stamp_sec is None:
                continue

            self._snapshots[vehicle_id] = _Snapshot(
                vehicle_id=vehicle_id,
                x=x,
                y=y,
                stamp_sec=float(stamp_sec),
            )

        self._overtake.update_v2x(msg, now_sec=now_sec)

    def force_abort(self, reason: str = "external_abort") -> None:
        self._forced_abort_reason = str(reason or "external_abort")
        self._yield_target_id = None
        self._yield_started_sec = None
        self._yield_hold_until_sec = None
        self._catchup_target_id = None
        self._catchup_started_sec = None
        self._overtake.force_abort(reason)

    def compute_behavior(
        self,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_v_mps: float,
        reference_xy,
        reference_widths,
        now_sec: float,
        base_speed_mps: float,
        current_lateral_offset_m: float = 0.0,
        velocity_lookup: Optional[Callable[[str], Tuple[float, float]]] = None,
    ) -> V2XRaceBehaviorResult:
        base_speed_mps = max(0.0, float(base_speed_mps))

        if self._forced_abort_reason is not None:
            reason = self._forced_abort_reason
            self._forced_abort_reason = None
            return V2XRaceBehaviorResult(
                active=True,
                state=self.ABORT,
                speed_cap_mps=0.0,
                target_lateral_offset_m=0.0,
                reason=reason,
            )

        path_xy = _normalize_path(reference_xy)
        relations = self._collect_relations(
            ego_x=ego_x,
            ego_y=ego_y,
            ego_yaw=ego_yaw,
            ego_v_mps=ego_v_mps,
            path_xy=path_xy,
            now_sec=now_sec,
            velocity_lookup=velocity_lookup,
        )

        front_target = self._front_target(relations)
        if (
            front_target is not None
            and self._effective_front_gap_m(front_target) <= self._cfg.emergency_stop_gap_m
        ):
            return self._emergency_stop(base_speed_mps, front_target)

        if getattr(self._overtake, "_state", self.CLEAR) not in (
            self.PREPARE,
            self.OVERTAKING,
            self.RETURN,
        ):
            catchup_result = self._compute_catchup_wait(
                relations=relations,
                front_target=front_target,
                now_sec=now_sec,
                base_speed_mps=base_speed_mps,
            )
            if catchup_result is not None:
                return catchup_result

            yield_result = self._compute_yield(
                relations=relations,
                front_target=front_target,
                now_sec=now_sec,
                base_speed_mps=base_speed_mps,
            )
            if yield_result is not None:
                return yield_result

        if self._cooldown_active(now_sec):
            return self._cooldown_result(
                front_target=front_target,
                ego_v_mps=ego_v_mps,
                now_sec=now_sec,
                base_speed_mps=base_speed_mps,
            )

        overtake_result = self._overtake.compute_behavior(
            ego_x=ego_x,
            ego_y=ego_y,
            ego_yaw=ego_yaw,
            ego_v_mps=ego_v_mps,
            reference_xy=path_xy,
            reference_widths=reference_widths,
            now_sec=now_sec,
            base_speed_mps=base_speed_mps,
            current_lateral_offset_m=current_lateral_offset_m,
            velocity_lookup=velocity_lookup,
        )

        if overtake_result.state == self.CLEAR and overtake_result.reason == "returned":
            self._cooldown_until_sec = now_sec + max(0.0, self._cfg.overtake_cooldown_sec)
            return self._cooldown_result(
                front_target=front_target,
                ego_v_mps=ego_v_mps,
                now_sec=now_sec,
                base_speed_mps=base_speed_mps,
                reason="overtake_returned",
            )

        if overtake_result.state == self.FOLLOW and front_target is not None:
            return self._follow_result(
                front_target,
                base_speed_mps,
                ego_v_mps,
                overtake_result.reason,
                overtake_result,
            )

        if overtake_result.state == self.FOLLOW and front_target is None:
            return V2XRaceBehaviorResult(
                active=False,
                state=self.CLEAR,
                speed_cap_mps=base_speed_mps,
                target_lateral_offset_m=0.0,
                reason="no_race_front_target",
            )

        if overtake_result.state == self.PREPARE and front_target is None:
            return V2XRaceBehaviorResult(
                active=False,
                state=self.CLEAR,
                speed_cap_mps=base_speed_mps,
                target_lateral_offset_m=0.0,
                reason="no_race_front_target",
            )

        if (
            front_target is not None
            and overtake_result.state in (self.PREPARE, self.OVERTAKING)
            and self._side_switch_blocked(overtake_result, current_lateral_offset_m)
        ):
            return self._follow_result(
                front_target,
                base_speed_mps,
                ego_v_mps,
                "side_switch_blocked",
                overtake_result,
            )

        if overtake_result.state == self.CLEAR and front_target is not None:
            return self._follow_result(
                front_target,
                base_speed_mps,
                ego_v_mps,
                "front_conflict",
            )

        if overtake_result.state == self.RETURN:
            blocker = self._return_blocker(relations, overtake_result.vehicle_id)
            if blocker is not None:
                side = overtake_result.side or self._cfg.preferred_side
                sign = 1.0 if side == "left" else -1.0
                return self._from_overtake_result(
                    overtake_result,
                    state=self.OVERTAKING,
                    speed_cap_mps=min(base_speed_mps, self._cfg.overtake_speed_cap_mps),
                    target_lateral_offset_m=sign * self._cfg.lateral_offset_m,
                    reason=f"return_blocked_by_{blocker.snapshot.vehicle_id}",
                )

        return self._from_overtake_result(overtake_result)

    def _compute_yield(
        self,
        relations: List[_RaceRelation],
        front_target: Optional[_RaceRelation],
        now_sec: float,
        base_speed_mps: float,
    ) -> Optional[V2XRaceBehaviorResult]:
        if self._yield_hold_until_sec is not None:
            if now_sec <= self._yield_hold_until_sec:
                return V2XRaceBehaviorResult(
                    active=True,
                    state=self.COOLDOWN,
                    speed_cap_mps=min(base_speed_mps, self._cfg.follow_speed_cap_mps),
                    target_lateral_offset_m=0.0,
                    reason="yield_side_hold",
                    cooldown_remaining_sec=self._yield_hold_until_sec - now_sec,
                )
            self._yield_hold_until_sec = None

        active_yield_relation = self._relation_by_id(relations, self._yield_target_id)
        if active_yield_relation is not None:
            if (
                not self._is_rear_relation(active_yield_relation)
                and active_yield_relation.forward_gap_m >= self._cfg.yield_release_gap_m
            ):
                return self._release_yield(now_sec, base_speed_mps, "yield_target_passed")
            if (
                self._yield_started_sec is not None
                and now_sec - self._yield_started_sec >= self._cfg.yield_timeout_sec
            ):
                return self._release_yield(now_sec, base_speed_mps, "yield_timeout")
            if self._should_continue_yield(active_yield_relation):
                return self._yield_result(base_speed_mps, active_yield_relation, "yield_active")

        yield_target = self._yield_target(relations)
        if yield_target is None:
            self._yield_target_id = None
            self._yield_started_sec = None
            return None

        if front_target is not None and front_target.forward_gap_m <= self._cfg.min_follow_gap_m:
            return None

        self._yield_target_id = yield_target.snapshot.vehicle_id
        self._yield_started_sec = now_sec
        return self._yield_result(base_speed_mps, yield_target, "yield_to_rear")

    def _compute_catchup_wait(
        self,
        relations: List[_RaceRelation],
        front_target: Optional[_RaceRelation],
        now_sec: float,
        base_speed_mps: float,
    ) -> Optional[V2XRaceBehaviorResult]:
        active_relation = self._relation_by_id(relations, self._catchup_target_id)
        if active_relation is not None:
            behind_gap = self._rear_gap_m(active_relation)
            timed_out = (
                self._catchup_started_sec is not None
                and now_sec - self._catchup_started_sec >= self._cfg.catchup_timeout_sec
            )
            if behind_gap <= self._cfg.catchup_release_gap_m or timed_out:
                self._catchup_target_id = None
                self._catchup_started_sec = None
                return None
            if self._catchup_candidate_ready(active_relation):
                return self._catchup_result(
                    base_speed_mps,
                    active_relation,
                    "catchup_active",
                )

        catchup_target = self._catchup_target(relations)
        if catchup_target is None:
            self._catchup_target_id = None
            self._catchup_started_sec = None
            return None

        self._catchup_target_id = catchup_target.snapshot.vehicle_id
        self._catchup_started_sec = now_sec
        return self._catchup_result(base_speed_mps, catchup_target, "catchup_wait")

    def _catchup_target(self, relations: List[_RaceRelation]) -> Optional[_RaceRelation]:
        candidates = [
            relation for relation in relations
            if self._catchup_candidate_ready(relation)
            and self._rear_gap_m(relation) >= self._cfg.catchup_start_gap_m
        ]
        if not candidates:
            return None
        return max(candidates, key=self._rear_gap_m)

    def _catchup_candidate_ready(self, relation: _RaceRelation) -> bool:
        if not self._is_rear_relation(relation):
            return False
        behind_gap = self._rear_gap_m(relation)
        if behind_gap > self._cfg.catchup_detect_range_m:
            return False
        if self._effective_lateral_offset_m(relation) > self._cfg.corridor_half_width_m:
            return False
        return True

    def _is_rear_relation(self, relation: _RaceRelation) -> bool:
        return (
            relation.signed_gap_m < 0.0
            or relation.yaw_signed_gap_m < -self._cfg.self_ignore_radius_m
        )

    def _rear_gap_m(self, relation: _RaceRelation) -> float:
        if relation.signed_gap_m < 0.0:
            return abs(relation.signed_gap_m)
        return abs(relation.yaw_signed_gap_m)

    def _release_yield(
        self,
        now_sec: float,
        base_speed_mps: float,
        reason: str,
    ) -> V2XRaceBehaviorResult:
        released_id = self._yield_target_id
        self._yield_target_id = None
        self._yield_started_sec = None
        self._yield_hold_until_sec = now_sec + max(0.0, self._cfg.yield_side_hold_sec)
        return V2XRaceBehaviorResult(
            active=True,
            state=self.COOLDOWN,
            speed_cap_mps=min(base_speed_mps, self._cfg.follow_speed_cap_mps),
            target_lateral_offset_m=0.0,
            reason=reason,
            yield_target_id=released_id,
            cooldown_remaining_sec=max(0.0, self._cfg.yield_side_hold_sec),
        )

    def _should_continue_yield(self, relation: _RaceRelation) -> bool:
        behind_gap = self._rear_gap_m(relation)
        if self._is_rear_relation(relation) and behind_gap <= self._cfg.yield_detect_range_m:
            return True
        if relation.forward_gap_m <= self._cfg.yield_release_gap_m:
            return True
        return False

    def _yield_target(self, relations: List[_RaceRelation]) -> Optional[_RaceRelation]:
        candidates = []
        for relation in relations:
            if not self._is_rear_relation(relation):
                continue
            behind_gap = self._rear_gap_m(relation)
            if behind_gap > self._cfg.yield_detect_range_m:
                continue
            if self._effective_lateral_offset_m(relation) > self._cfg.corridor_half_width_m:
                continue

            closing_speed = relation.relative_speed_mps
            closing_ready = (
                closing_speed is not None
                and closing_speed >= self._cfg.yield_min_closing_speed_mps
            )
            if behind_gap <= self._cfg.yield_start_gap_m or closing_ready:
                candidates.append(relation)

        if not candidates:
            return None
        return min(candidates, key=self._rear_gap_m)

    def _front_target(self, relations: List[_RaceRelation]) -> Optional[_RaceRelation]:
        candidates = []
        for relation in relations:
            front_gap = self._effective_front_gap_m(relation)
            front_lateral = self._effective_lateral_offset_m(relation)
            if front_gap <= 0.0 or front_gap > self._cfg.detection_range_m:
                continue
            in_corridor = front_lateral <= self._cfg.corridor_half_width_m
            near_conflict = (
                front_gap <= self._cfg.front_conflict_gap_m
                and front_lateral <= self._cfg.front_conflict_lateral_window_m
            )
            if in_corridor or near_conflict:
                candidates.append(relation)
        if not candidates:
            return None
        return min(candidates, key=self._effective_front_gap_m)

    def _effective_front_gap_m(self, relation: _RaceRelation) -> float:
        gaps = []
        yaw_clear_rear = relation.yaw_signed_gap_m < -self._cfg.self_ignore_radius_m
        if relation.signed_gap_m > self._cfg.self_ignore_radius_m and not yaw_clear_rear:
            gaps.append(relation.forward_gap_m)
        if relation.yaw_signed_gap_m > self._cfg.self_ignore_radius_m:
            gaps.append(relation.yaw_signed_gap_m)
        if not gaps:
            if yaw_clear_rear:
                return relation.yaw_signed_gap_m
            return relation.forward_gap_m
        return min(gaps)

    def _effective_lateral_offset_m(self, relation: _RaceRelation) -> float:
        return min(relation.lateral_offset_m, abs(relation.yaw_lateral_signed_m))

    def _side_switch_blocked(
        self,
        overtake_result: V2XOvertakeResult,
        current_lateral_offset_m: float,
    ) -> bool:
        target_offset = float(overtake_result.target_lateral_offset_m)
        current_offset = float(current_lateral_offset_m)
        if abs(target_offset) <= 1e-6:
            return False
        if abs(current_offset) <= self._cfg.side_switch_center_threshold_m:
            return False
        return target_offset * current_offset < 0.0

    def _return_blocker(
        self,
        relations: List[_RaceRelation],
        target_id: Optional[str],
    ) -> Optional[_RaceRelation]:
        for relation in relations:
            if relation.snapshot.vehicle_id == target_id:
                continue
            if relation.lateral_offset_m > self._cfg.corridor_half_width_m:
                continue
            if -self._cfg.return_rear_clearance_m <= relation.signed_gap_m <= self._cfg.return_clearance_m:
                return relation
        return None

    def _follow_result(
        self,
        target: _RaceRelation,
        base_speed_mps: float,
        ego_v_mps: float,
        reason: str,
        overtake_result: Optional[V2XOvertakeResult] = None,
    ) -> V2XRaceBehaviorResult:
        front_gap_m = self._effective_front_gap_m(target)
        if front_gap_m > self._cfg.approach_start_gap_m:
            speed_cap = base_speed_mps
            reason = "front_far_chase"
        elif front_gap_m > self._cfg.max_overtake_start_gap_m:
            speed_cap = min(base_speed_mps, self._cfg.approach_speed_cap_mps)
            reason = "front_approach"
        else:
            speed_cap = self._follow_speed_cap(target, front_gap_m, base_speed_mps, ego_v_mps)
        return V2XRaceBehaviorResult(
            active=True,
            state=self.FOLLOW,
            speed_cap_mps=speed_cap,
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id,
            target_id=target.snapshot.vehicle_id,
            target_role="front",
            side=None,
            gap_m=front_gap_m,
            signed_gap_m=target.signed_gap_m,
            yaw_signed_gap_m=target.yaw_signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
            relative_speed_mps=target.relative_speed_mps,
            target_speed_mps=target.target_speed_mps,
            left_wall_margin_m=(
                overtake_result.left_wall_margin_m if overtake_result else None),
            right_wall_margin_m=(
                overtake_result.right_wall_margin_m if overtake_result else None),
        )

    def _follow_speed_cap(
        self,
        target: _RaceRelation,
        front_gap_m: float,
        base_speed_mps: float,
        ego_v_mps: float,
    ) -> float:
        target_speed_mps = target.target_speed_mps
        if target_speed_mps is None:
            gap_after_min = max(front_gap_m - self._cfg.min_follow_gap_m, 0.0)
            braking_cap = math.sqrt(max(0.0, 2.0 * 1.2 * gap_after_min))
            return min(base_speed_mps, self._cfg.follow_speed_cap_mps, braking_cap)

        desired_gap = max(self._cfg.min_follow_gap_m, self._cfg.time_headway_sec * max(ego_v_mps, 0.0))
        gap_error = front_gap_m - desired_gap
        closing_speed = max(ego_v_mps, 0.0) - max(target_speed_mps, 0.0)
        follow_v = (
            target_speed_mps
            + self._cfg.follow_kp_gap * gap_error
            - self._cfg.follow_kd_closing * closing_speed
        )
        return min(
            base_speed_mps,
            self._cfg.follow_speed_cap_mps,
            max(0.0, follow_v),
        )

    def _yield_result(
        self,
        base_speed_mps: float,
        target: _RaceRelation,
        reason: str,
    ) -> V2XRaceBehaviorResult:
        return V2XRaceBehaviorResult(
            active=True,
            state=self.YIELD,
            speed_cap_mps=min(base_speed_mps, self._cfg.yield_speed_cap_mps),
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id,
            target_id=target.snapshot.vehicle_id,
            target_role="rear",
            gap_m=self._rear_gap_m(target),
            signed_gap_m=target.signed_gap_m,
            yaw_signed_gap_m=target.yaw_signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
            relative_speed_mps=target.relative_speed_mps,
            target_speed_mps=target.target_speed_mps,
            yield_active=True,
            yield_target_id=target.snapshot.vehicle_id,
        )

    def _catchup_result(
        self,
        base_speed_mps: float,
        target: _RaceRelation,
        reason: str,
    ) -> V2XRaceBehaviorResult:
        return V2XRaceBehaviorResult(
            active=True,
            state=self.CATCHUP_WAIT,
            speed_cap_mps=min(base_speed_mps, self._cfg.catchup_speed_cap_mps),
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id,
            target_id=target.snapshot.vehicle_id,
            target_role="rear",
            gap_m=self._rear_gap_m(target),
            signed_gap_m=target.signed_gap_m,
            yaw_signed_gap_m=target.yaw_signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
            relative_speed_mps=target.relative_speed_mps,
            target_speed_mps=target.target_speed_mps,
        )

    def _emergency_stop(
        self,
        base_speed_mps: float,
        target: _RaceRelation,
    ) -> V2XRaceBehaviorResult:
        front_gap_m = self._effective_front_gap_m(target)
        return V2XRaceBehaviorResult(
            active=True,
            state=self.ABORT,
            speed_cap_mps=0.0,
            target_lateral_offset_m=0.0,
            reason="emergency_front_gap",
            vehicle_id=target.snapshot.vehicle_id,
            target_id=target.snapshot.vehicle_id,
            target_role="front",
            gap_m=front_gap_m,
            signed_gap_m=target.signed_gap_m,
            yaw_signed_gap_m=target.yaw_signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
        )

    def _cooldown_active(self, now_sec: float) -> bool:
        return self._cooldown_until_sec is not None and now_sec < self._cooldown_until_sec

    def _cooldown_result(
        self,
        front_target: Optional[_RaceRelation],
        ego_v_mps: float,
        now_sec: float,
        base_speed_mps: float,
        reason: str = "cooldown",
    ) -> V2XRaceBehaviorResult:
        remaining = 0.0
        if self._cooldown_until_sec is not None:
            remaining = max(0.0, self._cooldown_until_sec - now_sec)
        if front_target is not None:
            result = self._follow_result(
                front_target,
                base_speed_mps,
                ego_v_mps,
                reason,
            )
            result.state = self.COOLDOWN
            result.cooldown_remaining_sec = remaining
            return result

        return V2XRaceBehaviorResult(
            active=True,
            state=self.COOLDOWN,
            speed_cap_mps=base_speed_mps,
            target_lateral_offset_m=0.0,
            reason=reason,
            cooldown_remaining_sec=remaining,
        )

    def _from_overtake_result(
        self,
        result: V2XOvertakeResult,
        state: Optional[str] = None,
        speed_cap_mps: Optional[float] = None,
        target_lateral_offset_m: Optional[float] = None,
        reason: Optional[str] = None,
    ) -> V2XRaceBehaviorResult:
        target_role = None
        if result.state in (self.FOLLOW, self.PREPARE, self.OVERTAKING, self.RETURN):
            target_role = "front"
        return V2XRaceBehaviorResult(
            active=result.active,
            state=state or result.state,
            speed_cap_mps=result.speed_cap_mps if speed_cap_mps is None else speed_cap_mps,
            target_lateral_offset_m=(
                result.target_lateral_offset_m
                if target_lateral_offset_m is None
                else target_lateral_offset_m
            ),
            reason=reason or result.reason,
            vehicle_id=result.vehicle_id,
            target_id=result.vehicle_id,
            target_role=target_role,
            side=result.side,
            gap_m=result.gap_m,
            signed_gap_m=result.signed_gap_m,
            lateral_offset_m=result.lateral_offset_m,
            relative_speed_mps=result.relative_speed_mps,
            target_speed_mps=result.target_speed_mps,
            left_wall_margin_m=result.left_wall_margin_m,
            right_wall_margin_m=result.right_wall_margin_m,
        )

    def _relation_by_id(
        self,
        relations: List[_RaceRelation],
        vehicle_id: Optional[str],
    ) -> Optional[_RaceRelation]:
        if vehicle_id is None:
            return None
        for relation in relations:
            if relation.snapshot.vehicle_id == vehicle_id:
                return relation
        return None

    def _collect_relations(
        self,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_v_mps: float,
        path_xy: Sequence[Tuple[float, float]],
        now_sec: float,
        velocity_lookup: Optional[Callable[[str], Tuple[float, float]]],
    ) -> List[_RaceRelation]:
        relations: List[_RaceRelation] = []
        cum_s: List[float] = []
        total_s = 0.0
        ego_projection = None
        if len(path_xy) >= 2:
            cum_s, total_s = _cumulative_distance(path_xy)
            ego_projection = _project_to_path(
                ego_x,
                ego_y,
                path_xy,
                cum_s,
                self._cfg.circular_path,
            )

        for snapshot in self._snapshots.values():
            age = now_sec - snapshot.stamp_sec
            if age < -1e-6 or age > self._cfg.stale_timeout_sec:
                continue

            dx = snapshot.x - ego_x
            dy = snapshot.y - ego_y
            if math.hypot(dx, dy) <= self._cfg.self_ignore_radius_m:
                continue

            yaw_forward_gap, yaw_signed_gap, yaw_lateral_signed = _yaw_relation(dx, dy, ego_yaw)

            if ego_projection is not None:
                target_projection = _project_to_path(
                    snapshot.x,
                    snapshot.y,
                    path_xy,
                    cum_s,
                    self._cfg.circular_path,
                )
                if target_projection is None:
                    forward_gap_m = yaw_forward_gap
                    signed_gap_m = yaw_signed_gap
                    lateral_signed_m = yaw_lateral_signed
                    path_index = 0
                else:
                    signed_gap_m = _signed_gap(
                        target_projection.s_m,
                        ego_projection.s_m,
                        total_s,
                        self._cfg.circular_path,
                    )
                    forward_gap_m = signed_gap_m
                    if self._cfg.circular_path and forward_gap_m <= 0.0:
                        forward_gap_m += total_s
                    lateral_signed_m = target_projection.lateral_signed_m
                    path_index = target_projection.path_index
                    if (
                        not self._cfg.circular_path
                        and target_projection.s_m <= 1e-6
                        and yaw_signed_gap < 0.0
                    ):
                        forward_gap_m = yaw_forward_gap
                        signed_gap_m = yaw_signed_gap
                        lateral_signed_m = yaw_lateral_signed
            else:
                forward_gap_m = yaw_forward_gap
                signed_gap_m = yaw_signed_gap
                lateral_signed_m = yaw_lateral_signed
                path_index = 0

            relative_speed_mps = None
            target_speed_mps = None
            if velocity_lookup is not None:
                vx, vy = velocity_lookup(snapshot.vehicle_id)
                forward_x = math.cos(ego_yaw)
                forward_y = math.sin(ego_yaw)
                target_forward_speed = vx * forward_x + vy * forward_y
                relative_speed_mps = target_forward_speed - ego_v_mps
                target_speed_mps = math.hypot(vx, vy)

            relations.append(_RaceRelation(
                snapshot=snapshot,
                forward_gap_m=forward_gap_m,
                signed_gap_m=signed_gap_m,
                lateral_signed_m=lateral_signed_m,
                lateral_offset_m=abs(lateral_signed_m),
                yaw_lateral_signed_m=yaw_lateral_signed,
                path_index=path_index,
                yaw_signed_gap_m=yaw_signed_gap,
                relative_speed_mps=relative_speed_mps,
                target_speed_mps=target_speed_mps,
            ))

        return relations


def _signed_gap(
    target_s: float,
    ego_s: float,
    total_s: float,
    circular_path: bool,
) -> float:
    signed_gap = target_s - ego_s
    if circular_path and total_s > 1e-6:
        half_s = total_s * 0.5
        if signed_gap > half_s:
            signed_gap -= total_s
        elif signed_gap < -half_s:
            signed_gap += total_s
    return signed_gap


def _project_to_path(
    x: float,
    y: float,
    path_xy: Sequence[Tuple[float, float]],
    cum_s: Sequence[float],
    circular_path: bool,
) -> Optional[_Projection]:
    best_s = 0.0
    best_lateral = 0.0
    best_idx = 0
    best_dist_sq = float("inf")
    n_points = len(path_xy)
    segment_count = n_points if circular_path else n_points - 1

    for i in range(segment_count):
        x0, y0 = path_xy[i]
        next_i = (i + 1) % n_points
        x1, y1 = path_xy[next_i]
        dx = x1 - x0
        dy = y1 - y0
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-12:
            continue

        t = ((x - x0) * dx + (y - y0) * dy) / seg_len_sq
        t = min(1.0, max(0.0, t))
        proj_x = x0 + t * dx
        proj_y = y0 + t * dy
        dist_sq = (x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y)
        if dist_sq < best_dist_sq:
            seg_len = math.sqrt(seg_len_sq)
            left_x = -dy / seg_len
            left_y = dx / seg_len
            best_s = cum_s[i] + seg_len * t
            best_lateral = (x - proj_x) * left_x + (y - proj_y) * left_y
            best_idx = i
            best_dist_sq = dist_sq

    if not math.isfinite(best_dist_sq):
        return None
    return _Projection(best_s, best_lateral, best_idx)


def _yaw_relation(dx: float, dy: float, ego_yaw: float) -> Tuple[float, float, float]:
    forward_x = math.cos(ego_yaw)
    forward_y = math.sin(ego_yaw)
    left_x = -forward_y
    left_y = forward_x
    gap = dx * forward_x + dy * forward_y
    lateral_signed = dx * left_x + dy * left_y
    return gap, gap, lateral_signed


def _cumulative_distance(path_xy: Sequence[Tuple[float, float]]) -> Tuple[List[float], float]:
    cum_s = [0.0]
    for i in range(1, len(path_xy)):
        x0, y0 = path_xy[i - 1]
        x1, y1 = path_xy[i]
        cum_s.append(cum_s[-1] + math.hypot(x1 - x0, y1 - y0))

    x_last, y_last = path_xy[-1]
    x_first, y_first = path_xy[0]
    total_s = cum_s[-1] + math.hypot(x_first - x_last, y_first - y_last)
    return cum_s, total_s


def _normalize_path(reference_xy) -> List[Tuple[float, float]]:
    if reference_xy is None:
        return []

    out: List[Tuple[float, float]] = []
    for point in reference_xy:
        try:
            x = float(point[0])
            y = float(point[1])
        except (TypeError, IndexError):
            x = float(point.x)
            y = float(point.y)
        if _finite(x) and _finite(y):
            out.append((x, y))
    return out
