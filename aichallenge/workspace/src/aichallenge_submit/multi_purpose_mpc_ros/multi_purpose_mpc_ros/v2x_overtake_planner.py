"""V2X-based overtake planner for Gate2.

The planner is intentionally ROS-free. It consumes duck-typed V2X messages and
returns a longitudinal speed cap plus a desired lateral offset. The ROS
controller is responsible for applying that offset to MPC path constraints.
"""

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple


Side = str


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
class V2XOvertakeConfig:
    detection_range_m: float = 35.0
    corridor_half_width_m: float = 2.0
    self_ignore_radius_m: float = 0.75
    min_overtake_start_gap_m: float = 6.0
    max_overtake_start_gap_m: float = 22.0
    abort_gap_m: float = 3.5
    abort_escape_lateral_threshold_m: float = 0.45
    return_clearance_m: float = 8.0
    preferred_side: str = "left"
    side_selection_policy: str = "largest_margin"
    side_margin_tie_threshold_m: float = 0.3
    lateral_offset_m: float = 1.8
    lateral_offset_rate_mps: float = 0.8
    constraint_half_width_m: float = 0.55
    min_lateral_clearance_m: float = 1.2
    min_wall_clearance_m: float = 0.5
    wall_safety_margin_m: float = 0.2
    wall_check_horizon_m: float = 24.0
    vehicle_width_m: float = 2.3
    overtake_speed_cap_mps: float = 18.0 / 3.6
    follow_speed_cap_mps: float = 12.0 / 3.6
    max_overtake_target_speed_mps: float = 10.0 / 3.6
    min_closing_speed_mps: float = 0.3
    min_ttc_sec: float = 0.8
    stale_timeout_sec: float = 2.0
    target_lost_hold_sec: float = 1.2
    circular_path: bool = True
    return_offset_threshold_m: float = 0.2

    @classmethod
    def from_config(
        cls,
        cfg,
        vehicle_width_m: float,
    ) -> "V2XOvertakeConfig":
        preferred_side = str(_get_attr(cfg, "preferred_side", "left")).lower()
        if preferred_side not in ("left", "right"):
            preferred_side = "left"
        side_selection_policy = str(
            _get_attr(cfg, "side_selection_policy", "largest_margin")).lower()
        if side_selection_policy not in ("largest_margin", "preferred"):
            side_selection_policy = "largest_margin"

        return cls(
            detection_range_m=float(_get_attr(cfg, "detection_range_m", 35.0)),
            corridor_half_width_m=float(_get_attr(cfg, "corridor_half_width_m", 2.0)),
            self_ignore_radius_m=float(_get_attr(cfg, "self_ignore_radius_m", 0.75)),
            min_overtake_start_gap_m=float(_get_attr(cfg, "min_overtake_start_gap_m", 6.0)),
            max_overtake_start_gap_m=float(_get_attr(cfg, "max_overtake_start_gap_m", 22.0)),
            abort_gap_m=float(_get_attr(cfg, "abort_gap_m", 3.5)),
            abort_escape_lateral_threshold_m=float(
                _get_attr(cfg, "abort_escape_lateral_threshold_m", 0.45)),
            return_clearance_m=float(_get_attr(cfg, "return_clearance_m", 8.0)),
            preferred_side=preferred_side,
            side_selection_policy=side_selection_policy,
            side_margin_tie_threshold_m=float(_get_attr(cfg, "side_margin_tie_threshold_m", 0.3)),
            lateral_offset_m=float(_get_attr(cfg, "lateral_offset_m", 1.8)),
            lateral_offset_rate_mps=float(_get_attr(cfg, "lateral_offset_rate_mps", 0.8)),
            constraint_half_width_m=float(_get_attr(cfg, "constraint_half_width_m", 0.55)),
            min_lateral_clearance_m=float(_get_attr(cfg, "min_lateral_clearance_m", 1.2)),
            min_wall_clearance_m=float(_get_attr(cfg, "min_wall_clearance_m", 0.5)),
            wall_safety_margin_m=float(_get_attr(cfg, "wall_safety_margin_m", 0.2)),
            wall_check_horizon_m=float(_get_attr(cfg, "wall_check_horizon_m", 24.0)),
            vehicle_width_m=float(_get_attr(cfg, "vehicle_width_m", vehicle_width_m)),
            overtake_speed_cap_mps=float(_get_attr(cfg, "overtake_speed_cap_kmph", 18.0)) / 3.6,
            follow_speed_cap_mps=float(_get_attr(cfg, "follow_speed_cap_kmph", 12.0)) / 3.6,
            max_overtake_target_speed_mps=float(_get_attr(cfg, "max_overtake_target_speed_kmph", 10.0)) / 3.6,
            min_closing_speed_mps=float(_get_attr(cfg, "min_closing_speed_mps", 0.3)),
            min_ttc_sec=float(_get_attr(cfg, "min_ttc_sec", 0.8)),
            stale_timeout_sec=float(_get_attr(cfg, "stale_timeout_sec", 2.0)),
            target_lost_hold_sec=float(_get_attr(cfg, "target_lost_hold_sec", 1.2)),
            circular_path=bool(_get_attr(cfg, "circular_path", True)),
            return_offset_threshold_m=float(_get_attr(cfg, "return_offset_threshold_m", 0.2)),
        )


@dataclass
class V2XOvertakeResult:
    active: bool
    state: str
    speed_cap_mps: float
    target_lateral_offset_m: float
    reason: str
    vehicle_id: Optional[str] = None
    side: Optional[str] = None
    gap_m: Optional[float] = None
    signed_gap_m: Optional[float] = None
    lateral_offset_m: Optional[float] = None
    relative_speed_mps: Optional[float] = None
    target_speed_mps: Optional[float] = None
    left_wall_margin_m: Optional[float] = None
    right_wall_margin_m: Optional[float] = None


@dataclass
class _Snapshot:
    vehicle_id: str
    x: float
    y: float
    stamp_sec: float


@dataclass
class _Relation:
    snapshot: _Snapshot
    forward_gap_m: float
    signed_gap_m: float
    lateral_signed_m: float
    lateral_offset_m: float
    path_index: int
    relative_speed_mps: Optional[float]
    target_speed_mps: Optional[float]


@dataclass
class _SideDecision:
    side: Optional[str]
    reason: str
    left_wall_margin_m: float
    right_wall_margin_m: float


class V2XOvertakePlanner:
    CLEAR = "clear"
    FOLLOW = "follow"
    PREPARE = "prepare_overtake"
    OVERTAKING = "overtaking"
    RETURN = "return_to_line"
    ABORT = "abort"

    def __init__(self, config: V2XOvertakeConfig):
        self._cfg = config
        self._snapshots: Dict[str, _Snapshot] = {}
        self._state = self.CLEAR
        self._target_id: Optional[str] = None
        self._side: Optional[str] = None
        self._target_lost_since_sec: Optional[float] = None

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
    ) -> V2XOvertakeResult:
        base_speed_mps = max(0.0, float(base_speed_mps))
        path_xy = self._normalize_path(reference_xy)
        widths = self._normalize_widths(reference_widths)
        relations = self._collect_relations(
            ego_x=ego_x,
            ego_y=ego_y,
            ego_yaw=ego_yaw,
            ego_v_mps=ego_v_mps,
            path_xy=path_xy,
            now_sec=now_sec,
            velocity_lookup=velocity_lookup,
        )

        active_relation = self._active_target_relation(relations)
        if self._state in (self.PREPARE, self.OVERTAKING, self.RETURN):
            if active_relation is None:
                return self._target_lost_hold_or_abort(base_speed_mps, now_sec)
            self._target_lost_since_sec = None
            return self._compute_active_behavior(
                active_relation,
                relations,
                widths,
                path_xy,
                current_lateral_offset_m,
                base_speed_mps,
            )

        candidates = [
            r for r in relations
            if 0.0 < r.forward_gap_m <= self._cfg.detection_range_m
            and r.lateral_offset_m <= self._cfg.corridor_half_width_m
        ]
        if not candidates:
            self._clear_state()
            return V2XOvertakeResult(
                active=False,
                state=self.CLEAR,
                speed_cap_mps=base_speed_mps,
                target_lateral_offset_m=0.0,
                reason="clear",
            )

        target = min(candidates, key=lambda r: r.forward_gap_m)
        if target.forward_gap_m <= self._cfg.abort_gap_m:
            self._target_id = target.snapshot.vehicle_id
            return self._abort(base_speed_mps, "abort_gap", target)

        decision = self._choose_side(target, relations, widths, path_xy)
        if not self._gap_allows_overtake(target):
            return self._follow(base_speed_mps, target, decision, "gap_or_speed_not_ready")
        if decision.side is None:
            return self._follow(base_speed_mps, target, decision, decision.reason)

        self._state = self.PREPARE
        self._target_id = target.snapshot.vehicle_id
        self._side = decision.side
        self._target_lost_since_sec = None
        return self._overtake_result(
            state=self.PREPARE,
            target=target,
            decision=decision,
            base_speed_mps=base_speed_mps,
            reason=decision.reason,
        )

    def _compute_active_behavior(
        self,
        target: _Relation,
        relations: List[_Relation],
        widths: Sequence[Tuple[float, float]],
        path_xy: Sequence[Tuple[float, float]],
        current_lateral_offset_m: float,
        base_speed_mps: float,
    ) -> V2XOvertakeResult:
        if self._state == self.RETURN:
            if abs(current_lateral_offset_m) <= self._cfg.return_offset_threshold_m:
                self._clear_state()
                return V2XOvertakeResult(
                    active=False,
                    state=self.CLEAR,
                    speed_cap_mps=base_speed_mps,
                    target_lateral_offset_m=0.0,
                    reason="returned",
                )
            return self._return_result(base_speed_mps, target, "returning")

        if (
            target.forward_gap_m <= self._cfg.abort_gap_m
            and not self._has_side_lateral_progress_m(
                current_lateral_offset_m,
                self._cfg.abort_escape_lateral_threshold_m,
            )
        ):
            return self._abort(base_speed_mps, "abort_gap", target)

        if target.signed_gap_m <= -self._cfg.return_clearance_m:
            next_target = self._next_queue_target(relations, target)
            if next_target is not None:
                decision = self._choose_side(
                    next_target,
                    relations,
                    widths,
                    path_xy,
                    self._side,
                )
                if decision.side is not None:
                    self._target_id = next_target.snapshot.vehicle_id
                    self._side = decision.side
                    self._state = self.OVERTAKING
                    return self._overtake_result(
                        state=self._state,
                        target=next_target,
                        decision=decision,
                        base_speed_mps=base_speed_mps,
                        reason=f"queue_target_{decision.reason}",
                    )

            self._state = self.RETURN
            return self._return_result(base_speed_mps, target, "target_passed")

        decision = self._choose_side(target, relations, widths, path_xy, self._side)
        if decision.side is None:
            return self._abort(base_speed_mps, decision.reason, target, decision)

        self._side = decision.side
        if (
            self._state == self.PREPARE
            and self._has_side_lateral_progress(current_lateral_offset_m, 0.75)
        ):
            self._state = self.OVERTAKING

        return self._overtake_result(
            state=self._state,
            target=target,
            decision=decision,
            base_speed_mps=base_speed_mps,
            reason=decision.reason,
        )

    def _gap_allows_overtake(self, target: _Relation) -> bool:
        if target.forward_gap_m < self._cfg.min_overtake_start_gap_m:
            return False
        if target.forward_gap_m > self._cfg.max_overtake_start_gap_m:
            return False

        if target.target_speed_mps is not None:
            target_is_slow = target.target_speed_mps <= self._cfg.max_overtake_target_speed_mps
        else:
            target_is_slow = True

        if target.relative_speed_mps is not None:
            target_is_slow = (
                target_is_slow
                or target.relative_speed_mps <= -self._cfg.min_closing_speed_mps
            )
            if target.relative_speed_mps < -1e-6:
                ttc = target.forward_gap_m / abs(target.relative_speed_mps)
                if ttc < self._cfg.min_ttc_sec:
                    return False

        return target_is_slow

    def _has_side_lateral_progress(
        self,
        current_lateral_offset_m: float,
        ratio: float,
    ) -> bool:
        threshold = abs(self._cfg.lateral_offset_m) * max(0.0, min(float(ratio), 1.0))
        return self._has_side_lateral_progress_m(current_lateral_offset_m, threshold)

    def _has_side_lateral_progress_m(
        self,
        current_lateral_offset_m: float,
        threshold_m: float,
    ) -> bool:
        threshold = max(0.0, float(threshold_m))
        if self._side == "right":
            return current_lateral_offset_m <= -threshold
        if self._side == "left":
            return current_lateral_offset_m >= threshold
        return abs(current_lateral_offset_m) >= threshold

    def _next_queue_target(
        self,
        relations: List[_Relation],
        current_target: _Relation,
    ) -> Optional[_Relation]:
        candidates = [
            relation for relation in relations
            if relation.snapshot.vehicle_id != current_target.snapshot.vehicle_id
            and 0.0 < relation.forward_gap_m <= self._cfg.detection_range_m
            and relation.lateral_offset_m <= self._cfg.corridor_half_width_m
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda relation: relation.forward_gap_m)

    def _choose_side(
        self,
        target: _Relation,
        relations: List[_Relation],
        widths: Sequence[Tuple[float, float]],
        path_xy: Sequence[Tuple[float, float]],
        forced_side: Optional[str] = None,
    ) -> _SideDecision:
        left_margin, right_margin = self._wall_margins(target.path_index, widths, path_xy)
        left_safe = left_margin >= 0.0 and self._side_v2x_safe("left", target, relations)
        right_safe = right_margin >= 0.0 and self._side_v2x_safe("right", target, relations)

        if forced_side == "left":
            if left_safe:
                return _SideDecision("left", "left_forced_clear", left_margin, right_margin)
            return _SideDecision(None, "forced_left_unsafe", left_margin, right_margin)
        if forced_side == "right":
            if right_safe:
                return _SideDecision("right", "right_forced_clear", left_margin, right_margin)
            return _SideDecision(None, "forced_right_unsafe", left_margin, right_margin)

        preferred = self._cfg.preferred_side

        if left_safe and not right_safe:
            return _SideDecision("left", "left_only_clear", left_margin, right_margin)
        if right_safe and not left_safe:
            return _SideDecision("right", "right_only_clear", left_margin, right_margin)
        if not left_safe and not right_safe:
            return _SideDecision(None, "no_safe_side", left_margin, right_margin)

        if self._cfg.side_selection_policy == "largest_margin":
            margin_diff = left_margin - right_margin
            if abs(margin_diff) <= self._cfg.side_margin_tie_threshold_m:
                return _SideDecision(
                    preferred,
                    f"{preferred}_preferred_tie",
                    left_margin,
                    right_margin,
                )
            side = "left" if left_margin >= right_margin else "right"
            return _SideDecision(side, f"{side}_larger_wall_margin", left_margin, right_margin)

        return _SideDecision(preferred, f"{preferred}_preferred", left_margin, right_margin)

    def _wall_margins(
        self,
        start_idx: int,
        widths: Sequence[Tuple[float, float]],
        path_xy: Sequence[Tuple[float, float]],
    ) -> Tuple[float, float]:
        if not widths:
            return -float("inf"), -float("inf")

        horizon_indices = self._horizon_indices(start_idx, path_xy, len(widths))
        left_available = min(widths[i][0] for i in horizon_indices)
        right_available = min(widths[i][1] for i in horizon_indices)
        required = (
            abs(self._cfg.lateral_offset_m)
            + self._cfg.vehicle_width_m * 0.5
            + self._cfg.wall_safety_margin_m
            + self._cfg.min_wall_clearance_m
        )
        return left_available - required, right_available - required

    def _side_v2x_safe(
        self,
        side: str,
        target: _Relation,
        relations: List[_Relation],
    ) -> bool:
        side_sign = 1.0 if side == "left" else -1.0
        desired_lateral = side_sign * self._cfg.lateral_offset_m
        for relation in relations:
            if relation.snapshot.vehicle_id == target.snapshot.vehicle_id:
                continue
            if relation.signed_gap_m < -3.0:
                continue
            if relation.signed_gap_m > target.forward_gap_m + self._cfg.return_clearance_m:
                continue
            if abs(relation.lateral_signed_m - desired_lateral) < self._cfg.min_lateral_clearance_m:
                return False
        return True

    def _horizon_indices(
        self,
        start_idx: int,
        path_xy: Sequence[Tuple[float, float]],
        n_widths: int,
    ) -> List[int]:
        if not path_xy or len(path_xy) < 2:
            return [min(max(start_idx, 0), n_widths - 1)]

        indices = []
        dist = 0.0
        idx = start_idx % n_widths
        prev = path_xy[idx % len(path_xy)]
        while dist <= self._cfg.wall_check_horizon_m and len(indices) < n_widths:
            indices.append(idx)
            next_idx = (idx + 1) % n_widths if self._cfg.circular_path else idx + 1
            if next_idx >= n_widths:
                break
            curr = path_xy[next_idx % len(path_xy)]
            dist += math.hypot(curr[0] - prev[0], curr[1] - prev[1])
            prev = curr
            idx = next_idx
        return indices or [min(max(start_idx, 0), n_widths - 1)]

    def _follow(
        self,
        base_speed_mps: float,
        target: _Relation,
        decision: _SideDecision,
        reason: str,
    ) -> V2XOvertakeResult:
        self._state = self.FOLLOW
        self._target_id = target.snapshot.vehicle_id
        self._side = None
        return V2XOvertakeResult(
            active=True,
            state=self.FOLLOW,
            speed_cap_mps=min(base_speed_mps, self._cfg.follow_speed_cap_mps),
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id,
            gap_m=target.forward_gap_m,
            signed_gap_m=target.signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
            relative_speed_mps=target.relative_speed_mps,
            target_speed_mps=target.target_speed_mps,
            left_wall_margin_m=decision.left_wall_margin_m,
            right_wall_margin_m=decision.right_wall_margin_m,
        )

    def _abort(
        self,
        base_speed_mps: float,
        reason: str,
        target: Optional[_Relation] = None,
        decision: Optional[_SideDecision] = None,
    ) -> V2XOvertakeResult:
        self._state = self.ABORT
        self._side = None
        self._target_lost_since_sec = None
        speed_cap = 0.0 if reason == "abort_gap" else min(base_speed_mps, self._cfg.follow_speed_cap_mps)
        return V2XOvertakeResult(
            active=True,
            state=self.ABORT,
            speed_cap_mps=speed_cap,
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id if target else self._target_id,
            gap_m=target.forward_gap_m if target else None,
            signed_gap_m=target.signed_gap_m if target else None,
            lateral_offset_m=target.lateral_offset_m if target else None,
            relative_speed_mps=target.relative_speed_mps if target else None,
            target_speed_mps=target.target_speed_mps if target else None,
            left_wall_margin_m=decision.left_wall_margin_m if decision else None,
            right_wall_margin_m=decision.right_wall_margin_m if decision else None,
        )

    def _target_lost_hold_or_abort(
        self,
        base_speed_mps: float,
        now_sec: float,
    ) -> V2XOvertakeResult:
        if self._target_lost_since_sec is None:
            self._target_lost_since_sec = now_sec

        lost_duration = now_sec - self._target_lost_since_sec
        if (
            self._side is not None
            and lost_duration <= self._cfg.target_lost_hold_sec
        ):
            offset_sign = 1.0 if self._side == "left" else -1.0
            target_offset = 0.0 if self._state == self.RETURN else offset_sign * self._cfg.lateral_offset_m
            return V2XOvertakeResult(
                active=True,
                state=self._state,
                speed_cap_mps=min(base_speed_mps, self._cfg.follow_speed_cap_mps),
                target_lateral_offset_m=target_offset,
                reason="target_lost_hold",
                vehicle_id=self._target_id,
                side=self._side,
            )

        return self._abort(base_speed_mps, "target_lost")

    def _overtake_result(
        self,
        state: str,
        target: _Relation,
        decision: _SideDecision,
        base_speed_mps: float,
        reason: str,
    ) -> V2XOvertakeResult:
        side = decision.side or self._side or self._cfg.preferred_side
        offset_sign = 1.0 if side == "left" else -1.0
        return V2XOvertakeResult(
            active=True,
            state=state,
            speed_cap_mps=min(base_speed_mps, self._cfg.overtake_speed_cap_mps),
            target_lateral_offset_m=offset_sign * self._cfg.lateral_offset_m,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id,
            side=side,
            gap_m=target.forward_gap_m,
            signed_gap_m=target.signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
            relative_speed_mps=target.relative_speed_mps,
            target_speed_mps=target.target_speed_mps,
            left_wall_margin_m=decision.left_wall_margin_m,
            right_wall_margin_m=decision.right_wall_margin_m,
        )

    def _return_result(
        self,
        base_speed_mps: float,
        target: _Relation,
        reason: str,
    ) -> V2XOvertakeResult:
        return V2XOvertakeResult(
            active=True,
            state=self.RETURN,
            speed_cap_mps=min(base_speed_mps, self._cfg.overtake_speed_cap_mps),
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=target.snapshot.vehicle_id,
            side=self._side,
            gap_m=target.forward_gap_m,
            signed_gap_m=target.signed_gap_m,
            lateral_offset_m=target.lateral_offset_m,
            relative_speed_mps=target.relative_speed_mps,
            target_speed_mps=target.target_speed_mps,
        )

    def _clear_state(self) -> None:
        self._state = self.CLEAR
        self._target_id = None
        self._side = None
        self._target_lost_since_sec = None

    def _active_target_relation(self, relations: List[_Relation]) -> Optional[_Relation]:
        if self._target_id is None:
            return None
        for relation in relations:
            if relation.snapshot.vehicle_id == self._target_id:
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
    ) -> List[_Relation]:
        relations: List[_Relation] = []
        for snapshot in self._snapshots.values():
            age = now_sec - snapshot.stamp_sec
            if age < -1e-6 or age > self._cfg.stale_timeout_sec:
                continue

            dx = snapshot.x - ego_x
            dy = snapshot.y - ego_y
            if math.hypot(dx, dy) <= self._cfg.self_ignore_radius_m:
                continue

            if path_xy:
                relation = self._path_relation(ego_x, ego_y, snapshot, path_xy)
                if relation is None:
                    continue
                forward_gap_m, signed_gap_m, lateral_signed_m, path_index = relation
            else:
                forward_gap_m, signed_gap_m, lateral_signed_m = self._yaw_relation(dx, dy, ego_yaw)
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

            relations.append(_Relation(
                snapshot=snapshot,
                forward_gap_m=forward_gap_m,
                signed_gap_m=signed_gap_m,
                lateral_signed_m=lateral_signed_m,
                lateral_offset_m=abs(lateral_signed_m),
                path_index=path_index,
                relative_speed_mps=relative_speed_mps,
                target_speed_mps=target_speed_mps,
            ))

        return relations

    def _path_relation(
        self,
        ego_x: float,
        ego_y: float,
        snapshot: _Snapshot,
        path_xy: Sequence[Tuple[float, float]],
    ) -> Optional[Tuple[float, float, float, int]]:
        if len(path_xy) < 2:
            return None

        ego_idx = self._nearest_index(ego_x, ego_y, path_xy)
        target_idx = self._nearest_index(snapshot.x, snapshot.y, path_xy)
        cum_s, total_s = self._cumulative_distance(path_xy)
        signed_gap = cum_s[target_idx] - cum_s[ego_idx]
        forward_gap = signed_gap
        if self._cfg.circular_path and forward_gap <= 0.0:
            forward_gap += total_s

        left_x, left_y = self._left_normal(target_idx, path_xy)
        lateral_signed = (
            (snapshot.x - path_xy[target_idx][0]) * left_x
            + (snapshot.y - path_xy[target_idx][1]) * left_y
        )
        return forward_gap, signed_gap, lateral_signed, target_idx

    @staticmethod
    def _yaw_relation(dx: float, dy: float, ego_yaw: float) -> Tuple[float, float, float]:
        forward_x = math.cos(ego_yaw)
        forward_y = math.sin(ego_yaw)
        left_x = -forward_y
        left_y = forward_x
        gap = dx * forward_x + dy * forward_y
        lateral_signed = dx * left_x + dy * left_y
        return gap, gap, lateral_signed

    @staticmethod
    def _nearest_index(
        x: float,
        y: float,
        path_xy: Sequence[Tuple[float, float]],
    ) -> int:
        best_idx = 0
        best_dist_sq = float("inf")
        for i, (px, py) in enumerate(path_xy):
            dist_sq = (px - x) * (px - x) + (py - y) * (py - y)
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i
        return best_idx

    @staticmethod
    def _left_normal(idx: int, path_xy: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
        if idx < len(path_xy) - 1:
            x0, y0 = path_xy[idx]
            x1, y1 = path_xy[idx + 1]
        else:
            x0, y0 = path_xy[idx - 1]
            x1, y1 = path_xy[idx]
        dx = x1 - x0
        dy = y1 - y0
        norm = math.hypot(dx, dy)
        if norm <= 1e-9:
            return 0.0, 1.0
        return -dy / norm, dx / norm

    @staticmethod
    def _cumulative_distance(
        path_xy: Sequence[Tuple[float, float]],
    ) -> Tuple[List[float], float]:
        cum_s = [0.0]
        for i in range(1, len(path_xy)):
            x0, y0 = path_xy[i - 1]
            x1, y1 = path_xy[i]
            cum_s.append(cum_s[-1] + math.hypot(x1 - x0, y1 - y0))

        x_last, y_last = path_xy[-1]
        x_first, y_first = path_xy[0]
        total_s = cum_s[-1] + math.hypot(x_first - x_last, y_first - y_last)
        return cum_s, total_s

    @staticmethod
    def _normalize_path(reference_xy) -> List[Tuple[float, float]]:
        if reference_xy is None:
            return []

        out: List[Tuple[float, float]] = []
        for point in reference_xy:
            try:
                x = float(point[0])
                y = float(point[1])
            except (TypeError, IndexError):
                x = float(getattr(point, "x"))
                y = float(getattr(point, "y"))
            if _finite(x) and _finite(y):
                out.append((x, y))
        return out

    @staticmethod
    def _normalize_widths(reference_widths) -> List[Tuple[float, float]]:
        if reference_widths is None:
            return []

        out: List[Tuple[float, float]] = []
        for item in reference_widths:
            try:
                left = float(item[0])
                right = float(item[1])
            except (TypeError, IndexError):
                left = float(getattr(item, "left"))
                right = float(getattr(item, "right"))
            if _finite(left) and _finite(right):
                out.append((max(left, 0.0), max(right, 0.0)))
        return out
