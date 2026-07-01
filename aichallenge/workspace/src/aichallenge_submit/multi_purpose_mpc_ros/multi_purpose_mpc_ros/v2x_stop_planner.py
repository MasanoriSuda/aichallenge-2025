"""V2X-based longitudinal stop planner for Gate1.

The planner is deliberately ROS-free. It accepts duck-typed messages whose
attributes match ``v2x_msgs/V2XVehiclePositionArray`` and returns a speed cap
that the ROS controller can merge into MPC ``v_max``.
"""

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple


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
class V2XStopConfig:
    detection_range_m: float = 35.0
    corridor_half_width_m: float = 2.0
    self_ignore_radius_m: float = 0.75
    target_stop_gap_m: float = 3.0
    stop_hold_gap_m: float = 3.5
    release_gap_m: float = 5.0
    comfortable_decel_mps2: float = 1.2
    stale_timeout_sec: float = 2.0
    max_speed_cap_mps: float = 30.0 / 3.6
    circular_path: bool = True

    @classmethod
    def from_config(cls, cfg, a_min_mps2: float) -> "V2XStopConfig":
        max_speed_cap_kmph = float(_get_attr(cfg, "max_speed_cap_kmph", 30.0))
        decel = float(_get_attr(cfg, "comfortable_decel_mps2", 1.4))
        max_decel = abs(float(a_min_mps2))
        if max_decel > 0.0:
            decel = min(decel, max_decel)

        return cls(
            detection_range_m=float(_get_attr(cfg, "detection_range_m", 35.0)),
            corridor_half_width_m=float(_get_attr(cfg, "corridor_half_width_m", 2.0)),
            self_ignore_radius_m=float(_get_attr(cfg, "self_ignore_radius_m", 0.75)),
            target_stop_gap_m=float(_get_attr(cfg, "target_stop_gap_m", 3.0)),
            stop_hold_gap_m=float(_get_attr(cfg, "stop_hold_gap_m", 3.5)),
            release_gap_m=float(_get_attr(cfg, "release_gap_m", 5.0)),
            comfortable_decel_mps2=max(decel, 0.1),
            stale_timeout_sec=float(_get_attr(cfg, "stale_timeout_sec", 2.0)),
            max_speed_cap_mps=max_speed_cap_kmph / 3.6,
            circular_path=bool(_get_attr(cfg, "circular_path", True)),
        )


@dataclass
class V2XStopResult:
    active: bool
    holding_stop: bool
    speed_cap_mps: float
    reason: str
    vehicle_id: Optional[str] = None
    gap_m: Optional[float] = None
    lateral_offset_m: Optional[float] = None
    relative_speed_mps: Optional[float] = None


@dataclass
class _Snapshot:
    vehicle_id: str
    x: float
    y: float
    stamp_sec: float


@dataclass
class _Candidate:
    snapshot: _Snapshot
    gap_m: float
    lateral_offset_m: float
    relative_speed_mps: Optional[float]


class V2XStopPlanner:
    def __init__(self, config: V2XStopConfig):
        self._cfg = config
        self._snapshots: Dict[str, _Snapshot] = {}
        self._holding_stop = False
        self._last_vehicle_id: Optional[str] = None

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

    def compute_speed_cap(
        self,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_v_mps: float,
        reference_xy,
        now_sec: float,
        base_speed_mps: float,
        velocity_lookup: Optional[Callable[[str], Tuple[float, float]]] = None,
    ) -> V2XStopResult:
        base_speed_mps = max(0.0, float(base_speed_mps))
        path_xy = self._normalize_path(reference_xy)
        candidates = self._collect_candidates(
            ego_x=ego_x,
            ego_y=ego_y,
            ego_yaw=ego_yaw,
            ego_v_mps=ego_v_mps,
            path_xy=path_xy,
            now_sec=now_sec,
            velocity_lookup=velocity_lookup,
        )

        if not candidates:
            return self._empty_candidate_result(base_speed_mps, now_sec)

        selected = min(candidates, key=lambda c: c.gap_m)
        holding = self._should_hold_stop(selected, ego_v_mps)
        self._holding_stop = holding
        self._last_vehicle_id = selected.snapshot.vehicle_id

        if holding:
            speed_cap_mps = 0.0
            reason = "holding_stop"
        else:
            available_gap = max(
                selected.gap_m - self._cfg.target_stop_gap_m, 0.0)
            speed_cap_mps = math.sqrt(
                2.0 * self._cfg.comfortable_decel_mps2 * available_gap)
            speed_cap_mps = min(
                speed_cap_mps,
                self._cfg.max_speed_cap_mps,
                base_speed_mps,
            )
            reason = "braking"

        return V2XStopResult(
            active=True,
            holding_stop=holding,
            speed_cap_mps=speed_cap_mps,
            reason=reason,
            vehicle_id=selected.snapshot.vehicle_id,
            gap_m=selected.gap_m,
            lateral_offset_m=selected.lateral_offset_m,
            relative_speed_mps=selected.relative_speed_mps,
        )

    def _should_hold_stop(self, candidate: _Candidate, ego_v_mps: float) -> bool:
        if candidate.gap_m <= self._cfg.target_stop_gap_m:
            return True

        if ego_v_mps <= 0.2 and candidate.gap_m <= self._cfg.stop_hold_gap_m:
            return True

        if (
            self._holding_stop
            and self._last_vehicle_id == candidate.snapshot.vehicle_id
            and candidate.gap_m < self._cfg.release_gap_m
        ):
            return True

        return False

    def _empty_candidate_result(
        self,
        base_speed_mps: float,
        now_sec: float,
    ) -> V2XStopResult:
        snapshot = (
            self._snapshots.get(self._last_vehicle_id)
            if self._last_vehicle_id is not None
            else None
        )
        if (
            self._holding_stop
            and snapshot is not None
            and now_sec - snapshot.stamp_sec <= self._cfg.stale_timeout_sec
        ):
            return V2XStopResult(
                active=True,
                holding_stop=True,
                speed_cap_mps=0.0,
                reason="holding_stop",
                vehicle_id=snapshot.vehicle_id,
            )

        self._holding_stop = False
        self._last_vehicle_id = None
        return V2XStopResult(
            active=False,
            holding_stop=False,
            speed_cap_mps=base_speed_mps,
            reason="clear",
        )

    def _collect_candidates(
        self,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_v_mps: float,
        path_xy: Sequence[Tuple[float, float]],
        now_sec: float,
        velocity_lookup: Optional[Callable[[str], Tuple[float, float]]],
    ) -> List[_Candidate]:
        candidates: List[_Candidate] = []
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
                if relation is not None:
                    gap_m, lateral_offset_m = relation
                else:
                    gap_m, lateral_offset_m = self._yaw_relation(dx, dy, ego_yaw)
            else:
                gap_m, lateral_offset_m = self._yaw_relation(dx, dy, ego_yaw)

            if gap_m <= 0.0 or gap_m > self._cfg.detection_range_m:
                continue
            if lateral_offset_m > self._cfg.corridor_half_width_m:
                continue

            relative_speed_mps = None
            if velocity_lookup is not None:
                vx, vy = velocity_lookup(snapshot.vehicle_id)
                forward_x = math.cos(ego_yaw)
                forward_y = math.sin(ego_yaw)
                relative_speed_mps = vx * forward_x + vy * forward_y - ego_v_mps

            candidates.append(_Candidate(
                snapshot=snapshot,
                gap_m=gap_m,
                lateral_offset_m=lateral_offset_m,
                relative_speed_mps=relative_speed_mps,
            ))

        return candidates

    def _path_relation(
        self,
        ego_x: float,
        ego_y: float,
        snapshot: _Snapshot,
        path_xy: Sequence[Tuple[float, float]],
    ) -> Optional[Tuple[float, float]]:
        if len(path_xy) < 2:
            return None

        cum_s, total_s = self._cumulative_distance(path_xy)
        ego_projection = self._project_to_path(ego_x, ego_y, path_xy, cum_s)
        target_projection = self._project_to_path(snapshot.x, snapshot.y, path_xy, cum_s)
        if ego_projection is None or target_projection is None:
            return None

        ego_s, _ = ego_projection
        target_s, lateral_offset = target_projection
        gap = target_s - ego_s
        if self._cfg.circular_path and gap <= 0.0:
            gap += total_s

        return gap, lateral_offset

    def _project_to_path(
        self,
        x: float,
        y: float,
        path_xy: Sequence[Tuple[float, float]],
        cum_s: Sequence[float],
    ) -> Optional[Tuple[float, float]]:
        best_s = 0.0
        best_lateral = 0.0
        best_dist_sq = float("inf")
        n_points = len(path_xy)
        segment_count = n_points if self._cfg.circular_path else n_points - 1

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
                segment_s = cum_s[i] + math.sqrt(seg_len_sq) * t
                best_s = segment_s
                best_lateral = math.sqrt(dist_sq)
                best_dist_sq = dist_sq

        if not math.isfinite(best_dist_sq):
            return None
        return best_s, best_lateral

    @staticmethod
    def _yaw_relation(dx: float, dy: float, ego_yaw: float) -> Tuple[float, float]:
        forward_x = math.cos(ego_yaw)
        forward_y = math.sin(ego_yaw)
        left_x = -forward_y
        left_y = forward_x
        gap = dx * forward_x + dy * forward_y
        lateral_offset = abs(dx * left_x + dy * left_y)
        return gap, lateral_offset

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
