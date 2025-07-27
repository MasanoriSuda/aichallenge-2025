import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class Waypoint:
    def __init__(self, x, y, yaw=0.0, kappa=0.0, v_ref=0.0):
        self.x = x
        self.y = y
        self.psi = yaw
        self.kappa = kappa
        self.v_ref = v_ref

    def __sub__(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class ReferencePath:
    def __init__(self, map_obj, wp_x, wp_y, path_resolution, smoothing_distance, max_width, circular=False):
        self.map = map_obj
        self.wp_x = wp_x
        self.wp_y = wp_y
        self.resolution = path_resolution
        self.smoothing_distance = smoothing_distance
        self.max_width = max_width
        self.circular = circular

        self.waypoints = self._construct_path(wp_x, wp_y)
        self.length = len(self.waypoints)
        self.segment_lengths = self._compute_segment_lengths()
        self.v_profile = None
        self.kappa_profile = None  # Optional curvature profile

    def _construct_path(self, wp_x, wp_y):
        distance = np.cumsum(np.sqrt(np.diff(wp_x, prepend=wp_x[0])**2 + np.diff(wp_y, prepend=wp_y[0])**2))
        s_interp = np.linspace(distance[0], distance[-1], int((distance[-1] - distance[0]) / self.resolution))
        fx = interp1d(distance, wp_x, kind='linear')
        fy = interp1d(distance, wp_y, kind='linear')
        x_interp = fx(s_interp)
        y_interp = fy(s_interp)

        waypoints = []
        for i in range(len(x_interp)):
            if i == len(x_interp) - 1:
                yaw = np.arctan2(y_interp[i] - y_interp[i - 1], x_interp[i] - x_interp[i - 1])
            else:
                yaw = np.arctan2(y_interp[i + 1] - y_interp[i], x_interp[i + 1] - x_interp[i])
            waypoints.append(Waypoint(x_interp[i], y_interp[i], yaw))
        return waypoints

    def _compute_segment_lengths(self):
        lengths = [0.0]
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i].x - self.waypoints[i - 1].x
            dy = self.waypoints[i].y - self.waypoints[i - 1].y
            lengths.append(np.sqrt(dx ** 2 + dy ** 2))
        return lengths

    def get_speed(self, s):
        if self.v_profile is None:
            return 0.0
        if s < 0:
            return self.v_profile[0]
        if s > self.length:
            return self.v_profile[-1]
        idx = np.linspace(0, self.length, len(self.v_profile))
        return float(np.interp(s, idx, self.v_profile))

    def get_curvature(self, s):
        if self.kappa_profile is None:
            return 0.0
        if s < 0:
            return self.kappa_profile[0]
        if s > self.length:
            return self.kappa_profile[-1]
        idx = np.linspace(0, self.length, len(self.kappa_profile))
        return float(np.interp(s, idx, self.kappa_profile))

    def show(self):
        x = [wp.x for wp in self.waypoints]
        y = [wp.y for wp in self.waypoints]
        plt.plot(x, y, 'g--', linewidth=1)
        plt.plot(x[0], y[0], 'go')
        plt.plot(x[-1], y[-1], 'ro')

    def get_waypoint(self, idx):
        """インデックスで waypoint を取得。範囲外は境界にクリップ。"""
        idx = max(0, min(len(self.waypoints) - 1, idx))
        return self.waypoints[idx]

    def update_path_constraints(self, waypoint_idx, horizon, upper_bound, lower_bound):
        """道幅に基づいた制約を horizon 分返す。"""
        horizon = int(horizon) if horizon else 1

        if waypoint_idx >= len(self.waypoints):
            waypoint_idx = len(self.waypoints) - 1

        ub = np.full(horizon, upper_bound)
        lb = np.full(horizon, -lower_bound)
        width = np.full(horizon, upper_bound + lower_bound)
        return ub, lb, width
    def set_speed_profile(self, v_profile):
        self.v_profile = v_profile
        # 必要なら各 Waypoint にも反映
        if len(self.waypoints) == len(v_profile):
            for i in range(len(self.waypoints)):
                self.waypoints[i].v_ref = v_profile[i]