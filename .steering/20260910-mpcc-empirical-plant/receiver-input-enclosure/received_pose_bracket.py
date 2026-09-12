"""Offline empirical estimate from values already received at a frozen decision.

No runtime imports, publication authority, fitting, extrapolation or claimed error
bound. Raw capture and original observation remain separate from derived values.
Use only the receiver's immutable history, never a bag receipt as availability.
"""
import math

CHANNELS = (
    ('velocity_u_vy', 'velocity_source_sec', (3, 4)),
    ('imu_body_yaw_rate', 'yaw_rate_source_sec', (5,)),
    ('physical_tire_steering', 'tire_source_sec', (7,)),
)


def reconstruct_at_pose(capture, observation):
    """Return labelled estimates and both raw endpoints; raise on wrong ownership.

A channel without an already received upper endpoint keeps its original held
value and is labelled as such. This does not make that channel time synchronized.
Source stamps newer than the capture's decision epoch are excluded even when
present in its received history. No input is borrowed from a later capture.
"""
    if capture.get('status') != 'valid' or capture.get('authority') is not False:
        raise ValueError('invalid or authoritative diagnostic capture')
    pose = capture['pose_source_sec']
    cutoff = capture['now_sec']
    receipt = capture['captured_steady_ns']
    raw = observation['initial_x_y_yaw_u_vy_r_desired_tire']
    selected = capture['selected_x_y_yaw_u_vy_r_desired_tire']
    if (not all(math.isfinite(x) for x in [pose, cutoff, *raw, *selected]) or
            not 0 <= pose <= cutoff or type(receipt) is not int or receipt < 0 or
            observation['pose_source_sec'] != pose or len(raw) != 8 or len(selected) != 8 or
            any(raw[i] != selected[i] for i in [0, 1, 2, 3, 4, 5, 7])):
        raise ValueError('raw sensor/pose observation does not belong to capture')
    result = list(raw)
    channels = {}
    for i, (name, stamp_key, indices) in enumerate(CHANNELS):
        stamp = capture['selected_component_source_sec'][i]
        if observation[stamp_key] != stamp or not 0 <= stamp <= pose:
            raise ValueError('selected component epoch does not belong to capture')
        history = capture[name]
        if not 1 <= len(history) <= 256:
            raise ValueError('unbounded or missing received history')
        previous = -1
        for row in history:
            if (len(row) != 4 or not all(math.isfinite(x) for x in [row[0], row[2], row[3]]) or
                    not previous < row[0] or row[0] < 0 or type(row[1]) is not int or
                    not 0 <= row[1] <= receipt):
                raise ValueError('invalid source order/value/receiver timestamp')
            previous = row[0]
        left = next((row for row in history if row[0] == stamp), None)
        if left is None or any(left[j + 2] != raw[index] for j, index in enumerate(indices)):
            raise ValueError('selected raw value is missing or changed')
        if any(stamp < row[0] <= pose for row in history):
            raise ValueError('capture contains a different latest-at-pose value')
        right = next((row for row in history if pose <= row[0] <= cutoff), None)
        fraction = None
        status = 'held-no-received-upper-endpoint'
        if right is not None:
            fraction = 0.0 if left[0] == right[0] else (pose - left[0]) / (right[0] - left[0])
            if not 0 <= fraction <= 1:
                raise ValueError('extrapolation is not a bracket')
            status = 'raw-at-pose' if fraction == 0 else 'interpolated-at-pose'
            for j, index in enumerate(indices):
                result[index] = (1 - fraction) * left[j + 2] + fraction * right[j + 2]
        channels[name] = dict(status=status, left=list(left), right=None if right is None else list(right),
                              fraction=fraction, derived_values=[result[index] for index in indices])
    return dict(schema='mpcc-received-pose-bracket-diagnostic/v1', authority=False,
                decision_id=capture['control_decision_id'], effective_pose_source_sec=pose,
                availability_cutoff_sec=cutoff, captured_steady_ns=receipt,
                raw_initial_x_y_yaw_u_vy_r_desired_tire=list(raw),
                derived_initial_x_y_yaw_u_vy_r_desired_tire=result, channels=channels,
                scope='Empirical interpolation only; no error enclosure or certified state. Desired steering remains the original observation value.')
