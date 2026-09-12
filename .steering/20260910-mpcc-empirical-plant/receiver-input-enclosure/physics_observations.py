"""Read force probes with an explicit pose reference for offline diagnostics.

Rigidbody velocities belong to COM. Rigidbody.position belongs to kart_root.
The native controller model's position belongs to base_link. Callers must name
their pose reference and supply the serialized geometry; no implicit conversion
or future measurement is admitted as a controller prediction.
"""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class PhysicsObservations:
    pose_reference: str
    vehicle_id: int
    raw: np.ndarray
    columns: dict
    times: np.ndarray
    rotations: np.ndarray
    position_unity_xyz: np.ndarray
    yaw_unity_xz: np.ndarray
    com_velocity_u_vy_r: np.ndarray
    com_from_reference_body_xyz: np.ndarray
    tire_after_vehicle_update: np.ndarray

    def field(self, name):
        return self.raw[:, self.columns[name]]

    def vector(self, prefix):
        return self.raw[:, [self.columns[prefix + k] for k in 'xyz']]


@dataclass
class VehicleUpdateBodyState:
    phase: str
    com_velocity_u_vy_r: np.ndarray
    tire: np.ndarray
    tire_supported: np.ndarray


def vehicle_update_body_state(observations, *, phase):
    """Choose the force-probe phase without changing the legacy end-state reader.

    Begin directly records Rigidbody velocity/rotation. It does not record tire:
    infer that value only from the preceding contiguous End row for this vehicle.
    The first row and gaps have unsupported (NaN) tire, never a zero substitute.
    This is a scoring reference, not a received controller observation.
    """
    if phase == 'after_vehicle_update':
        return VehicleUpdateBodyState(
            phase, observations.com_velocity_u_vy_r,
            observations.tire_after_vehicle_update,
            np.isfinite(observations.tire_after_vehicle_update),
        )
    if phase != 'before_vehicle_update':
        raise ValueError('explicit before_vehicle_update or after_vehicle_update required')
    rotation = Rotation.from_quat(observations.raw[:, [
        observations.columns['bq_' + k] for k in 'xyzw'
    ]]).as_matrix()
    local = lambda values: np.einsum('nji,nj->ni', rotation, values)
    velocity = local(observations.vector('bv_'))
    omega = local(observations.vector('bw_'))
    # fixedTimeAsDouble advances by the float fixedDeltaTime. Allow only its
    # double accumulation rounding, not a fraction of a physics step.
    elapsed = np.diff(observations.times)
    dt = observations.field('dt')[1:]
    tolerance = 8 * np.spacing(np.maximum(1., np.abs(observations.times[1:])))
    contiguous = (
        (np.diff(observations.field('tick')) == 1)
        & (np.diff(observations.field('id')) == 0)
        & np.isfinite(dt) & (dt > 0)
        & (np.abs(elapsed - dt) <= tolerance)
    )
    tire = np.full(len(observations.times), np.nan)
    tire[1:] = np.where(contiguous, observations.tire_after_vehicle_update[:-1], np.nan)
    return VehicleUpdateBodyState(
        phase, np.column_stack([velocity[:, 2], -velocity[:, 0], -omega[:, 1]]),
        tire, np.isfinite(tire),
    )


def read_physics(path, *, pose_reference, geometry_path, vehicle_id=None):
    """Preserve raw physics timestamps and expose COM and pose fields separately."""
    if pose_reference not in ('kart_root', 'base_link'):
        raise ValueError('explicit kart_root or base_link reference required')
    geometry = json.loads(Path(geometry_path).read_text())
    offset = np.asarray(geometry['base_link_in_kart_unity_xyz'], dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('invalid serialized base_link offset')
    with np.load(path) as archive:
        raw = archive['values']
        columns = {str(key): i for i, key in enumerate(archive['columns'])}
    identities = np.unique(raw[:, columns['id']])
    if vehicle_id is None:
        if len(identities) != 1:
            raise ValueError('multi-vehicle physics requires vehicle_id')
        vehicle_id = int(identities[0])
    raw = raw[raw[:, columns['id']] == vehicle_id]
    if not len(raw):
        raise ValueError('vehicle_id absent from physics')
    times = raw[:, columns['fixed']]
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('physics times must strictly increase within one vehicle')
    vector = lambda key: raw[:, [columns[key + k] for k in 'xyz']]
    rotation = Rotation.from_quat(raw[:, [columns['eq_' + k] for k in 'xyzw']]).as_matrix()
    local = lambda values: np.einsum('nji,nj->ni', rotation, values)
    position = vector('ep_').copy()
    if pose_reference == 'base_link':
        position += np.einsum('nij,j->ni', rotation, offset)
    velocity = local(vector('ev_'))
    omega = local(vector('ew_'))
    return PhysicsObservations(
        pose_reference, vehicle_id, raw, columns, times, rotation, position,
        np.arctan2(rotation[:, 2, 2], rotation[:, 0, 2]),
        np.column_stack([velocity[:, 2], -velocity[:, 0], -omega[:, 1]]),
        local(vector('com_') - position),
        -raw[:, columns['steer_deg']] * np.pi / 180,
    )
