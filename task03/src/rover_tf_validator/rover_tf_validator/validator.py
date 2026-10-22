"""Homework: implement only the two policy functions below.

The transport, TF error adapter, retries, raw-TF audit, and report formatting are
supplied. Run the public tests after each step. No ROS install is needed for them.
"""
import math

from .contracts import Lookup, Observation, report
from .contracts import LookupFailure
from .geometry import apply_transform


def quaternion_is_valid(rotation, tolerance=1e-3):
    """Return bool: exactly four finite components and |norm(q)-1| <= tolerance.

    TODO: Do not normalize an invalid quaternion and silently accept it.
    The supplied AuditedBuffer calls this hook on raw TF *before* insertion.
    None marks the untouched starter and is deliberately not a passing audit.
    """
    if len(rotation) != 4:
        return False
    if not all(math.isfinite(c) for c in rotation):
        return False
    norm = math.sqrt(sum(c * c for c in rotation))
    return abs(norm - 1.0) <= tolerance


def validate_observation(observation: Observation, lookup: Lookup):
    """Return the report dictionary; make one non-blocking lookup attempt.

    TODO:
    - Reject empty/absolute frame IDs, non-positive stamp_ns, non-finite points.
    - Call lookup(target_frame, source_frame, measurement_stamp_ns).
    - Preserve LookupFailure.status/detail; the supplied queue retries it.
    - Validate returned transform numbers and quaternion before applying it.
    - Apply the transform (geometry.apply_transform is supplied), then report OK.

    Never substitute latest (stamp=0), current time, or an identity transform.
    This function must return quickly: /tf and /clock share the executor.
    """
    for frame in (observation.source_frame, observation.target_frame):
        if not frame or frame.startswith('/'):
            return report(observation, 'INVALID_INPUT', f'bad frame id: {frame!r}')
    if observation.stamp_ns <= 0:
        return report(observation, 'INVALID_INPUT', 'stamp_ns must be positive')
    if not all(math.isfinite(v) for v in observation.point):
        return report(observation, 'INVALID_INPUT', 'point is not finite')

    try:
        transform = lookup(observation.target_frame, observation.source_frame, observation.stamp_ns)
    except LookupFailure as error:
        return report(observation, error.status, error.detail)

    if not all(math.isfinite(v) for v in transform.translation):
        return report(observation, 'INVALID_INPUT', 'transform translation is not finite')
    if not quaternion_is_valid(transform.rotation):
        return report(observation, 'INVALID_INPUT', 'transform quaternion is not unit length')

    return report(observation, 'OK', point=apply_transform(transform, observation.point))
