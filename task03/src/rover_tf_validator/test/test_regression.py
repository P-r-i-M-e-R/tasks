"""My regression: validator must use the measurement stamp, not latest."""
import pytest
from rover_tf_validator.contracts import Observation, RigidTransform, LookupFailure
from rover_tf_validator.validator import validate_observation

SPEED = 0.5             # m/s along x in odom
NEWEST_NS = 12_700_000_000  # newest tf in the buffer
MEAS_NS = 12_500_000_000    # measurement is 0.2 s older (sensor delay)


def moving_lookup(target, source, stamp_ns):
    # fake buffer: base moves along x, history from 10 s up to NEWEST_NS
    if stamp_ns == 0:
        stamp_ns = NEWEST_NS  # what tf2 does for "latest"
    if stamp_ns < 10_000_000_000 or stamp_ns > NEWEST_NS:
        raise LookupFailure('TIME_UNAVAILABLE', 'outside of buffer')
    x = SPEED * stamp_ns / 1e9
    return RigidTransform((x, 0., 0.), (0., 0., 0., 1.))


def test_historical_lookup_differs_from_latest():
    obs = Observation('lidar', 'r/lidar_link', 'r/odom', MEAS_NS, (1., 0., 0.))
    result = validate_observation(obs, moving_lookup)
    assert result['status'] == 'OK'
    right = 1. + SPEED * MEAS_NS / 1e9    # 7.25
    latest = 1. + SPEED * NEWEST_NS / 1e9  # 7.35
    assert result['point'][0] == pytest.approx(right, abs=1e-9)
    # 10 cm off with latest, way more than the 5 mm tolerance
    assert abs(result['point'][0] - latest) > 0.005
    assert result['stamp_ns'] == MEAS_NS


def test_future_stamp_is_not_replaced_with_latest():
    obs = Observation('lidar', 'r/lidar_link', 'r/odom', NEWEST_NS + 1_000_000_000, (1., 0., 0.))
    result = validate_observation(obs, moving_lookup)
    assert result['status'] == 'TIME_UNAVAILABLE'
    assert result['point'] is None
