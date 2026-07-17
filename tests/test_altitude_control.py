import numpy as np
import pytest

from flight_dynamics.altitude_control import (
    LookaheadAltitudeController,
    flight_path_angle_from_state,
    lookahead_flight_path_angle_command,
)
from flight_dynamics.integrators import RK4_controlled


def level_state(airspeed_fps=100.0, pitch_rad=np.deg2rad(3.0), altitude_ft=1000.0):
    U = airspeed_fps * np.cos(pitch_rad)
    W = airspeed_fps * np.sin(pitch_rad)
    return np.array([U, W, 0.0, pitch_rad, altitude_ft])


def test_flight_path_angle_is_pitch_minus_angle_of_attack():
    state = level_state()
    gamma, alpha, airspeed = flight_path_angle_from_state(state)
    assert gamma == pytest.approx(0.0, abs=1.0e-12)
    assert alpha == pytest.approx(state[3])
    assert airspeed == pytest.approx(100.0)


def test_lookahead_command_has_correct_sign_and_speed_scaling():
    slow_state = level_state(airspeed_fps=100.0)
    fast_state = level_state(airspeed_fps=200.0)
    climb_command, slow_distance = lookahead_flight_path_angle_command(
        1100.0, slow_state, 5.0
    )
    descent_command, _ = lookahead_flight_path_angle_command(
        900.0, slow_state, 5.0
    )
    fast_command, fast_distance = lookahead_flight_path_angle_command(
        1100.0, fast_state, 5.0
    )
    assert climb_command > 0.0
    assert descent_command < 0.0
    assert fast_distance > slow_distance
    assert abs(fast_command) < abs(climb_command)


def test_controller_uses_trim_pitch_and_c172_elevator_sign():
    trim_control = np.array([0.45, np.deg2rad(-1.0)])
    trim_pitch = np.deg2rad(3.0)
    controller = LookaheadAltitudeController(
        target_altitude_ft=1100.0,
        trim_control=trim_control,
        trim_pitch_rad=trim_pitch,
        elevator_limits_rad=(np.deg2rad(-23.0), np.deg2rad(28.0)),
    )
    control = controller(0.0, level_state(pitch_rad=trim_pitch))
    assert controller.last_diagnostics["pitch_command_rad"] > trim_pitch
    assert control[1] < trim_control[1]

    controller.reset()
    controller.target_altitude_ft = 1000.0
    control = controller(0.0, level_state(pitch_rad=trim_pitch))
    assert controller.last_diagnostics["pitch_command_rad"] == pytest.approx(trim_pitch)
    assert control == pytest.approx(trim_control)


def test_controlled_rk4_holds_one_control_per_step():
    def plant(t, state, control):
        return np.array([-state[0] + control[0]])

    def controller(t, state):
        return np.array([1.0])

    t, state, control = RK4_controlled(
        plant, controller, (0.0, 1.0), [0.0], 0.01
    )
    assert t[-1] == pytest.approx(1.0)
    assert state[-1, 0] == pytest.approx(1.0 - np.exp(-1.0), rel=1.0e-8)
    assert np.all(control == 1.0)
