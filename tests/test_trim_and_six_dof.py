"""Validation tests for bounded trim and the nonlinear C172 six-DOF model."""

from copy import deepcopy

import numpy as np
import pytest

from flight_dynamics import control_inputs, trim_solver
from flight_dynamics.c172_params import dt, params
from flight_dynamics.conversions import kts2fps
from flight_dynamics.integrators import RK4
from flight_dynamics.six_dof_dynamics import aircraft_six_dof_dynamics
from flight_dynamics.trim_solver import (
    TrimConvergenceError,
    lateral_trim,
    longitudinal_trim,
    six_dof_trim,
)


TRIM_SPEED_FPS = kts2fps(90.0)
TRIM_ALTITUDE_FT = 4000.0
TRIM_GAMMA_RAD = 0.0
TRIM_HEADING_RAD = 0.0
SIX_DOF_SIMULATION_DURATION_S = 20.0

# Raw physical tolerances are intentionally much looser than the nominal numerical
# result while still being small relative to flight-dynamics acceleration and speed.
LINEAR_ACCELERATION_TOLERANCE = 1.0e-6 # [ft/s^2]
ANGULAR_ACCELERATION_TOLERANCE = 1.0e-8 # [rad/s^2]
VELOCITY_TOLERANCE = 1.0e-6 # [ft/s]
ANGLE_RATE_TOLERANCE = 1.0e-8 # [rad/s]


@pytest.fixture(scope="module")
def six_dof_trim_case():
    """Solve the nominal C172 six-DOF trim condition once for shared tests."""
    trim_target = np.array([
        TRIM_SPEED_FPS,
        TRIM_GAMMA_RAD,
        TRIM_ALTITUDE_FT,
        TRIM_HEADING_RAD,
    ])
    trim_state, trim_control, result = six_dof_trim(
        np.zeros(8),
        trim_target,
        params,
        verbose=False,
    )
    return trim_target, trim_state, trim_control, result


def test_six_dof_trim_converges_and_exposes_diagnostics(six_dof_trim_case):
    trim_target, trim_state, trim_control, result = six_dof_trim_case

    assert result.success
    assert result.trim_valid
    assert result.raw_residuals.shape == (8,)
    assert np.isfinite(result.raw_residual_norm)
    assert result.residual_norm == result.raw_residual_norm
    assert not hasattr(result, "scaled_residuals")
    assert not hasattr(result, "scaled_residual_norm")
    assert np.all(np.isfinite(trim_state))
    assert np.all(np.isfinite(trim_control))
    assert trim_target[0] == pytest.approx(TRIM_SPEED_FPS)


def test_six_dof_trim_controls_respect_declared_limits(six_dof_trim_case):
    _, _, trim_control, result = six_dof_trim_case
    throttle, delta_e, delta_a, delta_r = trim_control
    limits = params["control_limits"]

    assert 0.0 <= throttle <= 1.0
    assert limits["elevator"][0] <= delta_e <= limits["elevator"][1]
    assert limits["aileron"][0] <= delta_a <= limits["aileron"][1]
    assert limits["rudder"][0] <= delta_r <= limits["rudder"][1]
    assert not result.at_or_near_bound


def test_six_dof_raw_trim_residuals_are_below_physical_tolerances(
    six_dof_trim_case,
):
    _, _, _, result = six_dof_trim_case
    raw = result.raw_residuals

    assert np.max(np.abs(raw[:3])) < LINEAR_ACCELERATION_TOLERANCE
    assert np.max(np.abs(raw[3:6])) < ANGULAR_ACCELERATION_TOLERANCE
    assert abs(raw[6]) < VELOCITY_TOLERANCE
    assert abs(raw[7]) < VELOCITY_TOLERANCE


def test_six_dof_dynamics_match_the_trim_target(six_dof_trim_case):
    trim_target, trim_state, trim_control, _ = six_dof_trim_case
    V_tas, gamma, _, heading = trim_target

    xdot = aircraft_six_dof_dynamics(
        0.0,
        trim_state,
        trim_control,
        params,
        control_inputs.neutral_elevator_deflection,
        control_inputs.neutral_aileron_deflection,
        control_inputs.neutral_rudder_deflection,
    )

    desired_altitude_rate = V_tas * np.sin(gamma)
    cross_track_velocity = (
        -xdot[9] * np.sin(heading)
        + xdot[10] * np.cos(heading)
    )

    assert np.max(np.abs(xdot[:3])) < LINEAR_ACCELERATION_TOLERANCE
    assert np.max(np.abs(xdot[3:6])) < ANGULAR_ACCELERATION_TOLERANCE
    assert np.max(np.abs(xdot[6:9])) < ANGLE_RATE_TOLERANCE
    assert abs(xdot[11] - desired_altitude_rate) < VELOCITY_TOLERANCE
    assert abs(cross_track_velocity) < VELOCITY_TOLERANCE


def test_positive_aileron_produces_negative_initial_roll_acceleration(
    six_dof_trim_case,
):
    """C_l_delta_a < 0 defines positive aileron as negative body-x roll here."""
    _, trim_state, trim_control, _ = six_dof_trim_case
    baseline = aircraft_six_dof_dynamics(
        0.0,
        trim_state,
        trim_control,
        params,
        control_inputs.neutral_elevator_deflection,
        control_inputs.neutral_aileron_deflection,
        control_inputs.neutral_rudder_deflection,
    )

    positive_aileron_control = trim_control.copy()
    positive_aileron_control[2] += np.deg2rad(1.0)
    perturbed = aircraft_six_dof_dynamics(
        0.0,
        trim_state,
        positive_aileron_control,
        params,
        control_inputs.neutral_elevator_deflection,
        control_inputs.neutral_aileron_deflection,
        control_inputs.neutral_rudder_deflection,
    )

    assert params["C_l_delta_a"] < 0.0
    assert perturbed[3] < baseline[3]


def test_positive_rudder_produces_negative_initial_yaw_acceleration(
    six_dof_trim_case,
):
    """C_n_delta_r < 0 defines positive rudder as negative body-z yaw here."""
    _, trim_state, trim_control, _ = six_dof_trim_case
    baseline = aircraft_six_dof_dynamics(
        0.0,
        trim_state,
        trim_control,
        params,
        control_inputs.neutral_elevator_deflection,
        control_inputs.neutral_aileron_deflection,
        control_inputs.neutral_rudder_deflection,
    )

    positive_rudder_control = trim_control.copy()
    positive_rudder_control[3] += np.deg2rad(1.0)
    perturbed = aircraft_six_dof_dynamics(
        0.0,
        trim_state,
        positive_rudder_control,
        params,
        control_inputs.neutral_elevator_deflection,
        control_inputs.neutral_aileron_deflection,
        control_inputs.neutral_rudder_deflection,
    )

    assert params["C_n_delta_r"] < 0.0
    assert perturbed[5] < baseline[5]


def test_bank_perturbation_integrates_without_nonfinite_states(six_dof_trim_case):
    _, trim_state, trim_control, _ = six_dof_trim_case
    initial_state = trim_state.copy()
    initial_state[6] += np.deg2rad(5.0)

    _, states = RK4(
        aircraft_six_dof_dynamics,
        (0.0, SIX_DOF_SIMULATION_DURATION_S),
        initial_state,
        dt,
        args=(
            trim_control,
            params,
            control_inputs.neutral_elevator_deflection,
            control_inputs.neutral_aileron_deflection,
            control_inputs.neutral_rudder_deflection,
        ),
    )

    assert np.all(np.isfinite(states))


def test_longitudinal_trim_uses_explicit_aircraft_parameters():
    assert not hasattr(trim_solver, "params")

    trim_target = np.array([
        TRIM_SPEED_FPS,
        TRIM_GAMMA_RAD,
        TRIM_ALTITUDE_FT,
    ])
    initial_guess = np.array([0.45, np.deg2rad(-2.0), 0.0])

    baseline_params = deepcopy(params)
    _, baseline_control, baseline_result = longitudinal_trim(
        initial_guess,
        trim_target,
        baseline_params,
        verbose=False,
    )

    lower_power_params = deepcopy(params)
    lower_power_params["P_max_SL"] *= 0.9
    _, lower_power_control, lower_power_result = longitudinal_trim(
        initial_guess,
        trim_target,
        lower_power_params,
        verbose=False,
    )

    assert baseline_result.trim_valid
    assert lower_power_result.trim_valid
    assert lower_power_control[0] > baseline_control[0]


def test_lateral_trim_is_bounded_and_validated():
    longitudinal_target = np.array([
        TRIM_SPEED_FPS,
        TRIM_GAMMA_RAD,
        TRIM_ALTITUDE_FT,
    ])
    longitudinal_state, _, _ = longitudinal_trim(
        np.array([0.45, np.deg2rad(-2.0), 0.0]),
        longitudinal_target,
        params,
        verbose=False,
    )

    _, lateral_control, result = lateral_trim(
        np.zeros(4),
        longitudinal_state,
        np.array([0.0, TRIM_HEADING_RAD]),
        params,
        verbose=False,
    )

    assert result.success
    assert result.trim_valid
    assert result.raw_residuals.shape == (4,)
    assert params["control_limits"]["aileron"][0] <= lateral_control[0]
    assert lateral_control[0] <= params["control_limits"]["aileron"][1]
    assert params["control_limits"]["rudder"][0] <= lateral_control[1]
    assert lateral_control[1] <= params["control_limits"]["rudder"][1]


def test_invalid_bounded_longitudinal_trim_raises_clear_exception():
    restricted_params = deepcopy(params)
    restricted_params["control_limits"]["elevator"] = (
        0.0,
        np.deg2rad(1.0),
    )

    with pytest.raises(TrimConvergenceError, match="failed validation"):
        longitudinal_trim(
            np.array([0.45, 0.0, 0.0]),
            np.array([TRIM_SPEED_FPS, TRIM_GAMMA_RAD, TRIM_ALTITUDE_FT]),
            restricted_params,
            verbose=False,
        )
