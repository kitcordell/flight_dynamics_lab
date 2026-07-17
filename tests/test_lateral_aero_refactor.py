import numpy as np

from flight_dynamics import aero_model
from flight_dynamics.c172_params import params
from flight_dynamics.control_inputs import (
    neutral_aileron_deflection,
    neutral_elevator_deflection,
    neutral_rudder_deflection,
)
from flight_dynamics.lateral_dynamics import aircraft_lateral_dynamics
from flight_dynamics.six_dof_dynamics import aircraft_six_dof_dynamics


# These derivative vectors were captured from commit fa6cd9a before the lateral
# coefficient-to-load calculations were moved into aero_model.py.
LATERAL_DERIVATIVE_BASELINE = np.array([
    4.81208179946109,
    -1.957249252599053,
    0.4843098305929123,
    0.02904235747220455,
    0.01156631094240965,
    -0.01916083323852429,
    0.12890953742305822,
])

SIX_DOF_DERIVATIVE_BASELINE = np.array([
    -0.23067602759679970,
    4.8120817994610903,
    1.4640359437601185,
    -1.9572492525990530,
    0.06442793065360913,
    0.48430983059291233,
    0.029042357472204555,
    0.011566310942409648,
    -0.019160833238524293,
    146.30858505946739,
    34.172815515081041,
    0.12890953742305822,
])


def test_lateral_aero_loads_convert_coefficients_to_dimensional_loads():
    qbar = 12.5
    C_Y = -0.08
    C_l = 0.015
    C_n = -0.025

    side_force, roll_moment, yaw_moment = aero_model.lateral_aero_loads(
        qbar,
        params,
        C_Y,
        C_l,
        C_n,
    )

    expected_side_force = qbar * params["S"] * C_Y
    expected_roll_moment = qbar * params["S"] * params["bw"] * C_l
    expected_yaw_moment = qbar * params["S"] * params["bw"] * C_n

    assert side_force == expected_side_force
    assert roll_moment == expected_roll_moment
    assert yaw_moment == expected_yaw_moment


def test_lateral_derivatives_match_pre_refactor_baseline():
    lateral_state = np.array([5.0, 0.03, -0.02, 0.08, 0.05, 0.2, 4000.0])
    lateral_control = np.array([0.04, -0.03])
    longitudinal_state = np.array([150.0, 7.0, 0.01, 0.05, 4000.0])

    derivatives = aircraft_lateral_dynamics(
        0.0,
        lateral_state,
        lateral_control,
        params,
        longitudinal_state,
        neutral_aileron_deflection,
        neutral_rudder_deflection,
    )

    np.testing.assert_allclose(
        derivatives,
        LATERAL_DERIVATIVE_BASELINE,
        rtol=0.0,
        atol=1e-12,
    )


def test_six_dof_derivatives_match_pre_refactor_baseline():
    six_dof_state = np.array([
        150.0,
        5.0,
        7.0,
        0.03,
        0.01,
        -0.02,
        0.08,
        0.05,
        0.2,
        100.0,
        50.0,
        4000.0,
    ])
    six_dof_control = np.array([0.45, -0.01, 0.04, -0.03])

    derivatives = aircraft_six_dof_dynamics(
        0.0,
        six_dof_state,
        six_dof_control,
        params,
        neutral_elevator_deflection,
        neutral_aileron_deflection,
        neutral_rudder_deflection,
    )

    np.testing.assert_allclose(
        derivatives,
        SIX_DOF_DERIVATIVE_BASELINE,
        rtol=0.0,
        atol=1e-12,
    )
