#%% Imports
from pathlib import Path
import sys

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flight_dynamics import control_inputs, conversions as conv
from flight_dynamics.aircraft_plotting import plot_six_dof_response, show_plots
from flight_dynamics.c172_params import params, dt
from flight_dynamics.integrators import RK4
from flight_dynamics.six_dof_dynamics import aircraft_six_dof_dynamics
from flight_dynamics.trim_solver import six_dof_trim


AIRCRAFT_NAME = "C172"
SIMULATION_DURATION_S = 20.0

# Select one input function for each control surface
SELECTED_ELEVATOR_INPUT = control_inputs.neutral_elevator_deflection
SELECTED_AILERON_INPUT = control_inputs.neutral_aileron_deflection
SELECTED_RUDDER_INPUT = control_inputs.neutral_rudder_deflection


#%% Set the six-DOF trim target
trim_speed_fps = conv.kts2fps(90.0)
trim_flight_path_angle_rad = np.deg2rad(0.0)
trim_altitude_ft = 4000.0
trim_heading_rad = np.deg2rad(0.0)

six_dof_trim_target = np.array([
    trim_speed_fps,
    trim_flight_path_angle_rad,
    trim_altitude_ft,
    trim_heading_rad,
])

# Initial guesses are throttle, elevator, aileron, rudder, alpha, beta, phi, theta
# Start the full nonlinear trim solver with every unknown set to zero
six_dof_trim_initial_guess = np.zeros(8)

# Solve all longitudinal and lateral trim states and controls together
six_dof_trim_state, six_dof_control, six_dof_trim_solution = six_dof_trim(
    six_dof_trim_initial_guess,
    six_dof_trim_target,
    params,
)


#%% Add the initial perturbation after solving trim
# Copy the trim state so the solved state is still available for comparison
six_dof_initial_state = six_dof_trim_state.copy()

# Start with a small bank-angle perturbation so the lateral response is visible
initial_bank_angle_rad = np.deg2rad(5.0)
six_dof_initial_state[6] += initial_bank_angle_rad


#%% Integrate the six-DOF equations of motion
simulation_time_s, six_dof_states = RK4(
    aircraft_six_dof_dynamics,
    (0.0, SIMULATION_DURATION_S),
    six_dof_initial_state,
    dt,
    args=(
        six_dof_control,
        params,
        SELECTED_ELEVATOR_INPUT,
        SELECTED_AILERON_INPUT,
        SELECTED_RUDDER_INPUT,
    ),
)


#%% Plot the six-DOF states
plot_six_dof_response(
    simulation_time_s,
    six_dof_states,
    aircraft_name=AIRCRAFT_NAME,
)

show_plots()
