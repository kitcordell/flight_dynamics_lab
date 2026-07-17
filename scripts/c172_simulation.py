#%% Imports
from pathlib import Path
import sys

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flight_dynamics.c172_params import params, tf, dt
from flight_dynamics import control_inputs
from flight_dynamics.external_data import (
    load_rate_of_climb_reference_data,
    load_xplane_longitudinal_data,
)
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.mechanics import drag_polar, power_curves
from flight_dynamics.performance_solver import (
    airspeed_max,
    dynamics_max_roc_vs_altitude,
    max_ROC,
)
from flight_dynamics.trim_solver import longitudinal_trim
from flight_dynamics import conversions as conv
from flight_dynamics.aircraft_plotting import (
    plot_aerodynamic_performance,
    plot_longitudinal_comparison,
    plot_performance_limits,
    show_plots,
)
from flight_dynamics.integrators import RK4

DATA_DIR = ROOT_DIR / "data"
# Match this label to the aircraft model and reference data loaded by the script.
AIRCRAFT_NAME = "C172"

# Select the elevator input function used by the longitudinal equations of motion.
# Change this to control_inputs.neutral_elevator_deflection for no elevator pulse.
SELECTED_CONTROL_INPUT = control_inputs.elevator_deflection

#%% Max Airspeed
max_speed_throttle = 0.75
max_speed_min_alt = 0
max_speed_max_alt = 12000
max_speed_alt_step = 1000

max_speed_alt_array, max_speed_array = airspeed_max(
    max_speed_throttle,
    [max_speed_min_alt, max_speed_max_alt, max_speed_alt_step],
    params,
)

print(
    f"Max Velocity @ {max_speed_throttle * 100:.0f}% throttle, "
    f"{max_speed_alt_array[0]:.0f} ft: "
    f"{max_speed_array[0]:.2f} ft/s | {conv.fps2kts(max_speed_array[0]):.2f} kts"
)

#%% Rate of Climb Calculations
# initial guess = [theta, delta_e, gamma]
roc_theta_guess = np.deg2rad(5.0)  # 5 degrees
roc_delta_e_guess = np.deg2rad(-2.0)  # -2 degrees
roc_gamma_guess = np.deg2rad(3.0)  # 3 degrees climb

roc_initial_guess = np.array([
    roc_theta_guess,
    roc_delta_e_guess,
    roc_gamma_guess,
])

roc_throttle = 1.0
roc_speed_sweep_initial_speed_fps = 90.0

roc_min_altitude_ft = 0
roc_max_altitude_ft = 12000
roc_altitude_step_ft = 1000

# Compute the maximum ROC at every requested altitude.
alt_array, _, ROC_alt_array = dynamics_max_roc_vs_altitude(
    roc_initial_guess,
    roc_throttle,
    [roc_min_altitude_ft, roc_max_altitude_ft, roc_altitude_step_ft],
    params,
    num_points=500,
    verbose=False,
)

# Keep one full airspeed sweep for the rate-of-climb versus airspeed plot.
roc_speed_sweep_target = [
    roc_throttle,
    roc_speed_sweep_initial_speed_fps,
    alt_array[-1],
]
V_array, ROC_array, V_max_ROC, ROC_max = max_ROC(
    roc_initial_guess,
    roc_speed_sweep_target,
    params,
    num_points=50,
    verbose=False,
)

#%% POH Data
roc_reference_data = load_rate_of_climb_reference_data(
    DATA_DIR / "c172_roc.csv"
)

plot_performance_limits(
    max_speed_alt_array,
    conv.fps2kts(max_speed_array),
    roc_reference_data["pressure_altitude_ft"],
    roc_reference_data["roc_fpm_20c"],
    alt_array,
    ROC_alt_array * 60,
    V_array,
    ROC_array * 60,
    V_max_ROC,
    ROC_max * 60,
    aircraft_name=AIRCRAFT_NAME,
    throttle_percent=max_speed_throttle * 100,
    reference_name="POH",
    comparison_temperature_c=20,
)

#%% Drag Polar and Power Calculations
drag_polar_altitude_ft = 0
power_curve_altitude_ft = 4000
power_curve_throttle = 1.0

drag_airspeed, D, D_i, D_p = drag_polar(drag_polar_altitude_ft, params)
power_airspeed, P_req, _, _, P_A = power_curves(
    power_curve_altitude_ft,
    power_curve_throttle,
    params,
)

plot_aerodynamic_performance(
    drag_airspeed,
    D,
    D_i,
    D_p,
    power_airspeed,
    P_req,
    P_A,
    aircraft_name=AIRCRAFT_NAME,
)

#%% Solve for trim conditions
# Trim Conditions
trim_speed_fps = conv.kts2fps(90)
trim_flight_path_angle_rad = np.deg2rad(0.0)
trim_altitude_ft = 4000
trim_target = np.array([
    trim_speed_fps,
    trim_flight_path_angle_rad,
    trim_altitude_ft,
])

# Guessed unknown states
throttle_guess = 0.45
delta_e_guess = np.deg2rad(-2.0)
theta_guess = 0.0
trim_initial_guess = np.array([
    throttle_guess,
    delta_e_guess,
    theta_guess,
])

x_trim, u_trim, trim_solution = longitudinal_trim(
    trim_initial_guess,
    trim_target,
    params,
)

#%% Dynamics Calculations
# Uses RK4 integration with the nonlinear longitudinal equations of motion
t_rk4, x_rk4 = RK4(
    aircraft_longitudinal_dynamics,
    (0.0, tf),
    x_trim,
    dt,
    args=(u_trim, params, SELECTED_CONTROL_INPUT),
)

# Calculate angle of attack from the forward and vertical body velocities
alpha = np.arctan2(x_rk4[:, 1], x_rk4[:, 0])

# Log the elevator schedule applied by the dynamics model at each time step
_, trim_elevator_rad = u_trim
elevator_deflection_history = SELECTED_CONTROL_INPUT(
    t_rk4,
    trim_elevator_rad,
)

#%% X-Plane Data
xplane_data = load_xplane_longitudinal_data(
    DATA_DIR / "Data.txt",
    row_start=6000,
    row_end=9000,
    time_reference_index=1,
)

#%% Comparison Plots
plot_longitudinal_comparison(
    t_rk4,
    x_rk4,
    alpha,
    elevator_deflection_history,
    aircraft_name=AIRCRAFT_NAME,
    reference_time_s=xplane_data["time_s"],
    reference_u_fps=xplane_data["u_fps"],
    reference_w_fps=xplane_data["w_fps"],
    reference_q_deg_s=xplane_data["q_deg_s"],
    reference_theta_deg=xplane_data["pitch_deg"],
    reference_alpha_deg=xplane_data["alpha_deg"],
    reference_altitude_ft=xplane_data["altitude_ft"],
    reference_elevator_deg=xplane_data["elevator_deg"],
    simulation_name="Sim",
    reference_name="X-Plane",
)

show_plots()
