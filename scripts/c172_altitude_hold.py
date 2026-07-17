#%% Imports
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flight_dynamics.altitude_control import LookaheadAltitudeController
from flight_dynamics.c172_params import params, dt
from flight_dynamics import conversions as conv
from flight_dynamics.integrators import RK4_controlled
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.trim_solver import longitudinal_trim


#%% Trim condition
trim_speed_fps = conv.kts2fps(90.0)
trim_altitude_ft = 4000.0
trim_target = np.array([trim_speed_fps, 0.0, trim_altitude_ft])
trim_initial_guess = np.array([0.45, np.deg2rad(-2.0), 0.0])

x_trim, u_trim, _ = longitudinal_trim(
    trim_initial_guess,
    trim_target,
    params,
    verbose=False,
)


#%% Controller and maneuver
altitude_step_ft = 100.0
step_time_s = 5.0
simulation_time_s = 60.0

controller = LookaheadAltitudeController(
    target_altitude_ft=trim_altitude_ft,
    trim_control=u_trim,
    trim_pitch_rad=x_trim[3],
    lookahead_time_s=5.0,
    gamma_kp=1.0,
    pitch_kp=1.0,
    pitch_kd=0.5,
    gamma_limit_rad=np.deg2rad(6.0),
    pitch_correction_limit_rad=np.deg2rad(10.0),
    elevator_limits_rad=params["control_limits"]["elevator"],
)


def scheduled_controller(t, state):
    controller.target_altitude_ft = (
        trim_altitude_ft if t < step_time_s else trim_altitude_ft + altitude_step_ft
    )
    return controller(t, state)


t, x, u = RK4_controlled(
    aircraft_longitudinal_dynamics,
    scheduled_controller,
    (0.0, simulation_time_s),
    x_trim,
    dt,
    args=(params,),
)

alpha = np.arctan2(x[:, 1], x[:, 0])
gamma = x[:, 3] - alpha
altitude_command = np.where(
    t < step_time_s,
    trim_altitude_ft,
    trim_altitude_ft + altitude_step_ft,
)
lookahead_distance = np.maximum(
    np.hypot(x[:, 0], x[:, 1]) * controller.lookahead_time_s,
    controller.minimum_lookahead_distance_ft,
)
gamma_command = np.clip(
    np.arctan2(altitude_command - x[:, 4], lookahead_distance),
    -controller.gamma_limit_rad,
    controller.gamma_limit_rad,
)


#%% Plots
fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

axes[0].plot(t, x[:, 4], label="Altitude")
axes[0].plot(t, altitude_command, "--", label="Command")
axes[0].set_ylabel("Altitude [ft]")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t, np.rad2deg(gamma), label="Flight-path angle")
axes[1].plot(t, np.rad2deg(gamma_command), "--", label="Command")
axes[1].set_ylabel("Gamma [deg]")
axes[1].legend()
axes[1].grid(True)

axes[2].plot(t, np.rad2deg(x[:, 3]), label="Pitch")
axes[2].plot(t, np.rad2deg(x[:, 2]), label="Pitch rate")
axes[2].set_ylabel("Angle / rate [deg, deg/s]")
axes[2].legend()
axes[2].grid(True)

axes[3].plot(t, np.rad2deg(u[:, 1]), label="Elevator")
axes[3].plot(t, conv.fps2kts(np.hypot(x[:, 0], x[:, 1])), label="TAS [kt]")
axes[3].set_xlabel("Time [s]")
axes[3].set_ylabel("Control / speed")
axes[3].legend()
axes[3].grid(True)

fig.suptitle("C172 Look-Ahead Altitude Hold Test")
fig.tight_layout()
plt.show()
