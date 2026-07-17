"""Aircraft performance solver helpers."""

import numpy as np
from scipy.optimize import least_squares

from flight_dynamics import conversions
from flight_dynamics.atmosphere import standard_atmosphere
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.mechanics import velocity_max
from flight_dynamics.trim_solver import build_trim_states


def _altitude_array(alt_range):
    """Convert a single altitude or range definition into an altitude array."""
    # A scalar is treated as one requested altitude
    if np.isscalar(alt_range):
        altitudes = np.array([alt_range], dtype=float)
    else:
        # Convert lists, tuples, or numpy arrays into one flat array
        altitudes = np.asarray(alt_range, dtype=float).reshape(-1)

    # A one-value array is another way to request a single altitude
    if altitudes.size == 1:
        altitudes = altitudes.copy()

    # Three values define minimum altitude, maximum altitude, and step size
    elif altitudes.size == 3:
        alt_min, alt_max, alt_step = altitudes

        if alt_step <= 0.0:
            raise ValueError("Altitude step must be greater than zero")
        if alt_max < alt_min:
            raise ValueError("Maximum altitude must be greater than minimum altitude")

        # Add half a step so an evenly spaced maximum altitude is included
        altitudes = np.arange(
            alt_min,
            alt_max + 0.5 * alt_step,
            alt_step,
            dtype=float,
        )

    else:
        raise ValueError(
            "alt_range must be one altitude or "
            "[minimum altitude, maximum altitude, altitude step]"
        )

    if not np.all(np.isfinite(altitudes)):
        raise ValueError("Altitude values must be finite")

    return altitudes


# Calculates maximum true airspeed at one altitude or across an altitude range
# throttle: throttle setting from 0.0 to 1.0
# alt_range: one altitude or [minimum altitude, maximum altitude, altitude step]
# params: aircraft parameters
def airspeed_max(
    throttle,
    alt_range,
    params,
    plot=False,
    verbose=False,
    aircraft_name="Aircraft",
):
    """Calculate maximum true airspeed where power required equals power available."""
    # Throttle is passed to the propulsion model as a fraction of full power
    if not np.isfinite(throttle) or throttle < 0.0 or throttle > 1.0:
        raise ValueError("Throttle must be between 0.0 and 1.0")

    # Build the requested altitude points and initialize the output array
    alt_array = _altitude_array(alt_range)
    max_speed_array = np.zeros_like(alt_array, dtype=float)

    # Use the previous solution as the next initial guess as altitude increases
    V_max_guess = 300.0
    for i, altitude in enumerate(alt_array):
        V_max = velocity_max(
            altitude,
            throttle,
            params,
            V_max_guess,
        )
        max_speed_array[i] = V_max
        V_max_guess = V_max

        if verbose:
            print(
                f"Maximum airspeed at {altitude:.0f} ft: "
                f"{V_max:.2f} ft/s | {conversions.fps2kts(V_max):.2f} kt TAS"
            )

    # Keep plotting optional so this function can be used by scripts or solvers
    if plot:
        from flight_dynamics.aircraft_plotting import plot_maximum_airspeed

        plot_maximum_airspeed(
            alt_array,
            conversions.fps2kts(max_speed_array),
            aircraft_name=aircraft_name,
            throttle_percent=throttle * 100.0,
        )

    # Airspeeds are returned in ft/s to match the rest of the dynamics package
    return alt_array, max_speed_array


#%% Rate of Climb Solver

## Helps root solver determine trim conditions by determining how close the calculated derivative from the EOMs are to zero
def climb_trim_residuals(unknown, trim_target, params):
    # unknown = [theta, delta_e, gamma]
    theta, delta_e, gamma = unknown

    # trim_target = [throttle, V, alt]
    throttle, V, alt = trim_target

    x = build_trim_states(V, gamma, alt, theta)
    u = np.array([throttle, delta_e])

    xdot_actual = aircraft_longitudinal_dynamics(0.0, x, u, params)

    # Steady climb trim conditions
    residual = np.array([
        xdot_actual[0],   # U_dot
        xdot_actual[1],   # W_dot
        xdot_actual[2]    # Q_dot
    ])

    return residual


## Solves for the rate of climb given a throttle setting, TAS and altitude

# x0 = [theta_guess, delta_e_guess, gamma_guess]
# trim_target = [throttle, V, alt]
# params: aircraft parameters used by the dynamics model
def climb_trim_at_speed(x0, trim_target, params, verbose=True):
    throttle_trim, V_trim, alt_trim = trim_target

    sol = least_squares(
        climb_trim_residuals,
        x0,
        args=(trim_target, params)
    )

    theta_trim, delta_e_trim, gamma_trim = sol.x

    x = build_trim_states(V_trim, gamma_trim, alt_trim, theta_trim)
    u = np.array([throttle_trim, delta_e_trim])

    U_trim, W_trim, Q_trim, theta_state, alt_state = x

    ROC_fps = V_trim * np.sin(gamma_trim)
    ROC_fpm = ROC_fps * 60.0

    if verbose:
        print("\nTrim Target:")
        print("Throttle:", throttle_trim * 100, "[%]")
        print("Velocity:", V_trim, "[ft/s]")
        print("Altitude:", alt_trim, "[ft]")

        print("\nTrim Solution:")
        print("Theta:", theta_trim, "[rad]")
        print("Elevator Deflection:", delta_e_trim, "[rad]")
        print("Flight Path Angle:", gamma_trim, "[rad]")

        print("\nTrimmed State:")
        print("Forward Body Velocity (U):", U_trim, "[ft/s]")
        print("Vertical Body Velocity (W):", W_trim, "[ft/s]")
        print("Pitch Rate (Q):", Q_trim, "[rad/s]")
        print("Body Angle (theta):", theta_state, "[rad]")
        print("Altitude:", alt_state, "[ft]")

        print("\nClimb Performance:")
        print("ROC:", ROC_fps, "[ft/s]")
        print("ROC:", ROC_fpm, "[ft/min]")

    return x, u, gamma_trim, ROC_fps, ROC_fpm, sol


## Calculates the maximum rate of climb by sweeping through the minimum (V_S) and maximum (V_ne) possible airspeeds.
def max_ROC(x0, trim_target, params, num_points=50, verbose=True):
    throttle, _, alt = trim_target
    _, T, _ = standard_atmosphere(alt)

    # Convert IAS performance limits to TAS at the current altitude.
    V_ne = conversions.ias2tas(conversions.kts2fps(params["V_ne"]), alt, T)
    V_S = conversions.ias2tas(conversions.kts2fps(params["V_S"]), alt, T)
    V_array = np.linspace(V_S, V_ne, num_points)

    ROC_array = np.zeros_like(V_array)

    for i, V in enumerate(V_array):
        current_trim_target = [throttle, V, alt]
        _, _, _, ROC_fps, _, _ = climb_trim_at_speed(
            x0,
            current_trim_target,
            params,
            verbose=False,
        )
        ROC_array[i] = ROC_fps

    i_max = int(np.argmax(ROC_array))
    max_roc = ROC_array[i_max]
    max_roc_speed = V_array[i_max]

    if verbose:
        print(f"\nMaximum ROC found at V = {max_roc_speed} ft/s with ROC = {max_roc} ft/s | {max_roc*60} ft/min")

    return V_array, ROC_array, max_roc_speed, max_roc


## Calculates maximum rate of climb across a requested altitude range.
# x0 = [theta_guess, delta_e_guess, gamma_guess]
# throttle: throttle setting from 0.0 to 1.0
# alt_range: one altitude or [minimum altitude, maximum altitude, altitude step]
def dynamics_max_roc_vs_altitude(
    x0,
    throttle,
    alt_range,
    params,
    num_points=50,
    verbose=False,
):
    # Build the altitude array using the same input format as airspeed_max.
    alt_array = _altitude_array(alt_range)

    # Store the best climb speed and maximum ROC found at each altitude.
    max_roc_speed_array = np.zeros_like(alt_array, dtype=float)
    max_roc_array = np.zeros_like(alt_array, dtype=float)

    # Solve the full airspeed sweep independently at every altitude.
    for i, altitude in enumerate(alt_array):
        trim_target = [throttle, 0.0, altitude]
        _, _, max_roc_speed, max_roc = max_ROC(
            x0,
            trim_target,
            params,
            num_points=num_points,
            verbose=False,
        )

        max_roc_speed_array[i] = max_roc_speed
        max_roc_array[i] = max_roc

        if verbose:
            print(
                f"Maximum ROC at {altitude:.0f} ft: "
                f"{max_roc:.2f} ft/s | {max_roc * 60.0:.0f} ft/min "
                f"at {max_roc_speed:.2f} ft/s"
            )

    return alt_array, max_roc_speed_array, max_roc_array
