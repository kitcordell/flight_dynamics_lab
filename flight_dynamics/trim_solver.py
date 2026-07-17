"""Trim solvers shared by the longitudinal, lateral, and six-DOF models."""

import numpy as np
from scipy.optimize import least_squares

from flight_dynamics.axis_transformations import velocity_to_body
from flight_dynamics.constants import g
from flight_dynamics.lateral_dynamics import aircraft_lateral_dynamics
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.six_dof_dynamics import aircraft_six_dof_dynamics


# The optimizer works with dimensionless residuals made from these characteristic scales.
# Linear acceleration is scaled by one g, angular quantities by an order-one rate, and
# velocity errors by a representative light-aircraft speed.
DEFAULT_LINEAR_ACCELERATION_SCALE = g          # [ft/s^2]
DEFAULT_ANGULAR_ACCELERATION_SCALE = 1.0       # [rad/s^2]
DEFAULT_ANGULAR_RATE_SCALE = 1.0               # [rad/s]
DEFAULT_VELOCITY_SCALE = 100.0                 # [ft/s]

# A scaled residual norm of 1e-6 corresponds approximately to 3.2e-5 ft/s^2,
# 1e-6 rad/s^2, or 1e-4 ft/s when one residual component dominates.
DEFAULT_TRIM_RESIDUAL_TOLERANCE = 1.0e-6

# Values within this relative/absolute fraction of a finite bound are reported.
DEFAULT_BOUND_TOLERANCE = 1.0e-6

# Keep angle searches away from the Euler-angle singularity at +/- 90 degrees.
DEFAULT_TRIM_ANGLE_LIMIT_RAD = np.deg2rad(89.0)


class TrimConvergenceError(RuntimeError):
    """Raised when a trim optimizer exits without a valid low-residual solution."""


def _validate_positive(name, value):
    """Require a finite positive solver scale or tolerance."""
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")


def _control_bounds(aircraft_params, surface_name):
    """Read and validate one control-surface limit pair from the aircraft parameters."""
    try:
        lower, upper = aircraft_params["control_limits"][surface_name]
    except KeyError as exc:
        raise KeyError(
            f"aircraft_params['control_limits']['{surface_name}'] is required"
        ) from exc

    lower = float(lower)
    upper = float(upper)
    if not np.isfinite(lower) or not np.isfinite(upper) or lower >= upper:
        raise ValueError(
            f"{surface_name} control limits must be finite and ordered lower < upper"
        )

    return lower, upper


def _near_bound_names(values, bounds, variable_names, bound_tolerance):
    """Return names of solved variables that are at or very close to finite bounds."""
    lower_bounds, upper_bounds = bounds
    near_bounds = []

    for value, lower, upper, name in zip(
        values,
        lower_bounds,
        upper_bounds,
        variable_names,
    ):
        if np.isfinite(lower) and np.isclose(
            value,
            lower,
            rtol=bound_tolerance,
            atol=bound_tolerance,
        ):
            near_bounds.append(f"{name}:lower")

        if np.isfinite(upper) and np.isclose(
            value,
            upper,
            rtol=bound_tolerance,
            atol=bound_tolerance,
        ):
            near_bounds.append(f"{name}:upper")

    return tuple(near_bounds)


def _finalize_trim_result(
    sol,
    raw_residuals,
    residual_scales,
    bounds,
    variable_names,
    residual_tolerance,
    bound_tolerance,
):
    """Attach diagnostics to a SciPy result and reject an invalid trim solution."""
    _validate_positive("residual_tolerance", residual_tolerance)
    _validate_positive("bound_tolerance", bound_tolerance)

    raw_residuals = np.asarray(raw_residuals, dtype=float)
    residual_scales = np.asarray(residual_scales, dtype=float)

    if raw_residuals.shape != residual_scales.shape:
        raise ValueError("raw residuals and residual scales must have the same shape")
    if not np.all(np.isfinite(residual_scales)) or np.any(residual_scales <= 0.0):
        raise ValueError("all residual scales must be finite and greater than zero")

    scaled_residuals = raw_residuals / residual_scales

    # OptimizeResult supports additional named fields, which keeps the existing SciPy
    # result interface while exposing physical and scaled diagnostics to callers/tests.
    sol.raw_residuals = raw_residuals
    sol.scaled_residuals = scaled_residuals
    sol.raw_residual_norm = float(np.linalg.norm(raw_residuals))
    sol.scaled_residual_norm = float(np.linalg.norm(scaled_residuals))
    sol.residual_norm = sol.raw_residual_norm
    sol.residual_tolerance = float(residual_tolerance)
    sol.near_bounds = _near_bound_names(
        sol.x,
        bounds,
        variable_names,
        bound_tolerance,
    )
    sol.at_or_near_bound = bool(sol.near_bounds)

    sol.trim_valid = bool(
        sol.success
        and np.all(np.isfinite(raw_residuals))
        and sol.scaled_residual_norm <= residual_tolerance
    )

    if not sol.trim_valid:
        raise TrimConvergenceError(
            "Trim solve failed validation: "
            f"success={sol.success}, message={sol.message!s}, "
            f"scaled residual norm={sol.scaled_residual_norm:.6g}, "
            f"tolerance={residual_tolerance:.6g}, "
            f"raw residuals={raw_residuals}"
        )

    return sol


def _print_bound_status(sol):
    """Print a clear bound warning without treating a valid bounded solution as failure."""
    if sol.at_or_near_bound:
        print("Variables At or Near Bounds:", ", ".join(sol.near_bounds))
    else:
        print("Variables At or Near Bounds: none")


#%% Longitudinal Trim
def build_trim_states(V, gamma, alt, theta):
    """Build the longitudinal state [U, W, Q, theta, altitude]."""
    U, W, _ = velocity_to_body(V, gamma, theta)
    Q = 0.0 # pitch rate, [rad/s]
    return np.array([U, W, Q, theta, alt])


def level_trim_residuals(unknown, trim_target, aircraft_params):
    """Return raw physical longitudinal trim residuals."""
    V, gamma, alt = trim_target
    throttle, delta_e, theta = unknown

    x = build_trim_states(V, gamma, alt, theta)
    u = np.array([throttle, delta_e])

    xdot_desired = np.zeros_like(x)
    xdot_desired[4] = V * np.sin(gamma) # desired altitude rate, [ft/s]

    xdot_actual = aircraft_longitudinal_dynamics(
        0.0,
        x,
        u,
        aircraft_params,
    )
    return xdot_actual - xdot_desired


def _scaled_level_trim_residuals(
    unknown,
    trim_target,
    aircraft_params,
    residual_scales,
):
    return level_trim_residuals(unknown, trim_target, aircraft_params) / residual_scales


def longitudinal_trim(
    x0,
    trim_target,
    aircraft_params,
    verbose=True,
    *,
    residual_tolerance=DEFAULT_TRIM_RESIDUAL_TOLERANCE,
    bound_tolerance=DEFAULT_BOUND_TOLERANCE,
    linear_acceleration_scale=DEFAULT_LINEAR_ACCELERATION_SCALE,
    angular_acceleration_scale=DEFAULT_ANGULAR_ACCELERATION_SCALE,
    angular_rate_scale=DEFAULT_ANGULAR_RATE_SCALE,
    velocity_scale=DEFAULT_VELOCITY_SCALE,
):
    """Solve longitudinal trim and return state, control, and validated result."""
    delta_e_min, delta_e_max = _control_bounds(aircraft_params, "elevator")

    lower_bounds = np.array([
        0.0,
        delta_e_min,
        -DEFAULT_TRIM_ANGLE_LIMIT_RAD,
    ])
    upper_bounds = np.array([
        1.0,
        delta_e_max,
        DEFAULT_TRIM_ANGLE_LIMIT_RAD,
    ])
    bounds = (lower_bounds, upper_bounds)

    residual_scales = np.array([
        linear_acceleration_scale,
        linear_acceleration_scale,
        angular_acceleration_scale,
        angular_rate_scale,
        velocity_scale,
    ])

    sol = least_squares(
        _scaled_level_trim_residuals,
        x0,
        bounds=bounds,
        args=(trim_target, aircraft_params, residual_scales),
        method="dogbox",
    )

    V_trim, gamma_trim, alt_trim = trim_target
    throttle_trim, delta_e_trim, theta_trim = sol.x
    x = build_trim_states(V_trim, gamma_trim, alt_trim, theta_trim)
    u = np.array([throttle_trim, delta_e_trim])

    raw_residuals = level_trim_residuals(sol.x, trim_target, aircraft_params)
    sol = _finalize_trim_result(
        sol,
        raw_residuals,
        residual_scales,
        bounds,
        ("throttle", "elevator", "theta"),
        residual_tolerance,
        bound_tolerance,
    )

    if verbose:
        U_trim, W_trim, Q_trim, _, _ = x

        print("\nLongitudinal Trim Target:")
        print("Velocity:", V_trim, "[ft/s]")
        print("Flight Path Angle:", gamma_trim, "[rad]")
        print("Altitude:", alt_trim, "[ft]")

        print("\nLongitudinal Trim Solutions:")
        print("Throttle:", throttle_trim, "[0-1]")
        print("Elevator Deflection:", delta_e_trim, "[rad]")
        print("Pitch Angle:", theta_trim, "[rad]")
        print("Forward Body Velocity (U):", U_trim, "[ft/s]")
        print("Vertical Body Velocity (W):", W_trim, "[ft/s]")
        print("Pitch Rate (Q):", Q_trim, "[rad/s]")

        print("\nLongitudinal Trim Raw Residuals:")
        print("U Acceleration Residual:", sol.raw_residuals[0], "[ft/s^2]")
        print("W Acceleration Residual:", sol.raw_residuals[1], "[ft/s^2]")
        print("Pitch Acceleration Residual:", sol.raw_residuals[2], "[rad/s^2]")
        print("Pitch Angle Rate Residual:", sol.raw_residuals[3], "[rad/s]")
        print("Altitude Rate Residual:", sol.raw_residuals[4], "[ft/s]")
        _print_bound_status(sol)

    return x, u, sol


#%% Lateral Trim
def build_lateral_trim_state(V_side, phi, heading, longitudinal_state):
    """Build the reduced lateral state [V, P, R, phi, theta, psi, altitude]."""
    _, _, _, theta, alt = longitudinal_state
    P = 0.0 # roll rate, [rad/s]
    R = 0.0 # yaw rate, [rad/s]

    return np.array([
        V_side,
        P,
        R,
        phi,
        theta,
        heading,
        alt,
    ])


def lateral_trim_residuals(unknown, longitudinal_state, trim_target, aircraft_params):
    """Return raw physical lateral trim residuals."""
    delta_a, delta_r, V_side, phi = unknown
    beta_target, heading = trim_target

    x = build_lateral_trim_state(V_side, phi, heading, longitudinal_state)
    u = np.array([delta_a, delta_r])
    xdot = aircraft_lateral_dynamics(
        0.0,
        x,
        u,
        aircraft_params,
        longitudinal_state,
    )

    U_trim, W_trim, _, _, _ = longitudinal_state
    longitudinal_speed = np.sqrt(U_trim**2 + W_trim**2)
    desired_side_velocity = longitudinal_speed * np.tan(beta_target)

    return np.array([
        xdot[0],
        xdot[1],
        xdot[2],
        V_side - desired_side_velocity,
    ])


def _scaled_lateral_trim_residuals(
    unknown,
    longitudinal_state,
    trim_target,
    aircraft_params,
    residual_scales,
):
    return (
        lateral_trim_residuals(
            unknown,
            longitudinal_state,
            trim_target,
            aircraft_params,
        )
        / residual_scales
    )


def lateral_trim(
    x0,
    longitudinal_state,
    trim_target,
    aircraft_params,
    verbose=True,
    *,
    residual_tolerance=DEFAULT_TRIM_RESIDUAL_TOLERANCE,
    bound_tolerance=DEFAULT_BOUND_TOLERANCE,
    linear_acceleration_scale=DEFAULT_LINEAR_ACCELERATION_SCALE,
    angular_acceleration_scale=DEFAULT_ANGULAR_ACCELERATION_SCALE,
    velocity_scale=DEFAULT_VELOCITY_SCALE,
):
    """Solve lateral trim and return state, control, and validated result."""
    delta_a_min, delta_a_max = _control_bounds(aircraft_params, "aileron")
    delta_r_min, delta_r_max = _control_bounds(aircraft_params, "rudder")

    lower_bounds = np.array([
        delta_a_min,
        delta_r_min,
        -np.inf,
        -DEFAULT_TRIM_ANGLE_LIMIT_RAD,
    ])
    upper_bounds = np.array([
        delta_a_max,
        delta_r_max,
        np.inf,
        DEFAULT_TRIM_ANGLE_LIMIT_RAD,
    ])
    bounds = (lower_bounds, upper_bounds)

    residual_scales = np.array([
        linear_acceleration_scale,
        angular_acceleration_scale,
        angular_acceleration_scale,
        velocity_scale,
    ])

    sol = least_squares(
        _scaled_lateral_trim_residuals,
        x0,
        bounds=bounds,
        args=(
            longitudinal_state,
            trim_target,
            aircraft_params,
            residual_scales,
        ),
        method="dogbox",
    )

    delta_a, delta_r, V_side, phi = sol.x
    beta_target, heading = trim_target
    x = build_lateral_trim_state(V_side, phi, heading, longitudinal_state)
    u = np.array([delta_a, delta_r])

    raw_residuals = lateral_trim_residuals(
        sol.x,
        longitudinal_state,
        trim_target,
        aircraft_params,
    )
    sol = _finalize_trim_result(
        sol,
        raw_residuals,
        residual_scales,
        bounds,
        ("aileron", "rudder", "side_velocity", "phi"),
        residual_tolerance,
        bound_tolerance,
    )

    if verbose:
        print("\nLateral Trim Target:")
        print("Sideslip Angle:", beta_target, "[rad]")
        print("Heading:", heading, "[rad]")

        print("\nLateral Trim Solutions:")
        print("Aileron Deflection:", delta_a, "[rad]")
        print("Rudder Deflection:", delta_r, "[rad]")
        print("Side Body Velocity (V):", V_side, "[ft/s]")
        print("Bank Angle:", phi, "[rad]")

        print("\nLateral Trim Raw Residuals:")
        print("V Acceleration Residual:", sol.raw_residuals[0], "[ft/s^2]")
        print("Roll Acceleration Residual:", sol.raw_residuals[1], "[rad/s^2]")
        print("Yaw Acceleration Residual:", sol.raw_residuals[2], "[rad/s^2]")
        print("Side Velocity Residual:", sol.raw_residuals[3], "[ft/s]")
        _print_bound_status(sol)

    return x, u, sol


#%% Six-DOF Trim
def build_six_dof_trim_state(V_tas, alt, alpha, beta, phi, theta, psi):
    """Build the complete 12-state vector used by the six-DOF model."""
    U = V_tas * np.cos(alpha) * np.cos(beta)
    V = V_tas * np.sin(beta)
    W = V_tas * np.sin(alpha) * np.cos(beta)

    return np.array([
        U,
        V,
        W,
        0.0, # roll rate P, [rad/s]
        0.0, # pitch rate Q, [rad/s]
        0.0, # yaw rate R, [rad/s]
        phi,
        theta,
        psi,
        0.0, # north position, [ft]
        0.0, # east position, [ft]
        alt,
    ])


def six_dof_trim_residuals(unknown, trim_target, aircraft_params):
    """Return the eight raw physical six-DOF trim residuals."""
    V_tas, gamma, alt, heading = trim_target
    throttle, delta_e, delta_a, delta_r, alpha, beta, phi, theta = unknown

    x = build_six_dof_trim_state(
        V_tas,
        alt,
        alpha,
        beta,
        phi,
        theta,
        heading,
    )
    u = np.array([throttle, delta_e, delta_a, delta_r])
    xdot = aircraft_six_dof_dynamics(0.0, x, u, aircraft_params)

    desired_altitude_rate = V_tas * np.sin(gamma)
    altitude_rate_residual = xdot[11] - desired_altitude_rate
    cross_track_velocity = (
        -xdot[9] * np.sin(heading)
        + xdot[10] * np.cos(heading)
    )

    return np.concatenate((
        xdot[:6],
        np.array([
            altitude_rate_residual,
            cross_track_velocity,
        ]),
    ))


def six_dof_trim_residual_scales(
    linear_acceleration_scale=DEFAULT_LINEAR_ACCELERATION_SCALE,
    angular_acceleration_scale=DEFAULT_ANGULAR_ACCELERATION_SCALE,
    velocity_scale=DEFAULT_VELOCITY_SCALE,
):
    """Return characteristic scales matching the eight six-DOF residuals."""
    return np.array([
        linear_acceleration_scale,
        linear_acceleration_scale,
        linear_acceleration_scale,
        angular_acceleration_scale,
        angular_acceleration_scale,
        angular_acceleration_scale,
        velocity_scale,
        velocity_scale,
    ])


def _scaled_six_dof_trim_residuals(
    unknown,
    trim_target,
    aircraft_params,
    residual_scales,
):
    return six_dof_trim_residuals(unknown, trim_target, aircraft_params) / residual_scales


def six_dof_trim(
    x0,
    trim_target,
    aircraft_params,
    verbose=True,
    *,
    residual_tolerance=DEFAULT_TRIM_RESIDUAL_TOLERANCE,
    bound_tolerance=DEFAULT_BOUND_TOLERANCE,
    linear_acceleration_scale=DEFAULT_LINEAR_ACCELERATION_SCALE,
    angular_acceleration_scale=DEFAULT_ANGULAR_ACCELERATION_SCALE,
    velocity_scale=DEFAULT_VELOCITY_SCALE,
):
    """Solve full straight-flight trim and return state, control, and result."""
    delta_e_min, delta_e_max = _control_bounds(aircraft_params, "elevator")
    delta_a_min, delta_a_max = _control_bounds(aircraft_params, "aileron")
    delta_r_min, delta_r_max = _control_bounds(aircraft_params, "rudder")

    lower_bounds = np.array([
        0.0,
        delta_e_min,
        delta_a_min,
        delta_r_min,
        -DEFAULT_TRIM_ANGLE_LIMIT_RAD,
        -DEFAULT_TRIM_ANGLE_LIMIT_RAD,
        -DEFAULT_TRIM_ANGLE_LIMIT_RAD,
        -DEFAULT_TRIM_ANGLE_LIMIT_RAD,
    ])
    upper_bounds = np.array([
        1.0,
        delta_e_max,
        delta_a_max,
        delta_r_max,
        DEFAULT_TRIM_ANGLE_LIMIT_RAD,
        DEFAULT_TRIM_ANGLE_LIMIT_RAD,
        DEFAULT_TRIM_ANGLE_LIMIT_RAD,
        DEFAULT_TRIM_ANGLE_LIMIT_RAD,
    ])
    bounds = (lower_bounds, upper_bounds)

    residual_scales = six_dof_trim_residual_scales(
        linear_acceleration_scale,
        angular_acceleration_scale,
        velocity_scale,
    )

    sol = least_squares(
        _scaled_six_dof_trim_residuals,
        x0,
        bounds=bounds,
        args=(trim_target, aircraft_params, residual_scales),
        method="dogbox",
    )

    V_tas, gamma, alt, heading = trim_target
    throttle, delta_e, delta_a, delta_r, alpha, beta, phi, theta = sol.x
    x = build_six_dof_trim_state(
        V_tas,
        alt,
        alpha,
        beta,
        phi,
        theta,
        heading,
    )
    u = np.array([throttle, delta_e, delta_a, delta_r])

    raw_residuals = six_dof_trim_residuals(sol.x, trim_target, aircraft_params)
    sol = _finalize_trim_result(
        sol,
        raw_residuals,
        residual_scales,
        bounds,
        (
            "throttle",
            "elevator",
            "aileron",
            "rudder",
            "alpha",
            "beta",
            "phi",
            "theta",
        ),
        residual_tolerance,
        bound_tolerance,
    )

    if verbose:
        print("\nSix-DOF Trim Target:")
        print("Velocity:", V_tas, "[ft/s]")
        print("Flight Path Angle:", gamma, "[rad]")
        print("Altitude:", alt, "[ft]")
        print("Heading:", heading, "[rad]")

        print("\nSix-DOF Trim Solutions:")
        print("Throttle:", throttle, "[0-1]")
        print("Elevator Deflection:", delta_e, "[rad]")
        print("Aileron Deflection:", delta_a, "[rad]")
        print("Rudder Deflection:", delta_r, "[rad]")
        print("Angle of Attack:", alpha, "[rad]")
        print("Sideslip Angle:", beta, "[rad]")
        print("Bank Angle:", phi, "[rad]")
        print("Pitch Angle:", theta, "[rad]")

        print("\nSix-DOF Trim Raw Residuals:")
        print("U Acceleration Residual:", sol.raw_residuals[0], "[ft/s^2]")
        print("V Acceleration Residual:", sol.raw_residuals[1], "[ft/s^2]")
        print("W Acceleration Residual:", sol.raw_residuals[2], "[ft/s^2]")
        print("Roll Acceleration Residual:", sol.raw_residuals[3], "[rad/s^2]")
        print("Pitch Acceleration Residual:", sol.raw_residuals[4], "[rad/s^2]")
        print("Yaw Acceleration Residual:", sol.raw_residuals[5], "[rad/s^2]")
        print("Altitude Rate Residual:", sol.raw_residuals[6], "[ft/s]")
        print("Cross-Track Velocity Residual:", sol.raw_residuals[7], "[ft/s]")
        _print_bound_status(sol)

    return x, u, sol
