"""Reusable plotting functions for aircraft performance and simulation data."""

import matplotlib.pyplot as plt
import numpy as np


def _title(aircraft_name, description):
    aircraft_name = str(aircraft_name).strip()
    if not aircraft_name:
        raise ValueError("aircraft_name must not be empty")
    return f"{aircraft_name} {description}"


def _plot_maximum_airspeed_axis(
    ax,
    altitude_ft,
    airspeed_ktas,
    throttle_percent,
):
    """Draw maximum airspeed data on an existing axis."""
    ax.plot(
        altitude_ft,
        airspeed_ktas,
        marker="o",
        label=f"Estimated {throttle_percent:g}% Throttle",
    )
    ax.set_title("Maximum TAS vs Altitude")
    ax.set_xlabel("Pressure Altitude (ft)")
    ax.set_ylabel("Maximum Airspeed (kt TAS)")
    ax.grid(True)
    ax.legend()


def _plot_roc_altitude_axis(
    ax,
    reference_altitude_ft,
    reference_roc_fpm,
    simulated_altitude_ft,
    simulated_roc_fpm,
    reference_name,
    comparison_temperature_c,
):
    """Draw reference and simulated maximum ROC versus altitude."""
    altitude_title = "Maximum Rate of Climb vs Altitude"
    if comparison_temperature_c is not None:
        altitude_title += f" @ {comparison_temperature_c:g}C"

    ax.plot(
        reference_altitude_ft,
        reference_roc_fpm,
        label=f"{reference_name} ROC",
    )
    ax.plot(
        simulated_altitude_ft,
        simulated_roc_fpm,
        label="Sim ROC",
    )
    ax.set_title(altitude_title)
    ax.set_xlabel("Altitude (ft)")
    ax.set_ylabel("Rate of Climb (ft/min)")
    ax.grid(True)
    ax.legend()


def _plot_roc_airspeed_axis(
    ax,
    airspeed_fps,
    roc_fpm,
    max_roc_airspeed_fps,
    max_roc_fpm,
):
    """Draw the ROC airspeed sweep and mark its maximum value."""
    ax.plot(airspeed_fps, roc_fpm, label="Sim Rate of Climb")
    ax.plot(
        max_roc_airspeed_fps,
        max_roc_fpm,
        "o",
        label="Max ROC",
    )
    ax.set_title("Rate of Climb vs Airspeed")
    ax.set_xlabel("Airspeed (ft/s)")
    ax.set_ylabel("Rate of Climb (ft/min)")
    ax.grid(True)
    ax.legend()


def plot_maximum_airspeed(
    altitude_ft,
    airspeed_ktas,
    *,
    aircraft_name,
    throttle_percent,
):
    """Plot maximum true airspeed across pressure altitude."""
    fig, ax = plt.subplots(1, 1, figsize=(7, 5))
    fig.suptitle(
        _title(aircraft_name, "Maximum Airspeed"),
        fontsize=13,
        fontweight="bold",
    )

    _plot_maximum_airspeed_axis(
        ax,
        altitude_ft,
        airspeed_ktas,
        throttle_percent,
    )

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    return fig, ax


def plot_climb_performance(
    reference_altitude_ft,
    reference_roc_fpm,
    simulated_altitude_ft,
    simulated_roc_fpm,
    airspeed_fps,
    roc_fpm,
    max_roc_airspeed_fps,
    max_roc_fpm,
    *,
    aircraft_name,
    reference_name="Reference",
    comparison_temperature_c=None,
):
    """Plot reference and simulated climb performance."""
    fig, (ax_altitude, ax_airspeed) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(
        _title(aircraft_name, "Climb Performance"),
        fontsize=13,
        fontweight="bold",
    )

    _plot_roc_altitude_axis(
        ax_altitude,
        reference_altitude_ft,
        reference_roc_fpm,
        simulated_altitude_ft,
        simulated_roc_fpm,
        reference_name,
        comparison_temperature_c,
    )

    _plot_roc_airspeed_axis(
        ax_airspeed,
        airspeed_fps,
        roc_fpm,
        max_roc_airspeed_fps,
        max_roc_fpm,
    )

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, (ax_altitude, ax_airspeed)


def plot_performance_limits(
    maximum_airspeed_altitude_ft,
    maximum_airspeed_ktas,
    reference_altitude_ft,
    reference_roc_fpm,
    simulated_altitude_ft,
    simulated_roc_fpm,
    roc_airspeed_fps,
    roc_fpm,
    max_roc_airspeed_fps,
    max_roc_fpm,
    *,
    aircraft_name,
    throttle_percent,
    reference_name="Reference",
    comparison_temperature_c=None,
):
    """Plot maximum airspeed and maximum rate-of-climb results in one window."""
    fig, (ax_airspeed, ax_roc_altitude, ax_roc_speed) = plt.subplots(
        1,
        3,
        figsize=(17, 5),
    )
    fig.suptitle(
        _title(aircraft_name, "Performance Limits"),
        fontsize=13,
        fontweight="bold",
    )

    # Maximum true airspeed across the requested altitude range
    _plot_maximum_airspeed_axis(
        ax_airspeed,
        maximum_airspeed_altitude_ft,
        maximum_airspeed_ktas,
        throttle_percent,
    )

    # Maximum rate of climb across altitude compared with reference data
    _plot_roc_altitude_axis(
        ax_roc_altitude,
        reference_altitude_ft,
        reference_roc_fpm,
        simulated_altitude_ft,
        simulated_roc_fpm,
        reference_name,
        comparison_temperature_c,
    )

    # Rate-of-climb airspeed sweep at the selected altitude
    _plot_roc_airspeed_axis(
        ax_roc_speed,
        roc_airspeed_fps,
        roc_fpm,
        max_roc_airspeed_fps,
        max_roc_fpm,
    )

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, (ax_airspeed, ax_roc_altitude, ax_roc_speed)


def plot_aerodynamic_performance(
    drag_airspeed_fps,
    total_drag_lbf,
    induced_drag_lbf,
    parasite_drag_lbf,
    power_airspeed_fps,
    power_required_ft_lbf_s,
    power_available_ft_lbf_s,
    *,
    aircraft_name,
):
    """Plot drag-polar and power curves together."""
    fig, (ax_drag, ax_power) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(
        _title(aircraft_name, "Aerodynamic Performance"),
        fontsize=13,
        fontweight="bold",
    )

    ax_drag.plot(drag_airspeed_fps, total_drag_lbf, label="Total Drag")
    ax_drag.plot(drag_airspeed_fps, induced_drag_lbf, label="Induced Drag")
    ax_drag.plot(drag_airspeed_fps, parasite_drag_lbf, label="Parasite Drag")
    ax_drag.set_xlabel("Velocity (ft/s)")
    ax_drag.set_ylabel("Drag (lbf)")
    ax_drag.set_title(_title(aircraft_name, "Drag Polar"))
    ax_drag.grid(True)
    ax_drag.legend()

    ax_power.plot(
        power_airspeed_fps,
        power_required_ft_lbf_s,
        label="Power Required",
    )
    ax_power.plot(
        power_airspeed_fps,
        power_available_ft_lbf_s,
        label="Power Available",
    )
    ax_power.set_xlabel("Velocity (ft/s)")
    ax_power.set_ylabel("Power (lb\u00b7ft/s)")
    ax_power.set_title(_title(aircraft_name, "Power Curves"))
    ax_power.grid(True)
    ax_power.legend()

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, (ax_drag, ax_power)


def plot_longitudinal_comparison(
    simulation_time_s,
    simulation_states,
    simulation_alpha_rad,
    simulation_elevator_rad,
    *,
    aircraft_name,
    reference_time_s,
    reference_u_fps,
    reference_w_fps,
    reference_q_deg_s,
    reference_theta_deg,
    reference_alpha_deg,
    reference_altitude_ft,
    reference_elevator_deg,
    simulation_name="Simulation",
    reference_name="Reference",
):
    """Compare longitudinal simulation states with reference data."""
    fig, axes = plt.subplots(7, 1, figsize=(9, 10), sharex=True)
    fig.suptitle(
        _title(aircraft_name, "Longitudinal State Comparison"),
        fontsize=13,
        fontweight="bold",
    )

    simulated_series = (
        simulation_states[:, 0],
        simulation_states[:, 1],
        np.rad2deg(simulation_states[:, 2]),
        np.rad2deg(simulation_states[:, 3]),
        np.rad2deg(simulation_alpha_rad),
        simulation_states[:, 4],
        np.rad2deg(simulation_elevator_rad),
    )
    reference_series = (
        reference_u_fps,
        reference_w_fps,
        reference_q_deg_s,
        reference_theta_deg,
        reference_alpha_deg,
        reference_altitude_ft,
        reference_elevator_deg,
    )
    axis_labels = (
        "u (ft/s)",
        "w (ft/s)",
        "Q (deg/s)",
        "theta (deg)",
        "alpha (deg)",
        "Altitude (ft)",
        "Elevator Deflection (deg)",
    )
    quantity_names = (
        "U",
        "W",
        "Q",
        "theta",
        "alpha",
        "alt",
        "Elevator Deflection",
    )

    for index, (ax, simulated, reference, ylabel, quantity) in enumerate(
        zip(
            axes,
            simulated_series,
            reference_series,
            axis_labels,
            quantity_names,
        )
    ):
        ax.plot(simulation_time_s, simulated)
        ax.plot(reference_time_s, reference, linestyle="dashed")
        ax.set_ylabel(ylabel)
        ax.grid(True)
        if index >= 3:
            ax.set_xlabel("Time (s)")
        ax.legend(
            (f"{simulation_name} {quantity}", f"{reference_name} {quantity}"),
            loc="center right",
        )

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    return fig, axes


def plot_six_dof_response(
    simulation_time_s,
    simulation_states,
    *,
    aircraft_name,
):
    """Plot all 12 states from a six-degree-of-freedom simulation."""
    simulation_states = np.asarray(simulation_states, dtype=float)

    # The six-DOF state is [U, V, W, P, Q, R, phi, theta, psi, north, east, altitude]
    if simulation_states.ndim != 2 or simulation_states.shape[1] != 12:
        raise ValueError("simulation_states must have shape (number of times, 12)")

    if len(simulation_time_s) != len(simulation_states):
        raise ValueError("simulation_time_s and simulation_states must have equal lengths")

    # Convert angular rates and Euler angles to degrees for easier interpretation
    plotted_states = (
        simulation_states[:, 0],
        simulation_states[:, 1],
        simulation_states[:, 2],
        np.rad2deg(simulation_states[:, 3]),
        np.rad2deg(simulation_states[:, 4]),
        np.rad2deg(simulation_states[:, 5]),
        np.rad2deg(simulation_states[:, 6]),
        np.rad2deg(simulation_states[:, 7]),
        np.rad2deg(simulation_states[:, 8]),
        simulation_states[:, 9],
        simulation_states[:, 10],
        simulation_states[:, 11],
    )

    state_titles = (
        "Forward Velocity U",
        "Side Velocity V",
        "Vertical Velocity W",
        "Roll Rate P",
        "Pitch Rate Q",
        "Yaw Rate R",
        "Bank Angle phi",
        "Pitch Angle theta",
        "Heading Angle psi",
        "North Position",
        "East Position",
        "Altitude",
    )

    axis_labels = (
        "U (ft/s)",
        "V (ft/s)",
        "W (ft/s)",
        "P (deg/s)",
        "Q (deg/s)",
        "R (deg/s)",
        "phi (deg)",
        "theta (deg)",
        "psi (deg)",
        "North (ft)",
        "East (ft)",
        "Altitude (ft)",
    )

    # Arrange related states into velocity, rate, attitude, and position rows
    fig, axes = plt.subplots(4, 3, figsize=(13, 11), sharex=True)
    fig.suptitle(
        _title(aircraft_name, "Six-DOF State Response"),
        fontsize=13,
        fontweight="bold",
    )

    for index, (ax, state, title, ylabel) in enumerate(
        zip(axes.ravel(), plotted_states, state_titles, axis_labels)
    ):
        ax.plot(simulation_time_s, state)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True)

        # Only the bottom row needs time labels because every column shares x
        if index >= 9:
            ax.set_xlabel("Time (s)")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    return fig, axes


def show_plots():
    """Display all figures created by the plotting functions."""
    plt.show()
