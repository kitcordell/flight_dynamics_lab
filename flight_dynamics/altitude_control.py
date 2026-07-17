"""Look-ahead altitude control for the nonlinear longitudinal model."""

from dataclasses import dataclass, field

import numpy as np


def flight_path_angle_from_state(state):
    """Return gamma, alpha, and airspeed from [U, W, Q, theta, h]."""
    state = np.asarray(state, dtype=float)
    if state.shape != (5,):
        raise ValueError("longitudinal state must have shape (5,)")

    U, W, _, theta, _ = state
    airspeed = float(np.hypot(U, W))
    if airspeed <= 0.0:
        raise ValueError("airspeed must be greater than zero")

    alpha = float(np.arctan2(W, U))
    gamma = float(theta - alpha)
    return gamma, alpha, airspeed


def lookahead_flight_path_angle_command(
    target_altitude_ft,
    state,
    lookahead_time_s,
    *,
    gamma_limit_rad=np.deg2rad(6.0),
    minimum_lookahead_distance_ft=25.0,
):
    """Point the velocity vector toward a target altitude at a look-ahead point."""
    if lookahead_time_s <= 0.0:
        raise ValueError("lookahead_time_s must be greater than zero")
    if minimum_lookahead_distance_ft <= 0.0:
        raise ValueError("minimum_lookahead_distance_ft must be greater than zero")
    if gamma_limit_rad <= 0.0:
        raise ValueError("gamma_limit_rad must be greater than zero")

    _, _, airspeed = flight_path_angle_from_state(state)
    altitude_ft = float(np.asarray(state, dtype=float)[4])
    distance_ft = max(
        airspeed * lookahead_time_s,
        minimum_lookahead_distance_ft,
    )
    gamma_command_rad = float(
        np.clip(
            np.arctan2(float(target_altitude_ft) - altitude_ft, distance_ft),
            -gamma_limit_rad,
            gamma_limit_rad,
        )
    )
    return gamma_command_rad, distance_ft


@dataclass
class LookaheadAltitudeController:
    """Altitude -> flight-path angle -> pitch -> elevator controller.

    All angles are radians. The default elevator sign is -1 because the current
    C172 model uses negative C_m_delta_e: a negative elevator command pitches up.
    """

    target_altitude_ft: float
    trim_control: np.ndarray
    trim_pitch_rad: float
    lookahead_time_s: float = 5.0
    gamma_kp: float = 1.0
    gamma_ki: float = 0.0
    pitch_kp: float = 1.0
    pitch_ki: float = 0.0
    pitch_kd: float = 0.5
    gamma_limit_rad: float = np.deg2rad(6.0)
    pitch_correction_limit_rad: float = np.deg2rad(10.0)
    elevator_limits_rad: tuple = (-np.inf, np.inf)
    minimum_lookahead_distance_ft: float = 25.0
    elevator_to_pitch_sign: float = -1.0
    integral_limit_rad_s: float = np.deg2rad(30.0)
    _gamma_integral: float = field(default=0.0, init=False, repr=False)
    _pitch_integral: float = field(default=0.0, init=False, repr=False)
    _last_time: float | None = field(default=None, init=False, repr=False)
    last_diagnostics: dict = field(default_factory=dict, init=False)

    def __post_init__(self):
        self.trim_control = np.asarray(self.trim_control, dtype=float)
        if self.trim_control.shape != (2,):
            raise ValueError("trim_control must contain [throttle, elevator]")
        if self.elevator_to_pitch_sign not in (-1.0, 1.0):
            raise ValueError("elevator_to_pitch_sign must be -1.0 or 1.0")
        if self.elevator_limits_rad[0] >= self.elevator_limits_rad[1]:
            raise ValueError("elevator limits must satisfy lower < upper")

    def reset(self):
        self._gamma_integral = 0.0
        self._pitch_integral = 0.0
        self._last_time = None
        self.last_diagnostics = {}

    def __call__(self, t, state):
        state = np.asarray(state, dtype=float)
        gamma_rad, alpha_rad, airspeed_fps = flight_path_angle_from_state(state)
        _, _, pitch_rate_rad_s, pitch_rad, altitude_ft = state
        dt = 0.0 if self._last_time is None else max(float(t) - self._last_time, 0.0)
        self._last_time = float(t)

        gamma_command_rad, distance_ft = lookahead_flight_path_angle_command(
            self.target_altitude_ft,
            state,
            self.lookahead_time_s,
            gamma_limit_rad=self.gamma_limit_rad,
            minimum_lookahead_distance_ft=self.minimum_lookahead_distance_ft,
        )
        gamma_error_rad = gamma_command_rad - gamma_rad
        if self.gamma_ki and dt:
            self._gamma_integral = float(np.clip(
                self._gamma_integral + gamma_error_rad * dt,
                -self.integral_limit_rad_s,
                self.integral_limit_rad_s,
            ))

        pitch_correction_rad = float(np.clip(
            self.gamma_kp * gamma_error_rad + self.gamma_ki * self._gamma_integral,
            -self.pitch_correction_limit_rad,
            self.pitch_correction_limit_rad,
        ))
        pitch_command_rad = float(self.trim_pitch_rad + pitch_correction_rad)
        pitch_error_rad = pitch_command_rad - pitch_rad
        if self.pitch_ki and dt:
            self._pitch_integral = float(np.clip(
                self._pitch_integral + pitch_error_rad * dt,
                -self.integral_limit_rad_s,
                self.integral_limit_rad_s,
            ))

        elevator_correction_rad = self.elevator_to_pitch_sign * (
            self.pitch_kp * pitch_error_rad
            + self.pitch_ki * self._pitch_integral
            - self.pitch_kd * pitch_rate_rad_s
        )
        elevator_rad = float(np.clip(
            self.trim_control[1] + elevator_correction_rad,
            *self.elevator_limits_rad,
        ))
        throttle = float(np.clip(self.trim_control[0], 0.0, 1.0))

        self.last_diagnostics = {
            "altitude_ft": float(altitude_ft),
            "airspeed_fps": airspeed_fps,
            "alpha_rad": alpha_rad,
            "gamma_rad": gamma_rad,
            "gamma_command_rad": gamma_command_rad,
            "lookahead_distance_ft": distance_ft,
            "pitch_command_rad": pitch_command_rad,
            "elevator_rad": elevator_rad,
        }
        return np.array([throttle, elevator_rad])
