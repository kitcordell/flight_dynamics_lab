import numpy as np


#%% Elevator Deflection Function
def elevator_deflection(t, delta_e):
    # Accept one time value during integration or a full time array for logging
    time = np.asarray(t, dtype=float)
    elevator_input = np.asarray(delta_e, dtype=float)

    # Apply the elevator pulse only inside the requested maneuver window
    pulse_active = (time > 5.6) & (time < 6.6)
    applied_elevator = elevator_input + np.where(
        pulse_active,
        np.deg2rad(-2.164),
        0.0,
    )

    # Dynamics calls use scalars, while logging calls return an array
    if applied_elevator.ndim == 0:
        return applied_elevator.item()

    return applied_elevator


def neutral_elevator_deflection(t, delta_e):
    return delta_e


def neutral_aileron_deflection(t, delta_a):
    # Hold the commanded aileron deflection constant
    return delta_a


def neutral_rudder_deflection(t, delta_r):
    # Hold the commanded rudder deflection constant
    return delta_r


def PD_elevator(theta, Q):
    K_p = 0.00
    K_d = 0.0
    theta_desired = np.deg2rad(3.0)

    # Positive error means aircraft pitch is above target.
    error = theta - theta_desired

    # Since positive elevator gives nose-down moment in your model:
    delta_e = K_p * error - K_d * Q

    if delta_e < np.deg2rad(-13):
        delta_e = np.deg2rad(-23)

    elif delta_e > np.deg2rad(28):
        delta_e = np.deg2rad(28)


    return delta_e
