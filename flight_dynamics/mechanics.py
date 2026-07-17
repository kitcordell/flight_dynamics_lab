"""Aircraft drag, power, airspeed, and excess-power performance mechanics."""

import numpy as np
from scipy.optimize import least_squares

from flight_dynamics import conversions
from flight_dynamics.aero_model import drag_coefficient
from flight_dynamics.atmosphere import standard_atmosphere
from flight_dynamics.thrust_model import power_available


#%% Drag Polar

# Assumptions:
    # Steady, unaccelerated, level flight
    # Lift equals aircraft weight
def drag_polar(alt_0, params):
    h = 0.01                              # velocity spacing used to size the array
    C_D_0 = params["C_D_0"]              # parasite drag coefficient
    e = params["e"]                      # Oswald efficiency factor
    W = params["W"]                      # aircraft weight, [lbf]
    S = params["S"]                      # wing area, [ft^2]
    AR = params["AR"]                    # wing aspect ratio

    rho, T, _ = standard_atmosphere(alt_0)
    V_S = conversions.ias2tas(conversions.kts2fps(params["V_S"]), alt_0, T)
    V_ne = conversions.ias2tas(conversions.kts2fps(params["V_ne"]), alt_0, T)
    N = int(np.round((V_ne - V_S) / h))
    V = np.linspace(V_S, V_ne, N)         # true airspeed range, [ft/s]

    qbar = 0.5 * rho * V**2               # dynamic pressure, [lb/ft^2]
    C_L = W / (qbar * S)                  # lift coefficient where lift equals weight

    C_D_i = C_L**2 / (np.pi * e * AR)     # induced drag coefficient
    C_D = C_D_0 + C_D_i                   # total drag coefficient

    D_p = qbar * S * C_D_0                # parasite drag, [lbf]
    D_i = qbar * S * C_D_i                # induced drag, [lbf]
    D = qbar * S * C_D                    # total drag, [lbf]

    return V, D, D_i, D_p


#%% Power Required and Power Available

def power_required(alt_0, params):
    # Power required is drag multiplied by true airspeed
    V, D, D_i, D_p = drag_polar(alt_0, params)

    P_req = D * V                         # total power required, [ft*lbf/s]
    P_i = D_i * V                         # induced power required, [ft*lbf/s]
    P_p = D_p * V                         # parasite power required, [ft*lbf/s]

    return V, P_req, P_i, P_p


def power_curves(alt, throttle, params):
    # Calculate power required across the full airspeed range
    V, P_req, P_i, P_p = power_required(alt, params)

    # Power available is constant with airspeed in the current engine model
    P_A = power_available(throttle, alt, params)
    P_A_curve = np.full_like(V, P_A)

    return V, P_req, P_i, P_p, P_A_curve


#%% Maximum Airspeed

def velocity_max(alt, throttle, params, x0):
    # Maximum airspeed occurs where power required equals power available
    rho, _, _ = standard_atmosphere(alt)
    P_A = power_available(throttle, alt, params)

    def residual(V):
        q = 0.5 * rho * V[0]**2
        C_L = params["W"] / (q * params["S"])
        C_D, _ = drag_coefficient(C_L, params)
        P_req = q * params["S"] * C_D * V[0]
        return [P_req - P_A]

    sol = least_squares(residual, x0)
    return sol.x[0]


#%% Excess-Power Rate of Climb

def rate_of_climb_excess_power(V, alt, throttle, params):
    """
    Calculate rate of climb from excess power.

    ROC = (P_available - P_required) / W

    Inputs:
        V: true airspeed [ft/s], scalar or array
        alt: pressure altitude [ft]
        throttle: throttle setting [0 to 1]
        params: aircraft parameter dictionary

    Returns:
        ROC [ft/s], scalar or array matching V
    """
    V = np.asarray(V, dtype=float)
    V_curve, P_req, _, _ = power_required(alt, params)
    P_req_at_V = np.interp(V, V_curve, P_req)
    P_avail = power_available(throttle, alt, params)
    excess_power = P_avail - P_req_at_V

    return excess_power / params["W"]


def roc_speed_sweep(alt, throttle, params):
    """
    Calculate ROC across the drag-polar speed range at one altitude.

    Returns:
        V: true airspeed array [ft/s]
        ROC: rate of climb array [ft/s]
        P_req: power required array [ft*lbf/s]
        P_avail: power available array [ft*lbf/s]
    """
    V, P_req, _, _ = power_required(alt, params)
    P_avail = power_available(throttle, alt, params)
    ROC = (P_avail - P_req) / params["W"]
    P_avail_curve = np.full_like(V, P_avail)

    return V, ROC, P_req, P_avail_curve


def max_roc_excess_power(alt, throttle, params):
    """
    Find maximum ROC at one altitude using excess power.

    Returns:
        V_max_roc: true airspeed at max ROC [ft/s]
        ROC_max: maximum rate of climb [ft/s]
        V: true airspeed array [ft/s]
        ROC: rate of climb array [ft/s]
    """
    V, ROC, _, _ = roc_speed_sweep(alt, throttle, params)
    max_index = int(np.argmax(ROC))

    return V[max_index], ROC[max_index], V, ROC


def max_roc_vs_altitude(altitudes, throttle, params):
    """
    Calculate maximum excess-power ROC across multiple altitudes.

    Returns:
        altitudes: altitude array [ft]
        V_max_roc: best-rate speed array [ft/s]
        ROC_max: maximum rate of climb array [ft/s]
    """
    altitudes = np.asarray(altitudes, dtype=float)
    V_max_roc = np.zeros_like(altitudes)
    ROC_max = np.zeros_like(altitudes)

    for i, alt in enumerate(altitudes):
        V_max_roc[i], ROC_max[i], _, _ = max_roc_excess_power(
            alt,
            throttle,
            params,
        )

    return altitudes, V_max_roc, ROC_max


if __name__ == "__main__":
    from flight_dynamics.c172_params import params

    altitude = 4000.0
    throttle = 1.0
    V_best, ROC_best, _, _ = max_roc_excess_power(altitude, throttle, params)

    print("Maximum ROC from excess power")
    print(f"Altitude: {altitude:.0f} ft")
    print(f"Throttle: {throttle:.2f}")
    print(
        f"Best ROC speed: {V_best:.2f} ft/s | "
        f"{conversions.fps2kts(V_best):.2f} kt"
    )
    print(
        f"Max ROC: {ROC_best:.2f} ft/s | "
        f"{ROC_best * 60.0:.0f} ft/min"
    )
