import numpy as np

from flight_dynamics.drag_polar import power_required
from flight_dynamics.thrust_model import power_available
from flight_dynamics import conversions


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
        V_max_roc[i], ROC_max[i], _, _ = max_roc_excess_power(alt, throttle, params)

    return altitudes, V_max_roc, ROC_max


if __name__ == "__main__":
    from flight_dynamics.c172_params import params

    altitude = 4000.0
    throttle = 1.0
    V_best, ROC_best, _, _ = max_roc_excess_power(altitude, throttle, params)

    print("Maximum ROC from excess power")
    print(f"Altitude: {altitude:.0f} ft")
    print(f"Throttle: {throttle:.2f}")
    print(f"Best ROC speed: {V_best:.2f} ft/s | {conversions.fps2kts(V_best):.2f} kt")
    print(f"Max ROC: {ROC_best:.2f} ft/s | {ROC_best * 60.0:.0f} ft/min")
