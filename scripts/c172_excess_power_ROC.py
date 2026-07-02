#%% Imports
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flight_dynamics.c172_params import params
from flight_dynamics.rate_of_climb_solver import max_roc_vs_altitude, roc_speed_sweep
from flight_dynamics.trim_solver import max_ROC
from flight_dynamics import conversions as conv
from flight_dynamics.plot_theme import AERO_COLORS, set_aerospace_theme, style_axes

DATA_DIR = ROOT_DIR / "data"

set_aerospace_theme()


#%% Excess Power ROC Calculations
throttle = 1.0
speed_sweep_alt = 4000.0
altitudes = np.arange(0.0, 12000.0 + 1000.0, 1000.0)

altitudes, V_max_roc_ep, ROC_max_ep = max_roc_vs_altitude(altitudes, throttle, params)
V_sweep_ep, ROC_sweep_ep, P_req, P_avail = roc_speed_sweep(speed_sweep_alt, throttle, params)
excess_power = P_avail - P_req

speed_sweep_max_index_ep = int(np.argmax(ROC_sweep_ep))
V_sweep_max_roc_ep = V_sweep_ep[speed_sweep_max_index_ep]
ROC_sweep_max_ep = ROC_sweep_ep[speed_sweep_max_index_ep]


#%% Dynamics ROC Calculations
# initial guess = [theta, delta_e, gamma]
x0_dynamics = np.array([
    np.deg2rad(5.0),
    np.deg2rad(-2.0),
    np.deg2rad(3.0),
])

V_max_roc_dyn = np.zeros_like(altitudes)
ROC_max_dyn = np.zeros_like(altitudes)

for i, altitude in enumerate(altitudes):
    trim_target = [throttle, 90.0, altitude]
    _, _, V_max_roc_dyn[i], ROC_max_dyn[i] = max_ROC(
        x0_dynamics,
        trim_target,
        params,
        verbose=False,
    )

trim_target = [throttle, 90.0, speed_sweep_alt]
V_sweep_dyn, ROC_sweep_dyn, V_sweep_max_roc_dyn, ROC_sweep_max_dyn = max_ROC(
    x0_dynamics,
    trim_target,
    params,
    verbose=False,
)


#%% POH Data
roc_poh = pd.read_csv(
    DATA_DIR / "c172_roc.csv",
    sep=",",
    engine="python",
    skipinitialspace=True,
)

poh_roc_alt = roc_poh["press_alt_ft"]
poh_roc_20C = roc_poh["fpm_20C"]


#%% Excess Power Plots
fig, axs = plt.subplots(2, 2, figsize=(13, 8))
fig.suptitle("C172 Rate of Climb: Excess Power vs Dynamics Trim", fontsize=14, fontweight="bold")

ax_alt, ax_best_speed, ax_speed, ax_power = axs.ravel()

ax_alt.plot(poh_roc_alt, poh_roc_20C, label="POH ROC @ 20C", color=AERO_COLORS["amber"])
ax_alt.plot(altitudes, ROC_max_ep * 60.0, label="Excess Power ROC", color=AERO_COLORS["cyan"])
ax_alt.plot(
    altitudes,
    ROC_max_dyn * 60.0,
    label="Dynamics Trim ROC",
    color=AERO_COLORS["blue"],
    linestyle="--",
)
ax_alt.set_title("Maximum ROC vs Altitude")
ax_alt.set_xlabel("Pressure Altitude (ft)")
ax_alt.set_ylabel("Rate of Climb (ft/min)")
ax_alt.legend()
style_axes(ax_alt)

ax_best_speed.plot(
    altitudes,
    conv.fps2kts(V_max_roc_ep),
    label="Excess Power",
    color=AERO_COLORS["green"],
)
ax_best_speed.plot(
    altitudes,
    conv.fps2kts(V_max_roc_dyn),
    label="Dynamics Trim",
    color=AERO_COLORS["blue"],
    linestyle="--",
)
ax_best_speed.set_title("Best-Rate Speed vs Altitude")
ax_best_speed.set_xlabel("Pressure Altitude (ft)")
ax_best_speed.set_ylabel("Best ROC Speed (kt TAS)")
ax_best_speed.legend()
style_axes(ax_best_speed)

ax_speed.plot(V_sweep_ep, ROC_sweep_ep * 60.0, label="Excess Power ROC", color=AERO_COLORS["green"])
ax_speed.plot(
    V_sweep_dyn,
    ROC_sweep_dyn * 60.0,
    label="Dynamics Trim ROC",
    color=AERO_COLORS["blue"],
    linestyle="--",
)
ax_speed.plot(
    V_sweep_max_roc_ep,
    ROC_sweep_max_ep * 60.0,
    "o",
    color=AERO_COLORS["red"],
    markeredgecolor=AERO_COLORS["text"],
    label="Excess Power Max",
)
ax_speed.plot(
    V_sweep_max_roc_dyn,
    ROC_sweep_max_dyn * 60.0,
    "s",
    color=AERO_COLORS["magenta"],
    markeredgecolor=AERO_COLORS["text"],
    label="Dynamics Max",
)
ax_speed.set_title(f"ROC vs Airspeed @ {speed_sweep_alt:.0f} ft")
ax_speed.set_xlabel("True Airspeed (ft/s)")
ax_speed.set_ylabel("Rate of Climb (ft/min)")
ax_speed.legend()
style_axes(ax_speed)

ax_power.plot(V_sweep_ep, P_req / 550.0, label="Power Required", color=AERO_COLORS["cyan"])
ax_power.plot(V_sweep_ep, P_avail / 550.0, label="Power Available", color=AERO_COLORS["amber"])
ax_power.fill_between(
    V_sweep_ep,
    P_req / 550.0,
    P_avail / 550.0,
    where=excess_power >= 0.0,
    color=AERO_COLORS["green"],
    alpha=0.18,
    label="Excess Power",
)
ax_power.set_title(f"Power Margin @ {speed_sweep_alt:.0f} ft")
ax_power.set_xlabel("True Airspeed (ft/s)")
ax_power.set_ylabel("Power (hp)")
ax_power.legend()
style_axes(ax_power)

fig.tight_layout(rect=[0, 0, 1, 0.94])
plt.show()

print("ROC comparison summary")
print(f"Altitude: {speed_sweep_alt:.0f} ft")
print(f"Throttle: {throttle:.2f}")
print(
    "Excess power best ROC speed: "
    f"{V_sweep_max_roc_ep:.2f} ft/s | {conv.fps2kts(V_sweep_max_roc_ep):.2f} kt TAS"
)
print(f"Excess power max ROC: {ROC_sweep_max_ep:.2f} ft/s | {ROC_sweep_max_ep * 60.0:.0f} ft/min")
print(
    "Dynamics trim best ROC speed: "
    f"{V_sweep_max_roc_dyn:.2f} ft/s | {conv.fps2kts(V_sweep_max_roc_dyn):.2f} kt TAS"
)
print(f"Dynamics trim max ROC: {ROC_sweep_max_dyn:.2f} ft/s | {ROC_sweep_max_dyn * 60.0:.0f} ft/min")
