import numpy as np




#%% Max Airspeed
    # Calculates the maximum airspeed by 
def airspeed_max(throttle,alt_range, params, plot=False, verbose=False):


    ## Calculates max speed for a range of altitudes
    if len(alt_range) == 3:     # Unpack the altitude range array
        alt_max = alt_range[1]  # maximum altitude for maxiumum velocity graph[ft]
        alt_step = alt_range[2]  # Steps to run calculations in between min and max altitudes [ft]
        alt_0 = alt_range[0]   # minimum altitude for maxiumum velocity graph [ft]
        
        alt_array = np.arrange(alt_0, alt_max, alt_step) # create altitude arrau

    # 
    if len(alt_range) !=3 and len(alt_range) != 1:
        print('Must either specify a single altitude or and altitude range and step(alt_0, alt_max, alt_step)')
    
    
    else :
        alt_0 = alt_range

        
    


    # throttle = 0.75
    # alt_0 = 0
    # alt_max = 12000
    # alt_array = np.arange(
    #     alt_0,
    #     alt_max + 1000,
    #     1000,
    # )
    max_speed_array = np.zeros_like(alt_array, dtype=float)

    V_max_guess = 300.0
    for i, altitude in enumerate(alt_array):
        V_max = velocity_max(altitude, throttle, params, V_max_guess)
        max_speed_array[i] = V_max
        V_max_guess = V_max

    cruise_poh = pd.read_csv(DATA_DIR / "c172_cruise_performance_visible_rows.csv")
    target_mcp_percent = throttle * 100.0

    poh_cruise_rows = []
    for _, group in cruise_poh.groupby("pressure_altitude_ft"):
        closest_index = (group["mcp_percent"] - target_mcp_percent).abs().idxmin()
        poh_cruise_rows.append(cruise_poh.loc[closest_index])

    poh_cruise = pd.DataFrame(poh_cruise_rows).sort_values("pressure_altitude_ft")
    poh_altitudes = poh_cruise["pressure_altitude_ft"].to_numpy(dtype=float)
    poh_tas = poh_cruise["ktas"].to_numpy(dtype=float)
    poh_mcp = poh_cruise["mcp_percent"].to_numpy(dtype=float)
    estimated_tas_at_poh_altitudes = np.interp(
        poh_altitudes,
        alt_array,
        conv.fps2kts(max_speed_array),
    )

    print(
        "Max Velocity @ 75% throttle, sea level: "
        f"{max_speed_array[0]:.2f} ft/s | {conv.fps2kts(max_speed_array[0]):.2f} kts"
    )
    print("\nCruise TAS comparison using nearest POH MCP to 75%:")
    for altitude, mcp, poh_ktas, estimated_ktas in zip(
        poh_altitudes,
        poh_mcp,
        poh_tas,
        estimated_tas_at_poh_altitudes,
    ):
        print(
            f"{altitude:5.0f} ft | POH {mcp:4.0f}% MCP: {poh_ktas:6.1f} kt TAS | "
            f"Estimated 75%: {estimated_ktas:6.1f} kt TAS"
        )

    fig_speed, ax_speed_alt = plt.subplots(1, 1, figsize=(7, 5))
    fig_speed.suptitle("C172 Maximum Airspeed", fontsize=13, fontweight="bold")
    ax_speed_alt.plot(
        alt_array,
        conv.fps2kts(max_speed_array),
        color=AERO_COLORS["cyan"],
        marker="o",
        markeredgecolor=AERO_COLORS["text"],
        label="Estimated 75% Throttle",
    )
    ax_speed_alt.plot(poh_altitudes,poh_tas,color=AERO_COLORS["amber"],markeredgecolor=AERO_COLORS["text"],linestyle="--",label="POH Closest to 75% MCP",)
    ax_speed_alt.set_title("TAS vs Altitude")
    ax_speed_alt.set_xlabel("Pressure Altitude (ft)")
    ax_speed_alt.set_ylabel("Maximum Airspeed (kt TAS)")
    ax_speed_alt.legend()
    style_axes(ax_speed_alt)
    fig_speed.tight_layout(rect=[0, 0, 1, 0.92])

#%% Imports
from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flight_dynamics.c172_params import params, tf, dt, alt_0
from flight_dynamics.control_inputs import elevator_deflection
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.drag_polar import drag_polar, power_curves, velocity_max
from flight_dynamics.trim_solver import longitudinal_trim, max_ROC
from flight_dynamics import conversions as conv
from flight_dynamics.integrators import RK4
from flight_dynamics.plot_theme import AERO_COLORS, set_aerospace_theme, style_axes

DATA_DIR = ROOT_DIR / "data"


throttle = 0.75 # [%]
min_alt = 0 # minimum altitude for maxiumum velocity graph [ft]
max_alt = 12000 # maximum altitude for maxiumum velocity graph[ft]
alt_step = 1000 # [ft]
airspeed_max(throttle, [min_alt, max_alt, alt_step], params)
