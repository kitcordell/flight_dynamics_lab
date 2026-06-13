


#%% Max Airspeed
def airspeed_max(throttle,):
    max_speed_throttle = 0.75
    max_speed_min_alt = 0
    max_speed_max_alt = 12000
    max_speed_alt_array = np.arange(
        max_speed_min_alt,
        max_speed_max_alt + 1000,
        1000,
    )
    max_speed_array = np.zeros_like(max_speed_alt_array, dtype=float)

    V_max_guess = 300.0
    for i, altitude in enumerate(max_speed_alt_array):
        V_max = velocity_max(altitude, max_speed_throttle, params, V_max_guess)
        max_speed_array[i] = V_max
        V_max_guess = V_max

    cruise_poh = pd.read_csv(DATA_DIR / "c172_cruise_performance_visible_rows.csv")
    target_mcp_percent = max_speed_throttle * 100.0

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
        max_speed_alt_array,
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
        max_speed_alt_array,
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