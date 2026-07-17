"""Load and prepare external flight data for model comparisons."""

from pathlib import Path

import numpy as np
import pandas as pd

from flight_dynamics import conversions


XPLANE_LONGITUDINAL_COLUMNS = (
    "_totl,_time ",
    "pitch,__deg ",
    "elev1,__deg .1",
    "alpha,__deg ",
    "p-alt,ftMSL ",
    "Vtrue,_ktas ",
    "____Q,deg/s ",
)

RATE_OF_CLIMB_REFERENCE_COLUMNS = {
    "press_alt_ft": "pressure_altitude_ft",
    "IAS": "indicated_airspeed_kt",
    "fpm_M20C": "roc_fpm_minus_20c",
    "fpm_0C": "roc_fpm_0c",
    "fpm_20C": "roc_fpm_20c",
    "fpm_40C": "roc_fpm_40c",
}


# Loads POH or flight-test rate-of-climb data used for model comparisons
# file_path: path to a comma-delimited rate-of-climb data file
def load_rate_of_climb_reference_data(file_path):
    """Load rate-of-climb reference data and give its columns clean names."""
    file_path = Path(file_path)

    # Reference rate-of-climb tables are stored as comma-delimited files
    reference_data = pd.read_csv(
        file_path,
        sep=",",
        engine="python",
        skipinitialspace=True,
    )

    # Make sure the file contains every altitude, airspeed, and ROC column
    missing_columns = [
        column
        for column in RATE_OF_CLIMB_REFERENCE_COLUMNS
        if column not in reference_data.columns
    ]
    if missing_columns:
        missing = ", ".join(missing_columns)
        raise ValueError(f"Rate-of-climb reference columns are missing: {missing}")

    # Keep only the columns used by the project and remove source-specific names
    prepared_data = reference_data[
        list(RATE_OF_CLIMB_REFERENCE_COLUMNS)
    ].rename(columns=RATE_OF_CLIMB_REFERENCE_COLUMNS)

    return prepared_data.reset_index(drop=True)


# Loads the X-Plane states used for the longitudinal model comparison
# file_path: path to the X-Plane pipe-delimited data file
# row_start and row_end: row range containing the maneuver to compare
# time_reference_index: selected row used as time zero
def load_xplane_longitudinal_data(
    file_path,
    row_start=None,
    row_end=None,
    time_reference_index=0,
):
    """Load X-Plane longitudinal states and convert them into model units."""
    file_path = Path(file_path)

    # X-Plane text exports use pipe separators and padded column names
    data_xplane = pd.read_csv(
        file_path,
        sep="|",
        engine="python",
        skipinitialspace=True,
    )

    # Make sure the selected export contains every state needed by the plots
    missing_columns = [
        column
        for column in XPLANE_LONGITUDINAL_COLUMNS
        if column not in data_xplane.columns
    ]
    if missing_columns:
        missing = ", ".join(missing_columns)
        raise ValueError(f"X-Plane data columns are missing: {missing}")

    # Select only the maneuver segment requested by the calling script
    data_xplane = data_xplane.iloc[row_start:row_end].copy()
    if data_xplane.empty:
        raise ValueError("The selected X-Plane row range is empty")

    if not 0 <= time_reference_index < len(data_xplane):
        raise IndexError("time_reference_index is outside the selected row range")

    # Subtract the selected X-Plane time so the comparison starts near zero
    time_xplane = data_xplane["_totl,_time "]
    time_xplane = time_xplane - time_xplane.iloc[time_reference_index]

    # Read the measured longitudinal states from the X-Plane export
    pitch_deg = data_xplane["pitch,__deg "]
    elevator_deg = data_xplane["elev1,__deg .1"]
    alpha_deg = data_xplane["alpha,__deg "]
    altitude_ft = data_xplane["p-alt,ftMSL "]
    pitch_rate_deg_s = data_xplane["____Q,deg/s "]

    # Resolve true airspeed into the body-axis forward and vertical velocities
    alpha_rad = np.deg2rad(alpha_deg)
    U_fps = conversions.kts2fps(
        data_xplane["Vtrue,_ktas "] * np.cos(alpha_rad)
    )
    W_fps = conversions.kts2fps(
        data_xplane["Vtrue,_ktas "] * np.sin(alpha_rad)
    )

    # Return clean names and units so the rest of the project does not depend
    # on the padded column names used by X-Plane
    prepared_data = pd.DataFrame({
        "time_s": time_xplane,
        "u_fps": U_fps,
        "w_fps": W_fps,
        "q_deg_s": pitch_rate_deg_s,
        "pitch_deg": pitch_deg,
        "alpha_deg": alpha_deg,
        "altitude_ft": altitude_ft,
        "elevator_deg": elevator_deg,
    })

    return prepared_data.reset_index(drop=True)
