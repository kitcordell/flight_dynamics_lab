# Project Structure

This repository now uses one small Python package for reusable flight-dynamics code and keeps runnable scripts, examples, notebooks, data, figures, and docs separate.

## High-Level Layout

```text
flight_dynamics_lab/
├── flight_dynamics/
├── scripts/
├── examples/
├── notebooks/
├── data/
├── figures/
├── docs/
├── README.md
└── requirements.txt
```

## Main Workflow

The current C172 longitudinal simulation workflow runs through these modules:

1. Parameters and constants: `flight_dynamics/c172_params.py`, `flight_dynamics/constants.py`
2. Atmosphere and conversions: `flight_dynamics/atmosphere.py`, `flight_dynamics/conversions.py`
3. Aerodynamics and propulsion: `flight_dynamics/aero_model.py`, `flight_dynamics/thrust_model.py`
4. Nonlinear equations of motion: `flight_dynamics/longitudinal_dynamics.py`
5. Trim solving: `flight_dynamics/trim_solver.py`
6. Time integration: `flight_dynamics/integrators.py`
7. Analysis and plotting entry point: `scripts/c172_simulation.py`

## Core Package

- `flight_dynamics/__init__.py`
  - Marks the reusable code as one importable package.

- `flight_dynamics/c172_params.py`
  - Central C172 parameter dictionary and simulation timing values.

- `flight_dynamics/constants.py`
  - Shared physical constants in imperial units.

- `flight_dynamics/control_inputs.py`
  - Elevator input scheduling functions.

- `flight_dynamics/atmosphere.py`
  - Standard-atmosphere density, temperature, and pressure calculation.

- `flight_dynamics/conversions.py`
  - Unit conversions, including knots, horsepower, and IAS-to-TAS helpers.

- `flight_dynamics/speed_of_sound.py`
  - Speed-of-sound helper used by airspeed conversion.

- `flight_dynamics/axis_transformations.py`
  - Body-axis and velocity-axis conversion helpers.

- `flight_dynamics/aero_model.py`
  - Lift, drag, induced drag, and pitching-moment coefficient functions.

- `flight_dynamics/thrust_model.py`
  - Naturally aspirated piston-engine available-power and thrust helpers.

- `flight_dynamics/longitudinal_dynamics.py`
  - Core nonlinear longitudinal equations of motion.

- `flight_dynamics/trim_solver.py`
  - Level and climb trim solvers plus maximum-rate-of-climb sweep.

- `flight_dynamics/drag_polar.py`
  - Drag-polar, power-required, power-available, and maximum-speed helpers.

- `flight_dynamics/rate_of_climb_solver.py`
  - Excess-power rate-of-climb helpers used by the alternate ROC script.

- `flight_dynamics/integrators.py`
  - Euler, RK2, and RK4 integrators.

- `flight_dynamics/plot_theme.py`
  - Plot styling used by the scripts.

## Entry Points

- `scripts/c172_simulation.py`
  - Main working script for the C172 project.
  - Generates max-speed, rate-of-climb, drag-polar, trim, and nonlinear longitudinal comparison plots.
  - Loads comparison data from `data/`.

- `scripts/c172_excess_power_ROC.py`
  - Alternate excess-power rate-of-climb analysis script.

## Side Studies

The `examples/` folder holds exploratory or classroom-style scripts that are not part of the core package. This includes pendulum examples, thin-airfoil/lifting-line studies, and unfinished longitudinal/performance scratch files.

## Data and Outputs

- `data/`
  - CSV, ODS, and text data used by scripts and notebooks.

- `figures/`
  - Saved output figures.

- `notebooks/`
  - Notebook versions or supporting notebook assets.

## Suggested Reading Order

1. `README.md`
2. `flight_dynamics/c172_params.py`
3. `flight_dynamics/aero_model.py`
4. `flight_dynamics/longitudinal_dynamics.py`
5. `flight_dynamics/trim_solver.py`
6. `flight_dynamics/integrators.py`
7. `scripts/c172_simulation.py`
