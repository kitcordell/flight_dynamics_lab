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
7. Aircraft-independent figure construction: `flight_dynamics/aircraft_plotting.py`
8. Analysis entry point: `scripts/c172_simulation.py`

## Core Package

- `flight_dynamics/__init__.py`
  - Marks the reusable code as one importable package.

- `flight_dynamics/c172_params.py`
  - Central C172 parameter dictionary and simulation timing values.

- `flight_dynamics/constants.py`
  - Shared physical constants in imperial units.

- `flight_dynamics/control_inputs.py`
  - Selectable elevator, aileron, and rudder input functions.

- `flight_dynamics/atmosphere.py`
  - Standard-atmosphere density, temperature, pressure, and speed-of-sound calculations.

- `flight_dynamics/conversions.py`
  - Unit conversions, including knots, horsepower, and IAS-to-TAS helpers.

- `flight_dynamics/external_data.py`
  - Loads external comparison data and converts source-specific columns into model units.
  - Contains the X-Plane longitudinal and rate-of-climb reference-data loaders.

- `flight_dynamics/axis_transformations.py`
  - Body-axis and velocity-axis conversion helpers.

- `flight_dynamics/aero_model.py`
  - Longitudinal and lateral-directional aerodynamic coefficient functions.

- `flight_dynamics/thrust_model.py`
  - Naturally aspirated piston-engine available-power and thrust helpers.

- `flight_dynamics/longitudinal_dynamics.py`
  - Core nonlinear longitudinal equations of motion.

- `flight_dynamics/lateral_dynamics.py`
  - Nonlinear lateral-directional equations of motion.
  - Holds the longitudinal states at values supplied by an explicit longitudinal trim state.

- `flight_dynamics/six_dof_dynamics.py`
  - Full nonlinear 12-state rigid-body equations of motion.
  - Uses the shared longitudinal and lateral-directional aerodynamic coefficient functions.

- `flight_dynamics/trim_solver.py`
  - Longitudinal, lateral-directional, and full six-DOF straight-flight trim solvers.
  - The lateral solver trims side velocity, bank angle, aileron, and rudder.
  - Solves the six-DOF control inputs, aerodynamic angles, bank angle, and pitch angle.
  - Applies aircraft control limits, raw-residual convergence validation, and
    bound-proximity diagnostics.

- `flight_dynamics/mechanics.py`
  - Drag-polar, power-required, power-available, and single-altitude maximum-speed helpers.
  - Excess-power rate-of-climb calculations and altitude/speed sweeps.

- `flight_dynamics/performance_solver.py`
  - Maximum-airspeed calculations for a single altitude or an altitude range.
  - Dynamics-trim rate-of-climb calculations and airspeed/altitude sweeps.
  - Returns numerical results and can optionally pass them to the general plotting module.

- `flight_dynamics/integrators.py`
  - Euler, RK2, and RK4 integrators.

- `flight_dynamics/aircraft_plotting.py`
  - Reusable figure builders for aircraft performance and simulation comparisons.
  - Combines maximum airspeed and rate-of-climb results in one performance-limits window.
  - Accepts an aircraft name and prepared datasets rather than depending on a specific aircraft model.

## Entry Points

- `scripts/c172_simulation.py`
  - Main working script for the C172 project.
  - Computes max-speed, rate-of-climb, drag-polar, trim, and nonlinear longitudinal results.
  - Selects the aircraft display name once and passes prepared results to
    `flight_dynamics/aircraft_plotting.py` for plotting.
  - Set `AIRCRAFT_NAME` to match the aircraft model and reference datasets being used.
  - Loads X-Plane comparison data through `flight_dynamics/external_data.py`.

- `scripts/c172_excess_power_ROC.py`
  - Alternate excess-power rate-of-climb analysis script.

- `scripts/c172_six_dof_simulation.py`
  - Solves the full six-DOF trim condition directly from configurable initial guesses.
  - Adds a bank-angle perturbation after trim is solved.
  - Integrates and plots the C172 six-degree-of-freedom response.

## Side Studies

The `examples/` folder holds exploratory or classroom-style scripts that are not part of the core package. This includes pendulum examples, thin-airfoil/lifting-line studies, and unfinished longitudinal/performance scratch files.

## Data and Outputs

- `data/`
  - CSV, ODS, and text data used by scripts and notebooks.

- `figures/`
  - Saved output figures.

- `notebooks/`
  - Notebook versions or supporting notebook assets.

- `tests/`
  - Pytest validation for trim convergence, physical bounds, raw residuals,
    control-direction signs, and finite six-DOF integration.

## Conventions and Validation Status

`docs/CONVENTIONS.md` documents the body and Earth axes, angular-rate and control
signs, coefficient conventions, units, trim validation, and the provisional status of
the current C172 lateral derivatives and inertia values.

## Suggested Reading Order

1. `README.md`
2. `flight_dynamics/c172_params.py`
3. `flight_dynamics/aero_model.py`
4. `flight_dynamics/longitudinal_dynamics.py`
5. `flight_dynamics/trim_solver.py`
6. `flight_dynamics/integrators.py`
7. `scripts/c172_simulation.py`
