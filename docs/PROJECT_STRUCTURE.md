# Project Structure

This document explains how the repository is organized and how the main C172 longitudinal simulation workflow fits together.

## High-Level Layout

The project has one main track and a few side-study scripts:

- Core C172 longitudinal modeling and simulation
- Utility functions for atmosphere, unit conversion, and integration
- Validation data and figures
- Experimental or classroom-style scripts that are not part of the main simulation path

## Main Workflow

The core C172 workflow runs in this order:

1. Aircraft and flight-condition parameters are defined in `c172_params.py`.
2. Atmospheric properties are computed with `standard_atmosphere.py`.
3. Aerodynamic coefficients are computed in `aero_model.py`.
4. Longitudinal equations of motion are evaluated in `aircraft_longitudinal_dynamics.py`.
5. A steady-flight trim condition is solved in `trim_solver.py`.
6. Time integration is performed with `integrators.py`.
7. Results are plotted and compared against POH / X-Plane style data in `scripts/c172_simulation.py`.

## Core Files

### Entry Point / Main Script

- `scripts/c172_simulation.py`
  - Main working script for the C172 project.
  - Generates drag-polar plots.
  - Solves for trim.
  - Simulates nonlinear longitudinal motion.
  - Loads comparison data from `data/Data.txt` and `data/c172_roc.csv`.
  - Produces the final comparison plots.

### Aircraft Definition

- `config/c172_params.py`
  - Central parameter dictionary for the aircraft.
  - Stores geometry, inertia, weight, trim speed, and aerodynamic coefficients.
  - Converts some values into simulation-ready units.
  - Sets the nominal starting altitude and time settings.

- `config/constants.py`
  - Shared physical constants in imperial units.
  - Includes gravity, gas constant, lapse rate, standard density, and standard temperature.

- `config/control_inputs.py`
  - Elevator input scheduling functions.

### Atmosphere and Speed Utilities

- `utils/standard_atmosphere.py`
  - Returns density, temperature, and pressure as a function of altitude.
  - Used by the dynamics, thrust, and conversion utilities.

- `utils/speed_of_sound.py`
  - Computes local speed of sound from temperature.

- `utils/ias2tas.py`
  - Converts indicated airspeed to true airspeed using atmosphere and compressible-flow relations.

- `utils/conversions.py`
  - Small unit-conversion helpers.
  - Includes knots-to-ft/s and horsepower-to-ft*lbf/s conversion.

- `utils/axis_transformations.py`
  - Converts between body-axis and velocity-axis quantities.

- `utils/integrators.py`
  - Custom numerical integrators.
  - Includes Euler, RK2, and RK4 implementations.

### Aerodynamics and Propulsion

- `models/aero_model.py`
  - Contains coefficient-level aerodynamic models.
  - Computes lift, drag, induced drag, and pitching moment coefficients.

- `models/drag_polar.py`
  - Builds a drag-polar style curve over a velocity range.
  - Uses the aircraft parameters and atmosphere model.

- `models/thrust_model.py`
  - Holds a naturally aspirated piston-engine thrust approximation.
  - Models available power and converts it to thrust using speed.

- `examples/power_required.py`
  - Intended for power-required calculations.
  - Looks like a work-in-progress rather than a finished module.

- `examples/ROC.py`
  - Intended for rate-of-climb calculations.
  - Currently incomplete.

### Dynamics and Trim

- `models/aircraft_longitudinal_dynamics.py`
  - Core nonlinear longitudinal state equations.
  - Computes aerodynamic forces, moments, and state derivatives.
  - Also contains the current elevator input scheduling function.

- `solvers/trim_solver.py`
  - Solves for a steady longitudinal trim condition.
  - Uses the nonlinear dynamics model and searches for thrust, elevator deflection, and pitch angle that make the state derivatives zero.

## Data and Outputs

- `data/Data.txt`
  - Imported as X-Plane style comparison data in the main simulation script.

- `data/c172_roc.csv`
  - POH-style climb-performance data used for comparison plots.

- `data/c172_roc.ods`
  - Spreadsheet version of the climb-rate data.

- `figures/drag_polar.png`
  - Saved drag-polar figure.

- `figures/sim_vs_xplane.png`
  - Saved simulation-vs-reference comparison figure.

## Documentation

- `README.md`
  - Project overview.
  - Documents the nonlinear longitudinal model and the aerodynamic equations.

- `docs/PROJECT_STRUCTURE.md`
  - This file.
  - Meant to be a navigation guide for the repo.

## Side Studies and Experimental Scripts

These files look useful, but they do not appear to be part of the main C172 nonlinear longitudinal pipeline:

- `examples/pendulum.py`
  - Demonstrates and compares custom integrators on a pendulum problem.

- `examples/pendulum_vector_field.py`
  - Likely related to the pendulum study or visualization.

- `examples/simpleODE.py`
  - Small ODE practice or test script.

- `examples/longitudinal_stability_linearized`
  - Appears to be an unfinished linearized longitudinal model script.

- `examples/temp.py`
  - Scratch/test file.

- `notebooks/c172_simulation.ipynb`
  - Notebook version of the simulation work.

## Suggested Mental Model

If you want to understand the repo quickly, read the files in this order:

1. `README.md`
2. `config/c172_params.py`
3. `models/aero_model.py`
4. `models/aircraft_longitudinal_dynamics.py`
5. `solvers/trim_solver.py`
6. `utils/integrators.py`
7. `scripts/c172_simulation.py`

## Current Structure Summary

The repository is already organized around a clear simulation pipeline:

- Parameters and constants
- Environment and conversion helpers
- Aerodynamic and propulsion models
- Nonlinear equations of motion
- Trim solution
- Time integration
- Validation and plotting

That makes the project easy to grow into cleaner modules later, especially if you eventually separate "library code" from "analysis scripts."
