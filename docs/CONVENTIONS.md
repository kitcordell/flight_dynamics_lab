# Model Conventions and Parameter Status

This project uses one consistent body-axis convention across the longitudinal,
lateral-directional, and six-degree-of-freedom models. The equations use a local,
flat-Earth position frame and imperial engineering units.

## Body and Earth Axes

The body frame is right-handed:

- Positive body `x` points forward through the nose.
- Positive body `y` points toward the right wing.
- Positive body `z` points downward.
- `U`, `V`, and `W` are velocity components along positive body `x`, `y`, and `z`.

The local Earth-frame position states are north, east, and altitude:

- North and east are positive in their named horizontal directions, in feet.
- Altitude is positive upward, in feet.
- The Earth is treated as locally flat. Latitude, longitude, and Earth curvature are
  not modeled.
- Wind is not currently a separate state or input, so body velocity is air-relative
  while the position kinematics use the same velocity without a wind correction.

## Angular Rates and Euler Angles

`P`, `Q`, and `R` follow the right-hand rule about the positive body axes:

- Positive `P` is positive roll about body `x` (right wing down).
- Positive `Q` is positive pitch about body `y` (nose up).
- Positive `R` is positive yaw about body `z` (nose right).

The Euler angles use the corresponding aerospace sequence:

- Positive `phi` is right bank.
- Positive `theta` is nose-up pitch.
- Positive `psi` turns the heading from north toward east.

The Euler-angle equations become singular at `theta = +/- 90 degrees`. Trim searches
are therefore limited to `+/- 89 degrees`; this is a numerical domain restriction, not
an aircraft operating limit.

## Aerodynamic Angles and Coefficients

The implemented aerodynamic angles are:

```text
alpha = atan2(W, U)
beta  = atan2(V, sqrt(U^2 + W^2))
```

The coefficient signs follow the body axes:

- Positive `C_L` produces lift upward, which contributes a negative body-`z` force.
- Positive `C_D` produces drag opposite the forward airspeed.
- Positive `C_Y` produces side force along positive body `y`.
- Positive `C_l`, `C_m`, and `C_n` produce moments about positive body `x`, `y`, and
  `z`, respectively.

## Control-Deflection Signs

Control variables are defined by the signs used in `c172_params.py`. They should not
be interpreted as a hardware hinge-angle convention without an external source:

- Positive elevator has `C_m_delta_e < 0`, so it initially produces negative pitch
  acceleration (nose down).
- Positive aileron has `C_l_delta_a < 0`, so it initially produces negative roll
  acceleration (left roll / left wing down).
- Positive rudder has `C_n_delta_r < 0`, so it initially produces negative yaw
  acceleration (nose left). It also has `C_Y_delta_r > 0`, producing positive body-`y`
  side force.

The automated control-direction tests verify these repository-specific signs.

## Units

Internal calculations use imperial engineering units unless a function explicitly
documents a conversion:

| Quantity | Unit |
| --- | --- |
| Length, position, altitude | ft |
| Velocity | ft/s |
| Acceleration | ft/s^2 |
| Force and weight | lbf |
| Mass | slug |
| Moment | lbf*ft |
| Moment of inertia | slug*ft^2 |
| Power | ft*lbf/s |
| Density | slug/ft^3 |
| Angles and control deflections | rad |
| Angular rates | rad/s |
| Angular accelerations | rad/s^2 |
| Aerodynamic coefficients | nondimensional |

Plotting and external-data helpers may convert these values to knots, degrees, or
feet per minute for presentation.

## C172 Parameter Provenance

The repository and its available commit history do not identify a primary source for
the current C172 lateral-directional derivatives or the `I_xx`, `I_yy`, `I_zz`, and
`I_xz` inertia values. They are therefore marked **provisional / unverified** in
`c172_params.py`. They must be checked against a documented aircraft configuration
before treating the lateral or six-DOF results as validated C172 predictions.

The elevator travel limits preserve values already present in `control_inputs.py`.
No source is recorded for those values or for the provisional aileron and rudder
limits. These limits provide bounded numerical optimization, not certification data.

## Trim Scaling and Validation

The trim solvers optimize dimensionless residuals but expose the raw physical
residuals on the returned SciPy result:

- Linear acceleration scale: one `g` (`32.174 ft/s^2`).
- Angular acceleration scale: `1 rad/s^2`.
- Angular-rate scale: `1 rad/s` where applicable.
- Velocity scale: `100 ft/s`.

The default scaled residual-norm tolerance is `1e-6`. With one dominant residual,
that corresponds approximately to `3.2e-5 ft/s^2`, `1e-6 rad/s^2`, or `1e-4 ft/s`.
All scales, the residual tolerance, and the bound-proximity tolerance are named
constants and configurable solver arguments.

Each returned `OptimizeResult` exposes:

- `raw_residuals` and `scaled_residuals`
- `raw_residual_norm` and `scaled_residual_norm`
- `near_bounds` and `at_or_near_bound`
- `trim_valid`

An unsuccessful solve, nonfinite residual, or scaled residual norm above tolerance
raises `TrimConvergenceError` rather than silently returning an invalid trim state.
