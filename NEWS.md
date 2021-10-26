# Changes in v0.3.5
- A new discrete-time system, `TwoDimensionalNonlinearDTSystem`, is now added.

# Changes in v0.3.4
- `TwoDimensionalNonlinearPolynomialSystem` is now `@Loggable`.

# Changes in v0.3.3
- The interface of `TwoDimensionalNonlinearOscillator` has been changed; the input `u` is now an `Array`, e.g., `[1.0]`, not a `Number`.

# Changes in v0.3.2
- Environments are added and made to be loggable (thanks to [@hnlee77](https://github.com/hnlee77)!).
    - `SingleIntegrator`
    - `LinearSystem_SingleIntegrator`

# Changes in v0.3.1
- Bug fix: `TwoDimensionalNonlinearOscillator <: AbstractEnv`

# Changes in v0.3.0
- `TwoDimensionalNonlinearOscillator` is added.

# Changes in v0.2.0
- Predefined stuffs' names are changed.

# Changes in v0.1.0
- Environments and controllers are detached from [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl).
