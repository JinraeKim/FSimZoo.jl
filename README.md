# FSimZoo
[FSimZoo.jl](https://github.com/JinraeKim/FSimZoo.jl)
contains predefined environments and controllers for [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl).

## Examples
- Examples include

    <details>
    <summary>basics</summary>

    - (Linear system) `LinearSystem`
    - (A simple integrator) `SingleIntegrator`
    - (Reference model) `ReferenceModel`
    - (Nonlinear polynomial system) `TwoDimensionalNonlinearPolynomialSystem`
        - [T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.](https://ieeexplore.ieee.org/document/7798777)
    - (Nonlinear oscillator) `TwoDimensionalNonlinearOscillator`
        - [J. A. Primbs, “Nonlinear Optimal Control: A Receding Horizon Approach,” California Institute of Technology, Pasadena, California, 1999.](https://thesis.library.caltech.edu/4124/)
    - (Multiple Envs) `MultipleEnvs` for multi-agent simulation

    </details>

    <details>
    <summary>fixed wings</summary>

    - (Wing Rock phenomenon) `TarnWingRock`, `ElzebdaWingRock`

    </details>

    <details>
    <summary>multicopters</summary>

    - (Hexacopter) `LeeHexacopter`, `LeeQuadcopter`, `GoodarziAgileQuadcopter` (**currently maintained**)
    - (Quadcopter) `IslamQuadcopter`, `GoodarziQuadcopter`

    </details>

    <details>
    <summary>allocators</summary>

    - (Moore-Penrose pseudo inverse control allocation) `PseudoInverseAllocator`

    </details>

    <details>
    <summary>controllers</summary>

    - (Linear quadratic regulator) `LQR`
    - (Proportional-Integral-Derivative controller) `PID`
        - Note that the derivative term is obtained via second-order filter.
    - (Pure proportional navigation guidance) `PPNG`
    - (For multicopter position tracking)
        - `BacksteppingPositionController` (control input: `T_dot`, `M`)
        - `GeometricTrackingController` (control input: `T`, `M`)
        - `InnerLoopGeometricTrackingController` and `OuterLoopGeometricTrackingController` (based on `GeometricTrackingController` but seperated for a hierarchical structure)
    - (Safety filters via control barrier functions (CBFs))
        - (Position CBF for input-affine systems) `InputAffinePositionCBF`

    </details>

    <details>
    <summary>integrated_environments</summary>

    - (Backstepping Position Controller + Static Allocator + Multicopter) `BacksteppingPositionController_StaticAllocator_Multicopter`
        - For example, `BacksteppingPositionController` (backstepping position controller) + `PseudoInverseAllocator` (pseudo-inverse allocator, a static allocator) + `LeeHexacopter` (hexacopter, a multicopter)
    - (Linear system + single integrator) `LinearSystem_SingleIntegrator` (WIP)
    - See `src/environments/integrated_environments`.

    </details>


## Utilities
- `ned2enu` and `enu2ned`: coordinate transformation

