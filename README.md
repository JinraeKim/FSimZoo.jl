# FSimZoo
[FSimZoo.jl](https://github.com/JinraeKim/FSimZoo.jl)
contains predefined environments and controllers for [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl).

## Road map
- [ ] Re-write missile environments and guidance laws; import them from [FlightGNC.jl](https://github.com/nhcho91/FlightGNC.jl).
- [ ] Re-name environments and controllers. Current naming is too confusing and unorganised.

## Examples
- Examples include

    <details>
    <summary>basics</summary>

    - (Linear system) `LinearSystemEnv`
    - (Reference model) `ReferenceModelEnv`
    - (Nonlinear system) `TwoDimensionalNonlinearPolynomialEnv`
        - [T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.](https://ieeexplore.ieee.org/document/7798777)
    - (Multiple Envs) `MultipleEnvs` for multi-agent simulation

    </details>

    <details>
    <summary>multicopters</summary>

    - (Hexacopter) `LeeHexacopterEnv` (**currently maintained**)
    - (Quadcopter) `IslamQuadcopterEnv`, `GoodarziQuadcopterEnv`

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

    </details>

    <details>
    <summary>integrated_environments</summary>

    - (Backstepping Position Controller + Static Allocator + Multicopter) `BacksteppingPositionController_StaticAllocator_MulticopterEnv`
        - For example, `BacksteppingPositionControllerEnv` (backstepping position controller) + `PseudoInverseAllocator` (pseudo-inverse allocator, a static allocator) + `LeeHexacopterEnv` (hexacopter, a multicopter)
    - See `src/environments/integrated_environments`.

    </details>

    <details>
    <summary>missiles</summary>

    - (point-mass simple missile in 3D space) `PointMass3DMissile`
    - (pursuer vs evador in 3D space) `PursuerEvador3DMissile`

    </details>
