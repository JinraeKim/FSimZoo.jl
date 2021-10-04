# FSimZoo
[FSimZoo.jl](https://github.com/JinraeKim/FSimZoo.jl)
contains predefined environments and controllers for [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl).

## Road map
- [x] Re-name environments and controllers. Current naming is too confusing and unorganised.
- [ ] Re-write missile environments and guidance laws; import them from [FlightGNC.jl](https://github.com/nhcho91/FlightGNC.jl).
- [ ] Re-write approximate optimal control methods from [ApproximateDPs.jl](https://github.com/hnlee77/ApproximateDPs.jl).

## Examples
- Examples include

    <details>
    <summary>basics</summary>

    - (Constant system) `ConstantSystem`
    - (Linear system) `LinearSystem`
    - (Reference model) `ReferenceModel`
    - (Nonlinear polynomial system) `TwoDimensionalNonlinearPolynomialSystem`
        - [T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.](https://ieeexplore.ieee.org/document/7798777)
    - (Nonlinear oscillator) `TwoDimensionalNonlinearOscillator`
        - [J. A. Primbs, “Nonlinear Optimal Control: A Receding Horizon Approach,” California Institute of Technology, Pasadena, California, 1999.](https://thesis.library.caltech.edu/4124/)
    - (Multiple Envs) `MultipleEnvs` for multi-agent simulation

    </details>

    <details>
    <summary>multicopters</summary>

    - (Hexacopter) `LeeHexacopter` (**currently maintained**)
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

    </details>

    <details>
    <summary>integrated_environments</summary>

    - (Backstepping Position Controller + Static Allocator + Multicopter) `BacksteppingPositionController_StaticAllocator_Multicopter`
        - For example, `BacksteppingPositionController` (backstepping position controller) + `PseudoInverseAllocator` (pseudo-inverse allocator, a static allocator) + `LeeHexacopter` (hexacopter, a multicopter)
    - See `src/environments/integrated_environments`.

    </details>


## Utilities
- `ned2enu` and `enu2ned`: coordinate transformation

