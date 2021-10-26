"""
A two dimensional nonlinear discrete time (DT) system.
"""

Base.@kwdef struct TwoDimensionalNonlinearDTSystem <: AbstractEnv
    c = 1/2
end

function State(env::TwoDimensionalNonlinearDTSystem)
    return function (x1, x2)
        ComponentArray(x1=x1, x2=x2)
    end
end

function Dynamics!(env::TwoDimensionalNonlinearDTSystem)
    @unpack c = env
    @Loggable function dynamics!(dx, x, p, t; u)
        @unpack x1, x2 = x
        @log state = x
        @log input = u
        dx.x1 = c * (x1 + x2)  # x1_next = dx.x1
        dx.x2 = c * x2 + u[1]  # x2_next = dx.x2
    end
end

function CubicSolution(env::TwoDimensionalNonlinearDTSystem)
    @unpack c = env
    return function (x)
        @unpack x2 = x
        a = (
             (cbrt(sqrt(3)*sqrt(27*c^2*x2^2 + 8) + 9*c*x2) / 3^(2/3))
             - (2 / (cbrt(3) * cbrt(sqrt(3)*sqrt(27*c^2*x2^2 + 8) + 9*c*x2)))
            )  # solve a^3 + 2a - 2*c*x2 for a ∈ ℝ
    end
end

function RunningCost(env::TwoDimensionalNonlinearDTSystem)
    @unpack c = env
    return function (x, u)
        a = CubicSolution(env)(x)
        r = (
             (3/4)*a^4 + a^2 + x'*[(1-c^2) -c^2; -c^2 (1-2*c^2)]*x
             + (1/4)*u[1]^4
            )
    end
end

function OptimalValue(env::TwoDimensionalNonlinearDTSystem)
    return function (x)
        @unpack x1, x2 = x
        x1^2 + x2^2
    end
end

function OptimalQValue(env::TwoDimensionalNonlinearDTSystem)
    return function (x, u)
        dx = copy(x)
        Dynamics!(env)(dx, x, nothing, 0.0; u)
        OptimalValue(env)(dx) + RunningCost(env)(x, u)
    end
end

function OptimalControl(env::TwoDimensionalNonlinearDTSystem)
    return function (x)
        a = CubicSolution(env)(x)
        -a
    end
end
