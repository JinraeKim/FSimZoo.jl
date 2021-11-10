"""
A two dimensional nonlinear discrete time (DT) system.
"""

Base.@kwdef struct TwoDimensionalNonlinearDTSystem <: AbstractEnv
    c = 0.5
    d = 1.0
end

function State(env::TwoDimensionalNonlinearDTSystem)
    return function (x1, x2)
        ComponentArray(x1=x1, x2=x2)
    end
end

function V1(env::TwoDimensionalNonlinearDTSystem)
    return function (x)
        if x > 0
            ans = x^(1/4)
        else
            ans = x^4
        end
        ans
    end
end

function Dynamics!(env::TwoDimensionalNonlinearDTSystem)
    @unpack c, d = env
    @Loggable function dynamics!(dx, x, p, t; u)
        @unpack x1, x2 = x
        @log state = x
        @log input = u
        dx.x1 = c*(x1 + x2)  # x1_next = dx.x1
        dx.x2 = c*x2 + u[1]  # x2_next = dx.x2
        @onlylog next_state = dx
    end
end

function CubicSolution(env::TwoDimensionalNonlinearDTSystem)
    @unpack c, d = env
    return function (x2)
        a = (
             (cbrt(sqrt(3)*sqrt(27*c^2*x2^2 + 8) + 9*c*x2) / 3^(2/3))
             - (2 / (cbrt(3) * cbrt(sqrt(3)*sqrt(27*c^2*x2^2 + 8) + 9*c*x2)))
            )  # solve a^3 + 2a - 2*c*x2 for a ∈ ℝ
    end
end

function RunningCost(env::TwoDimensionalNonlinearDTSystem)
    @unpack c, d = env
    return function (x, u)
        @unpack x1, x2 = x
        dx = copy(x)  # will be next state
        Dynamics!(env)(dx, x, nothing, nothing; u=u)
        a = CubicSolution(env)(x2)
        r̄ = (
             (3/4)*a^4 + a^2
             - d*V1(env)(dx.x1) + d*V1(env)(x1)
             + (1-c^2)*x2^2
            )  # for V(x) = d*sqrt(abs(x1)) + x2^2
        r = r̄ + (1/4)*u[1]^4
    end
end

function OptimalValue(env::TwoDimensionalNonlinearDTSystem)
    @unpack c, d = env
    return function (x)
        @unpack x1, x2 = x
        d*V1(env)(x1) + x2^2
    end
end

function OptimalQValue(env::TwoDimensionalNonlinearDTSystem)
    return function (x, u)
        dx = copy(x)
        Dynamics!(env)(dx, x, nothing, nothing; u=u)
        OptimalValue(env)(dx) + RunningCost(env)(x, u)  # V(x_next) + r(x, u)
    end
end

function OptimalControl(env::TwoDimensionalNonlinearDTSystem)
    return function (x)
        @unpack x2 = x
        a = CubicSolution(env)(x2)
        -a
    end
end
