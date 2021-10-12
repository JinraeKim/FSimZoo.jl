"""
# References
[1] T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.
"""
struct TwoDimensionalNonlinearPolynomialSystem <: AbstractEnv
end

function State(env::TwoDimensionalNonlinearPolynomialSystem)
    return function (x1::Real=-2.9, x2::Real=-2.9)
        ComponentArray(x1=x1, x2=x2)
    end
end

# function Dynamics!(env::TwoDimensionalNonlinearPolynomialSystem)
#     return function (dx, x, p, t; u)
#         @assert length(u) == 1
#         _u = u[1]  # Real or Array
#         @unpack x1, x2 = x
#         dx.x1 = -(1/2)*x1^3 - x1 - 2*x2
#         dx.x2 = (1/8)*x2^3 - x2 + (1/2)*_u^3
#         nothing
#     end
# end
function Dynamics!(env::TwoDimensionalNonlinearPolynomialSystem)
    @Loggable function dynamics!(dx, x, p, t; u)
        @assert length(u) == 1
        @log state = x
        @log input = u
        _u = u[1]  # Real or Array
        @unpack x1, x2 = x
        dx.x1 = -(1/2)*x1^3 - x1 - 2*x2
        dx.x2 = (1/8)*x2^3 - x2 + (1/2)*_u^3
        nothing
    end
end

function OptimalControl(env::TwoDimensionalNonlinearPolynomialSystem)
    return function (x::ComponentArray, p, t)
        @unpack x1, x2 = x
        -x2
    end
end

function OptimalValue(env::TwoDimensionalNonlinearPolynomialSystem)
    return function (x::ComponentArray)
        @unpack x1, x2 = x
        x1^2 + x2^2
    end
end

function RunningCost(env::TwoDimensionalNonlinearPolynomialSystem)
    return function (x::ComponentArray, _u)
        @assert length(_u) == 1
        u = _u[1]  # Real or Array
        @unpack x1, x2 = x
        x1^4 + 2*(x1+x2)^2 + (3/4)*u^4
    end
end
