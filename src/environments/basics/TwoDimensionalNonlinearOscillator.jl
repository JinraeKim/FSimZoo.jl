"""
A two dimensional nonlinear oscillator introduced in [1].
Note that the system is constructed by converse HJB method [2].
# References
[1] J. A. Primbs, “Nonlinear Optimal Control: A Receding Horizon Approach,”
California Institute of Technology, Pasadena, California, 1999.
[2] J. Doyle, J. A. Primbs, B. Shapiro, and V. Nevistic,
“Nonlinear Games: Examples and Counterexamples,”
in Proceedings of 35th IEEE Conference on Decision and Control, Kobe, Japan, 1996,
vol. 4, pp. 3915–3920. doi: 10.1109/CDC.1996.577292.
"""
struct TwoDimensionalNonlinearOscillator <: AbstractEnv
end

function State(env::TwoDimensionalNonlinearOscillator)
    return function (x1, x2)
        ComponentArray(x1=x1, x2=x2)
    end
end

function Dynamics!(env::TwoDimensionalNonlinearOscillator)
    @Loggable function dynamics!(dx, x, p, t; u)
        @unpack x1, x2 = x
        @log state = x
        @log input = u
        dx.x1 = x2
        dx.x2 = (
                 -x1 * (π/2 + atan(5*x1))
                 - (5*x1^2)/(2*(1+25*x1^2))
                 + 4*x2
                 + 3*u[1]
                )
    end
end

function RunningCost(env::TwoDimensionalNonlinearOscillator)
    return function (x, u)
        @unpack x1, x2 = x
        r = x2^2 + u[1]^2
    end
end

function OptimalValue(env::TwoDimensionalNonlinearOscillator)
    return function (x)
        @unpack x1, x2 = x
        V_star = x1^2 * (π/2 + atan(5*x1)) + x2^2
    end
end

function OptimalControl(env::TwoDimensionalNonlinearOscillator)
    return function (x)
        @unpack x1, x2 = x
        u_star = -3*x2
    end
end
