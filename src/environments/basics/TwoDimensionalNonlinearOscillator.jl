"""
A two dimensional nonlinear oscillator introduced in [1].
Note that the system is constructed by converse HJB method [2].
# References
[1] 
[2]
"""
struct TwoDimensionalNonlinearOscillator
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
        dx.x2 = -x1 * (
                       π/2
                       + atan(5*x1)
                       - (5*x1^2)/(2*(1+25*x1^2))
                       + 4*x2
                       + 3*u
                      )
    end
end

function RunningCost(env::TwoDimensionalNonlinearOscillator)
    return function (x, u)
        @unpack x1, x2 = x
        r = x2^2 + u^2
    end
end

function OptimalValue(env::TwoDimensionalNonlinearOscillator)
    return function (x, u)
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
