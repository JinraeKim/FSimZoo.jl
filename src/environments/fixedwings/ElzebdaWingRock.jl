"""
Wing-rock phenomenon in the roll motion of slendar delta wings [1].

# Notes
## Nondimensional values
According to [2], the time, state, and control input of this model are nondimensional.
But here, it is used as time [s], state [rad, rad/s].

## Input constraint
Input constraint in [3] is used.

## Ideal parameters
The default ideal parameters, W_true, are set as the specific model obtained at the angle of attack of 25 degrees in [1].
Note that the ideal parameters in [2] are (1000 x the ideal parameters in [1]).

# Refs
[1] J. M. Elzebda, A. H. Nayfeh, and D. T. Mook, “Development of an analyt- ical model of wing rock for slender delta wings,” J. Aircr., vol. 26, no. 8, pp. 737–743, 1989.
[2] N. Cho, H.-S. Shin, Y. Kim, and A. Tsourdos, “Composite Model Reference Adaptive Control with Parameter Convergence Under Finite Excitation,” IEEE Trans. Automat. Contr., vol. 63, no. 3, pp. 811–818, Mar. 2018, doi: 10.1109/TAC.2017.2737324.
[3] J. H. Tarn and F. Y. Hsu, “Fuzzy Control of Wing Rock for Slender Delta wings,” in 1993 American Control Conference, San Francisco, CA, USA: IEEE, Jun. 1993, pp. 1159–1161. doi: 10.23919/ACC.1993.4793048.
"""
Base.@kwdef struct ElzebdaWingRock <: AbstractWingRock
    W_true = 1e-3 * [-18.59521, 15.162375, -62.45153, 9.54708, 21.45291]
    u_min=-1.75
    u_max=+1.75
end


function State(env::ElzebdaWingRock)
    return function (x1=0.0, x2=0.0)
        ComponentArray(x1=x1, x2=x2)
    end
end


"""
Out-of-place dynamics
"""
function oop_dynamics(env::ElzebdaWingRock)
    (; W_true) = env
    return function (x, p, t; u)
        (; x1, x2) = x
        basis = [x1, x2, abs(x1)*x2, abs(x2)*x2, x1^3]
        Δ = W_true' * basis
        dx1 = x2
        dx2 = u + Δ
        dx = State(env)(dx1, dx2)
        return dx
    end
end


function Dynamics!(env::ElzebdaWingRock)
    (; u_min, u_max) = env
    @Loggable function dynamics!(dx, x, p, t; u)
        (; x1, x2) = x
        u_saturated = clamp.(u, u_min, u_max)
        @log state = x
        @log input = ComponentArray(u_cmd=u, u_applied=u_saturated) 
        dx .= oop_dynamics(env)(x, p, t; u=u_saturated)
    end
end
