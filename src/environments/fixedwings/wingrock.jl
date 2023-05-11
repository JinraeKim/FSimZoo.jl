"""
Wing-rock phenomenon in the roll motion of slendar delta wings [1].

# Notes
## Nondimensional values
The time, state, and control input of this model are nondimensional [2].

## Ideal parameters
The default ideal parameters, W_true, are set as the specific model obtained at the angle of attack of 25 degrees in [1].
Note that the ideal parameters in [2] are (1000 x the ideal parameters in [1]).

# Refs
[1] J. M. Elzebda, A. H. Nayfeh, and D. T. Mook, “Development of an analyt- ical model of wing rock for slender delta wings,” J. Aircr., vol. 26, no. 8, pp. 737–743, 1989.
[2] N. Cho, H.-S. Shin, Y. Kim, and A. Tsourdos, “Composite Model Reference Adaptive Control with Parameter Convergence Under Finite Excitation,” IEEE Trans. Automat. Contr., vol. 63, no. 3, pp. 811–818, Mar. 2018, doi: 10.1109/TAC.2017.2737324.
"""
Base.@kwdef struct WingRock <: AbstractEnv
    W_true = 1e-3 * [-18.59521, 15.162375, -62.45153, 9.54708, 21.45291]
end


function State(env::WingRock)
    return function (x1=0.0, x2=0.0)
        ComponentArray(x1=x1, x2=x2)
    end
end


function Dynamics!(env::WingRock)
    (; W_true) = env
    @Loggable function dynamics!(dx, x, p, t; u)
        (; x1, x2) = x
        @log state = x
        @log input = u
        ϕ = [x1, x2, abs(x1)*x2, abs(x2)*x2, x1^3]
        Δ = W_true' * ϕ
        dx.x1 = x2
        dx.x2 = u + Δ
    end
end
