"""
Wing-rock phenomenon in the roll motion of slendar delta wings at AoA=25 deg [1].

# Notes
x = [x1, x2]^T
x1 = ϕ (roll angle)
x2 = ϕ̇ (roll rate)

# Refs
[1] J. H. Tarn and F. Y. Hsu, “Fuzzy Control of Wing Rock for Slender Delta wings,” in 1993 American Control Conference, San Francisco, CA, USA: IEEE, Jun. 1993, pp. 1159–1161. doi: 10.23919/ACC.1993.4793048.
"""
Base.@kwdef struct TarnWingRock <: AbstractWingRock
    a1=-0.05686
    a2=+0.03254
    a3=+0.07334  
    a4=-0.35970
    a5=+1.46810
    C1=0.354
    C2=0.001
    u_min=-1.75
    u_max=+1.75
end


function State(env::TarnWingRock)
    return function (x1=0.0, x2=0.0)
        ComponentArray(x1=x1, x2=x2)
    end
end


"""
Out-of-place dynamics
"""
function oop_dynamics(env::TarnWingRock)
    (; a1, a2, a3, a4, a5, C1, C2) = env
    return function (x, p, t; u)
        (; x1, x2) = x
        ω_squares = -C1*a1
        μ1 = C1*a2 - C2
        b1 = C1*a3
        μ2 = C1*a4
        b2 = C1*a5
        basis = [x1, x2, x1^2*x2, x1*x2^2, x1^3]
        coeff = [-ω_squares, μ1, μ2, b2, b1]
        Δ = coeff' * basis
        dx1 = x2
        dx2 = u + Δ
        dx = State(env)(dx1, dx2)
        return dx
    end
end


function Dynamics!(env::TarnWingRock)
    (; u_min, u_max) = env
    @Loggable function dynamics!(dx, x, p, t; u)
        (; x1, x2) = x
        u_saturated = clamp.(u, u_min, u_max)
        @log state = x
        @log input = ComponentArray(u_cmd=u, u_applied=u_saturated) 
        dx .= oop_dynamics(env)(x, p, t; u=u_saturated)
    end
end
