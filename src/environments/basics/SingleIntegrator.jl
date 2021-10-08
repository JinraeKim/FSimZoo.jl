struct SingleIntegrator <: AbstractEnv
end

"""
Even for scalar integrator, you should put an array of length 1;
due to the limitation of in-place method `Dynamics!`.
"""
function State(env::SingleIntegrator)
    return function (x::AbstractArray)
        x
    end
end

function Dynamics!(env::SingleIntegrator)
    @Loggable function dynamics!(dx, x, p, t; u)
        @assert length(dx) == length(u)
        @log integral = x
        @log integrand = u
        dx .= u
    end
end
