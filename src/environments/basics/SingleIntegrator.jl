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
    return function (dx, x, p, t; u)
        @assert length(dx) == length(u)
        dx .= u
        nothing
    end
end
