struct SecondOrderActuator <: AbstractActuator
    ζ
    ω
end


function State(env::SecondOrderActuator)
    return function (δ, δ̇)
        ComponentArray(δ=δ, δ̇=δ̇)
    end
end


function Dynamics!(env::SecondOrderActuator)
    (; ζ, ω) = env
    return function dynamics!(dx, x, p, t; δ_cmd)
        (; δ, δ̇) = x
        dx.δ = δ̇
        dx.δ̇ = -2*ζ*ω*δ̇ + ω^2*(δ_cmd-δ)
    end
end
