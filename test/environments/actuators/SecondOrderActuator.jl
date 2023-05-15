using FSimZoo
using Test
using FSimBase


function main()
    t = 0.0
    ζ = 0.7
    ω = 150
    env = SecondOrderActuator(ζ, ω)
    x1, x2 = rand(2)
    X = State(env)(x1, x2)
    δ_cmd = 0.0
    dX = deepcopy(X)
    Dynamics!(env)(dX, X, [], t; δ_cmd=δ_cmd)
end


@testset "SecondOrderActuator" begin
    main()
end
