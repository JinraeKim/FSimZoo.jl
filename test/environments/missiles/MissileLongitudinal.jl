using FSimZoo
using Test
using FSimBase


function main()
    env = MissileLongitudinal()
    t = 0.0
    X = State(env)()
    δ = deg2rad(10)
    dX = deepcopy(X)
    Dynamics!(env)(dX, X, [], t; δ=δ)
end


@testset "MissileLongitudinal" begin
    main()
end
