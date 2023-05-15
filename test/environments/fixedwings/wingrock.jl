using FSimZoo
using Test
using FSimBase


function test_elzebdawingrock()
    t = 0.0
    env = ElzebdaWingRock()
    x1, x2 = rand(2)
    X = State(env)(x1, x2)
    u = 0.0
    dX = deepcopy(X)
    Dynamics!(env)(dX, X, [], t; u=u)
end


function test_tarnwingrock()
    t = 0.0
    env = TarnWingRock()
    x1, x2 = deg2rad.(10*rand(2))
    X = State(env)(x1, x2)
    u = 0.0
    dX = deepcopy(X)
    Dynamics!(env)(dX, X, [], t; u=u)
end


function main()
    test_elzebdawingrock()
    test_tarnwingrock()
end


@testset "wingrock" begin
    main()
end
