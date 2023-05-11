using FSimZoo
using Test


function main()
    W_true = rand(5)
    env = WingRock(W_true)
    x1, x2 = rand(2)
    X = State(env)(x1, x2)
    u = 0.0
    dX = deepcopy(X)
    Dynamics!(env)(dX, X, [], t; u=u)
end


@testset "wingrock" begin
    main()
end
