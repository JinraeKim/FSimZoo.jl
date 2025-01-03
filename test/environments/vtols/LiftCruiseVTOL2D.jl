using FSimZoo
using Test
using FSimBase
using ComponentArrays


function main()
    env = LiftCruiseVTOL2D()
    t = 0.0
    X = State(env)()
    dX = deepcopy(X)
    T_r = 0.0
    T_p = 0.0
    M = 0.0
    δ_e = 0.0
    u = ComponentArray(; T_r, T_p, M, δ_e)
    Dynamics!(env)(dX, X, [], t; u)
end


@testset "MissileLongitudinal" begin
    main()
end
