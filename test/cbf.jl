using Test
using FSimZoo
using FSimBase
using LinearAlgebra
using Convex


function test_InputAffinePositionCBF()
    f = (p, v) -> zeros(3) + [0, 0, 9.81]
    g = (p, v) -> I(3)
    h = p -> -p[3] - (-0.1)  # >= 0
    α1 = x -> 5*x
    α2 = x -> 5*x
    cbf = InputAffinePositionCBF(f, g, h, α1, α2)
    # filter
    p = zeros(3)
    v = zeros(3)
    u_nom = zeros(3)
    u = Convex.Variable(length(u_nom))
    u = Command(cbf, u, p, v, u_nom, [])
end


@testset "cbf" begin
    test_InputAffinePositionCBF()
end
