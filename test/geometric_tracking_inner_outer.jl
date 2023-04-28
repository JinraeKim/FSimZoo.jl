using Test
using ForwardDiff
using FSimZoo
using FSimBase


function outerloop()
    multicopter = LeeQuadcopter()
    (; m, g) = multicopter
    X0 = State(multicopter)()
    controller = OuterLoopGeometricTrackingController()
    p_d = t -> [cos(t), sin(t), -t]
    (; p, v, R) = X0
    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)

    t = 0.0
    _b_3_d = Command(
                     controller, p, v;
                     p_d=p_d(t),
                     v_d=v_d(t),
                     a_d=a_d(t),
                     m=m, g=g,
                    )
    @test length(_b_3_d) == 3
end


@testset "geometric_tracking_inner_outer" begin
    outerloop()
end
