using Test
using FSimZoo
using FSimBase
using ForwardDiff
using Plots


function main()
    multicopter = LeeQuadcopter()
    (; m, g, J) = multicopter
    X0 = State(multicopter)()
    k_p, k_v, k_R, k_ω = 5, 5, 5, 5
    controller = GeometricTrackingController(k_p=k_p, k_v=k_v, k_R=k_R, k_ω=k_ω)
    p_d = t -> [cos(t), sin(t), -t]
    b_1_d = t -> [1, 0, 0]
    (; p, v, q, ω) = X0
    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)
    a_d_dot = t -> ForwardDiff.derivative(a_d, t)
    a_d_ddot = t -> ForwardDiff.derivative(a_d_dot, t)
    b_1_d_dot = t -> ForwardDiff.derivative(b_1_d, t)
    b_1_d_ddot = t -> ForwardDiff.derivative(b_1_d_dot, t)

    t = 0.0
    R = quat2dcm(q)
    ν = Command(
                controller, p, v, R, ω;
                p_d=p_d(t),
                v_d=v_d(t),
                a_d=a_d(t),
                a_d_dot=a_d_dot(t),
                a_d_ddot=a_d_ddot(t),
                b_1_d=b_1_d(t),
                b_1_d_dot=b_1_d_dot(t),
                b_1_d_ddot=b_1_d_ddot(t),
                m=m, g=g, J=J,
               )
    @test length(ν) == 4
end


@testset "geometric_tracking" begin
    main()
end
