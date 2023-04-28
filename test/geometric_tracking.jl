using Test
using FSimZoo
using FSimBase
using ForwardDiff
using Plots
using DifferentialEquations


function main()
    multicopter = LeeQuadcopter()
    (; m, g, J) = multicopter
    X0 = State(multicopter)()
    k_p, k_v, k_R, k_ω = 5, 5, 5, 5
    controller = GeometricTrackingController(k_p=k_p, k_v=k_v, k_R=k_R, k_ω=k_ω)
    p_d = t -> [cos(t), sin(t), -t]
    b_1_d = t -> [1, 0, 0]
    (; p, v, R, ω) = X0
    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)
    a_d_dot = t -> ForwardDiff.derivative(a_d, t)
    a_d_ddot = t -> ForwardDiff.derivative(a_d_dot, t)
    b_1_d_dot = t -> ForwardDiff.derivative(b_1_d, t)
    b_1_d_ddot = t -> ForwardDiff.derivative(b_1_d_dot, t)

    t = 0.0
    ν = Command(
                controller, p, v, R', ω;
                a=zeros(3),  # IT SHOULD BE ESTIMATED!
                a_dot=zeros(3),  # IT SHOULD BE ESTIMATED!
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


function derivative_estimator(; t0=0.0, tf=3.0, Δt=0.05)
    k_p, k_v, k_R, k_ω = 5, 5, 5, 5
    controller = GeometricTrackingController(k_p=k_p, k_v=k_v, k_R=k_R, k_ω=k_ω)
    X0 = State(controller)()
    v_cmd = t -> [sin(pi*t), cos(pi*t), t]
    a_cmd = t -> ForwardDiff.derivative(v_cmd, t)
    a_dot_cmd = t -> ForwardDiff.derivative(a_cmd, t)
    prob = ODEProblem((dX, X, params, t) -> FSimBase.Dynamics!(controller)(dX, X, params, t; v=v_cmd(t)), X0, (t0, tf))
    sol = solve(prob, Tsit5())
    fig = plot(layout=(2, 1))
    ts = t0:Δt:tf
    âs = [controller.ω_n_v*sol(t).z2_v for t in ts]
    a_cmds = [a_cmd(t) for t in ts]
    â_dots = [controller.ω_n_a*sol(t).z2_a for t in ts]
    a_dot_cmds = [a_dot_cmd(t) for t in ts]
    plot!(fig, ts, hcat(âs...)', subplot=1, label="", lc=:blue)
    plot!(fig, ts, hcat(a_cmds...)', subplot=1, label="", lc=:red, ls=:dash)
    plot!(fig, ts, hcat(â_dots...)', subplot=2, label="", lc=:blue)
    plot!(fig, ts, hcat(a_dot_cmds...)', subplot=2, label="", lc=:red, ls=:dash)
    display(fig)
end


@testset "geometric_tracking" begin
    derivative_estimator()
    main()
end
