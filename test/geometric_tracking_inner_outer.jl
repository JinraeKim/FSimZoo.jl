using Test
using ForwardDiff
using FSimZoo
using FSimBase
using OrdinaryDiffEq
using Plots


function outerloop()
    multicopter = LeeQuadcopter()
    (; m, g) = multicopter
    X0 = State(multicopter)()
    controller = OuterLoopGeometricTrackingController()
    p_d = t -> [cos(t), sin(t), -t]
    (; p, v) = X0
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


function innerloop()
    multicopter = LeeQuadcopter()
    (; J) = multicopter
    X0 = State(multicopter)()
    controller = InnerLoopGeometricTrackingController()
    (; q, ω) = X0
    b_1_d = t -> [cos(t), sin(t), 0]
    b_1_d_dot = t -> ForwardDiff.derivative(b_1_d, t)
    b_1_d_ddot = t -> ForwardDiff.derivative(b_1_d_dot, t)
    _b_3_d = t -> [cos(t), sin(t), -t]  # dummy
    _b_3_d_dot = t -> ForwardDiff.derivative(_b_3_d, t)
    _b_3_d_ddot = t -> ForwardDiff.derivative(_b_3_d_dot, t)

    t = 0.0
    R = quat2dcm(q)
    ν = Command(
                controller, R, ω;
                b_1_d=b_1_d(t),
                b_1_d_dot=b_1_d_dot(t),
                b_1_d_ddot=b_1_d_ddot(t),
                _b_3_d=_b_3_d(t),
                _b_3_d_dot=_b_3_d_dot(t),
                _b_3_d_ddot=_b_3_d_ddot(t),
                J=J,
               )
    @test length(ν) == 4
end


function derivative_estimator(; t0=0.0, tf=3.0, Δt=0.05)
    controller = InnerLoopGeometricTrackingController()
    X0 = State(controller)()
    f_cmd = t -> [sin(pi*t), cos(pi*t), t]
    f_cmd_dot = t -> ForwardDiff.derivative(f_cmd, t)
    f_cmd_ddot = t -> ForwardDiff.derivative(f_cmd_dot, t)
    prob = ODEProblem((dX, X, params, t) -> FSimBase.Dynamics!(controller)(dX, X, params, t; f=f_cmd(t)), X0, (t0, tf))
    sol = solve(prob, Tsit5())
    fig = plot(layout=(2, 1))
    ts = t0:Δt:tf
    estimated_f_dots = [controller.ω_n_f*sol(t).z2_f for t in ts]
    f_cmd_dots = [f_cmd_dot(t) for t in ts]
    estimated_f_ddots = [controller.ω_n_f_dot*sol(t).z2_f_dot for t in ts]
    f_cmd_ddots = [f_cmd_ddot(t) for t in ts]
    plot!(fig, ts, hcat(estimated_f_dots...)', subplot=1, label="", lc=:blue)
    plot!(fig, ts, hcat(f_cmd_dots...)', subplot=1, label="", lc=:red, ls=:dash)
    plot!(fig, ts, hcat(estimated_f_ddots...)', subplot=2, label="", lc=:blue)
    plot!(fig, ts, hcat(f_cmd_ddots...)', subplot=2, label="", lc=:red, ls=:dash)
    display(fig)
end


@testset "geometric_tracking_inner_outer" begin
    outerloop()
    innerloop()
    derivative_estimator()
end
