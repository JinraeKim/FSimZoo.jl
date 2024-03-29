"""
# References
- Controller (it may be modified in this implementation)
[1] G. P. Falconi and F. Holzapfel,
“Adaptive Fault Tolerant Control Allocation for a Hexacopter System,”
Proc. Am. Control Conf., vol. 2016-July, pp. 6760–6766, 2016.
- Reference model (e.g., xd, vd, ad, ad_dot, ad_ddot)
[2] S. J. Su, Y. Y. Zhu, H. R. Wang, and C. Yun, “A Method to Construct a Reference Model for Model Reference Adaptive Control,” Adv. Mech. Eng., vol. 11, no. 11, pp. 1–9, 2019.

# NOTICE
- (Tracking control) Add the argument `pos_cmd_func` (a function of time such that (t) -> position(t) ∈ ℝ^3) of constructor,
please do not add the argument `pos_cmd` in the inner function of `Dynamics!(env::BacksteppingPositionController)`.
- (Set-point control) Do not add the argument `pos_cmd_func` (a function of time such that (t) -> position(t) ∈ ℝ^3) of constructor,
please add the argument `pos_cmd` in the inner function of `Dynamics!(env::BacksteppingPositionController)`.
"""
struct BacksteppingPositionController <: AbstractEnv
    Ref_model
    Ap
    Bp
    P
    Kp
    Kt
    Kω
    function BacksteppingPositionController(
            m::Real;
            pos_cmd_func=nothing,
            Kx=m*1*diagm(ones(3)),  # position
            Kv=m*1*1.82*diagm(ones(3)),  # velocity
            Kt=4*diagm(ones(3)),  # thrust
            Kω=20*diagm(ones(3)),  # angular velocity
        )
        @assert m > 0
        # from (s+5)^5 = s^5 + 25s^4 + ... (for **faster response**)
        Ks = []
        push!(Ks, Diagonal(3125*ones(3)))
        push!(Ks, Diagonal(3125*ones(3)))
        push!(Ks, Diagonal(1250*ones(3)))
        push!(Ks, Diagonal(250*ones(3)))
        push!(Ks, Diagonal(25*ones(3)))
        auto_diff = pos_cmd_func == nothing ? false : true
        Ref_model = ReferenceModel(length(Ks)-1, Ks, auto_diff, pos_cmd_func)
        Kp = hcat(Kx, Kv)
        Q = diagm(1*ones(6))
        # reference model
        Ap = [zeros(3, 3) Matrix(I, 3, 3);
              -(1/m)*Kx -(1/m)*Kv]
        Bp = [zeros(3, 3);
              (1/m)*Matrix(I, 3, 3)]
        P = MatrixEquations.lyapc(Ap, Q)
        new(Ref_model, Ap, Bp, P, Kp, Kt, Kω)
    end
end

"""
# Notes
ref_model.x_0 = xd
ref_model.x_1 = vd
ref_model.x_2 = ad
ref_model.x_3 = ȧd
ref_model.x_4 = äd
"""
function State(controller::BacksteppingPositionController)
    (; Ref_model) = controller
    return function (pos0, m, g)
        @assert m > 0
        ref_model = State(Ref_model)(pos0)
        Td = m*g
        ComponentArray(ref_model=ref_model, Td=Td)
    end
end

function Dynamics!(controller::BacksteppingPositionController)
    (; Ref_model) = controller
    @Loggable function dynamics!(dX, X, p, t; pos_cmd=nothing, Ṫd)
        @log state = X
        Dynamics!(Ref_model)(dX.ref_model, X.ref_model, (), t; x_cmd=pos_cmd)  # be careful; parameter = ()
        dX.Td = Ṫd
        nothing
    end
end


"""
Several functions are exported from `utils.jl`, e.g., T_u_inv(T).
"""
function Command(controller::BacksteppingPositionController)
    (; Ap, Bp, P, Kp, Kt, Kω) = controller
    T_u_inv(T) = [   0 1/T  0;
                  -1/T   0  0;
                     0   0 -1]
    T_u_inv_dot(T, Ṫ) = [    0 -Ṫ/T^2 0;
                         Ṫ/T^2      0 0;
                             0      0 0]
    T_ω(T) = [0 -T  0;
              T  0  0;
              0  0  0]
    skew(x) = [    0  -x[3]   x[2];
                x[3]      0  -x[1];
               -x[2]   x[1]     0]
    return function(p, v, R, ω,
                    xd, vd, ad, ȧd, äd, Td,
                    m::Real, J, g::Real,)
        ex = xd - p
        ev = vd - v
        ep = vcat(ex, ev)
        # u1
        g_vec = [0, 0, g]
        u1 = m*(ad - g_vec) + Kp*ep
        zB = R * [0, 0, 1]
        td = -Td * zB
        et = u1 - td
        ėp = Ap*ep + Bp*et
        u̇1 = m*ȧd + Kp*ėp
        T = Td  # TODO: no lag
        # u2
        u2 = T_u_inv(T) * R' * (2*Bp' * P * ep + u̇1 + Kt*et)
        Ṫd = u2[end]  # third element
        Ṫ = Ṫd  # TODO: no lag
        żB = -R * T_ω(1.0) * ω
        ėt = u̇1 + u2[end]*zB + Td*żB
        ëp = Ap*ėp + Bp*ėt
        ü1 = m*äd + Kp*ëp
        Ṙ = -skew(ω) * R'
        u̇2 = (
              (T_u_inv_dot(T, Ṫ)*R' + T_u_inv(T)*Ṙ) * (2*Bp'*P*ep + u̇1 + Kt*et)
              + T_u_inv(T) * R' * (2*Bp'*P*ėp + ü1 + Kt*ėt)
             )
        ω̇d = [1 0 0;
              0 1 0;
              0 0 0] * u̇2
        ωd = [u2[1:2]..., 0]
        eω = ωd - ω
        Md = cross(ω, J*ω) + J*(T_ω(T)'*R'*et + ω̇d + Kω*eω)
        νd = vcat(Td, Md)
        e = [ep;
             et;
             eω]
        νd, Ṫd, e, zB, T
    end
end
