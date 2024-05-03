"""
Geometric tracking controller for multicopter [1].

# References
[1] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” in 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, Dec. 2010. doi: 10.1109/CDC.2010.5717652.
"""
Base.@kwdef struct GeometricTrackingController <: AbstractEnv
    k_p = 5
    k_v = 5
    k_R = deg2rad(5)
    k_ω = deg2rad(5)
    ω_n_v = 5e2
    ζ_v = 0.99
    ω_n_a = 5e2
    ζ_a = 0.99
end


Base.@kwdef struct InnerLoopGeometricTrackingController <:AbstractEnv
    k_R = deg2rad(5)
    k_ω = deg2rad(5)
    ω_n_f = 1e2
    ζ_f = 0.99
    ω_n_f_dot = 1e2
    ζ_f_dot = 0.99
end


function FSimBase.State(controller::InnerLoopGeometricTrackingController)
    return function (
                     z1_f=zeros(3), z2_f=zeros(3),
                     z1_f_dot=zeros(3), z2_f_dot=zeros(3),
                    )
        X0 = ComponentArray(
                            z1_f=z1_f, z2_f=z2_f,
                            z1_f_dot=z1_f_dot, z2_f_dot=z2_f_dot,
                           )
        return X0
    end
end


function FSimBase.Dynamics!(controller::InnerLoopGeometricTrackingController)
    (; ω_n_f, ω_n_f_dot, ζ_f, ζ_f_dot) = controller
    @Loggable function dynamics!(dX, X, params, t; f)
        (; z1_f, z2_f, z1_f_dot, z2_f_dot) = X
        @log z1_f
        @log z2_f
        @log z1_f_dot
        @log z2_f_dot
        @log f
        dX.z1_f = ω_n_f * z2_f
        dX.z2_f = -2*ζ_f*ω_n_f*z2_f - ω_n_f*(z1_f-f)
        dX.z1_f_dot = ω_n_f_dot * z2_f_dot
        dX.z2_f_dot = -2*ζ_f_dot*ω_n_f_dot*z2_f_dot - ω_n_f_dot*(z1_f_dot-ω_n_f*z2_f)
    end
end


function Command(
        controller::InnerLoopGeometricTrackingController, R, ω;
        b_1_d, b_1_d_dot, b_1_d_ddot,
        _b_3_d, _b_3_d_dot, _b_3_d_ddot,
        J,
    )
    (; k_R, k_ω) = controller
    e_3 = [0, 0, 1]

    b_3_d = _b_3_d / norm(_b_3_d)
    b_3_d_dot = _unit_vector_dot(_b_3_d, _b_3_d_dot)
    b_3_d_ddot = _unit_vector_ddot(_b_3_d, _b_3_d_dot, _b_3_d_ddot)

    _b_2_d = cross(b_3_d, b_1_d)
    _b_2_d_dot = cross(b_3_d_dot, b_1_d) + cross(b_3_d, b_1_d_dot)
    _b_2_d_ddot = cross(b_3_d_ddot, b_1_d) + cross(b_3_d_dot, b_1_d_dot) + cross(b_3_d, b_1_d_ddot)
    b_2_d = _b_2_d / norm(_b_2_d)
    b_2_d_dot = _unit_vector_dot(_b_2_d, _b_2_d_dot)
    b_2_d_ddot = _unit_vector_ddot(_b_2_d, _b_2_d_dot, _b_2_d_ddot)

    R_d = [cross(b_2_d, b_3_d)  b_2_d  b_3_d]
    R_d_dot = [(cross(b_2_d_dot, b_3_d) + cross(b_2_d, b_3_d_dot)) b_2_d_dot b_3_d_dot]
    R_d_ddot = [(cross(b_2_d_ddot, b_3_d) + cross(b_2_d_dot, b_3_d_dot) + cross(b_2_d, b_3_d_ddot)) b_2_d_ddot b_3_d_ddot]
    ω_d_hat = R_d' * R_d_dot  # guessed from the equation between [1, Eq. (10)] and [1, Eq. (11)].
    ω_d = _vee(ω_d_hat)
    ω_d_dot_hat = R_d_dot' * R_d_dot + R_d' * R_d_ddot
    ω_d_dot = _vee(ω_d_dot_hat)

    e_R = 0.5 * _vee(R_d'*R - R'*R_d)
    e_ω = ω - R'*R_d*ω_d

    f = dot(_b_3_d, R*e_3)
    M = -k_R*e_R - k_ω*e_ω + cross(ω, J*ω) - J*(_hat(ω) * R' * R_d * ω_d - R' * R_d * ω_d_dot)
    ν = [f, M...]
    return ν
end



Base.@kwdef struct OuterLoopGeometricTrackingController <:AbstractEnv
    k_p = 5
    k_v = 5
end


function Command(
        controller::OuterLoopGeometricTrackingController, p, v;
        p_d, v_d, a_d,
        m, g,
    )
    (; k_p, k_v) = controller
    e_3 = [0, 0, 1]

    e_p = p - p_d
    e_v = v - v_d
    _b_3_d = -(-k_p*e_p - k_v*e_v - m*g*e_3 + m*a_d)  # desired force vector
    return _b_3_d
end


"""
cross(x, y) = _hat(x) * y
"""
function _hat(a)
    a1, a2, a3 = a
    return [0 -a3 a2;
            a3 0 -a1;
            -a2 a1 0]
end

"""
Inverse map of _hat
"""
function _vee(A)
    a1 = A[3, 2]
    a2 = A[1, 3]
    a3 = A[2, 1]
    return [a1, a2, a3]
end


"""
y = ||x||
y_dot = ?
"""
function _norm_dot(x, x_dot)
    y = norm(x)
    y_dot = (x'*x_dot) / y
    return y_dot
end


"""
y = ||x||
y_ddot = ?
"""
function _norm_ddot(x, x_dot, x_ddot)
    y = norm(x)
    y_dot = _norm_dot(x, x_dot)
    y_ddot = (1/y) * (x_dot'*x_dot + x'*x_ddot - y_dot^2)
    return y_ddot
end

"""
y = x / ||x||
y_dot = ?
"""
function _unit_vector_dot(x, x_dot)
    y = (1/norm(x)) * x
    y_dot = (1/norm(x)) * (x_dot - _norm_dot(x, x_dot)*y)
    return y_dot
end


"""
y = x / ||x||
y_ddot = ?
"""
function _unit_vector_ddot(x, x_dot, x_ddot)
    y = (1/norm(x)) * x
    y_dot = _unit_vector_dot(x, x_dot)
    y_ddot = (1/norm(x)) * (x_ddot - _norm_ddot(x, x_dot, x_ddot)*y - 2*_norm_dot(x, x_dot)*y_dot)
    return y_ddot
end


function FSimBase.State(controller::GeometricTrackingController)
    return function (
                     z1_v=zeros(3), z2_v=zeros(3),
                     z1_a=zeros(3), z2_a=zeros(3),
                    )
        X0 = ComponentArray(
                            z1_v=z1_v, z2_v=z2_v,
                            z1_a=z1_a, z2_a=z2_a,
                           )
        return X0
    end
end


"""
Time derivative estimator
"""
function FSimBase.Dynamics!(controller::GeometricTrackingController)
    (; ω_n_v, ω_n_a, ζ_v, ζ_a) = controller
    return function (dX, X, params, t; v)
        (; z1_v, z2_v, z1_a, z2_a) = X
        dX.z1_v = ω_n_v * z2_v
        dX.z2_v = -2*ζ_v*ω_n_v*z2_v - ω_n_v*(z1_v-v)
        dX.z1_a = ω_n_a * z2_a
        dX.z2_a = -2*ζ_a*ω_n_a*z2_a - ω_n_a*(z1_a-ω_n_v*z2_v)
    end
end


"""
R is defined as the rotation matrix in [1, Eqs.(3), (4)].
Please check if it is the transpose of the rotation matrix defined in FSimZoo multicopters.

# Notes
When implementing geometric controller, you need to access the acceleration and jerk,
which requires the knowledge of dynamics.
In general, it may be inaccurate.
Instead, here, I implemented geometric controller with observers (filters) to estimate them.

For the ideal case, see `Command_Ideal(controller::GeometricTrackingController, args...; kwargs...)`.
"""
function Command(
        controller::GeometricTrackingController, p, v, R, ω;
        a, a_dot,  # estimated
        p_d, v_d, a_d, a_d_dot, a_d_ddot,
        b_1_d, b_1_d_dot, b_1_d_ddot,
        m, g, J,
    )
    (; k_p, k_v, k_R, k_ω) = controller
    e_3 = [0, 0, 1]

    e_p = p - p_d
    e_v = v - v_d
    e_a = a - a_d  # See Notes
    e_a_dot = a_dot - a_d_dot  # See Notes

    _b_3_d = -(-k_p*e_p - k_v*e_v - m*g*e_3 + m*a_d)
    _b_3_d_dot = -(-k_p*e_v - k_v*e_a + m*a_d_dot)
    _b_3_d_ddot = -(-k_p*e_a - k_v*e_a_dot + m*a_d_ddot)
    b_3_d = _b_3_d / norm(_b_3_d)
    b_3_d_dot = _unit_vector_dot(_b_3_d, _b_3_d_dot)
    b_3_d_ddot = _unit_vector_ddot(_b_3_d, _b_3_d_dot, _b_3_d_ddot)

    _b_2_d = cross(b_3_d, b_1_d)
    _b_2_d_dot = cross(b_3_d_dot, b_1_d) + cross(b_3_d, b_1_d_dot)
    _b_2_d_ddot = cross(b_3_d_ddot, b_1_d) + cross(b_3_d_dot, b_1_d_dot) + cross(b_3_d, b_1_d_ddot)
    b_2_d = _b_2_d / norm(_b_2_d)
    b_2_d_dot = _unit_vector_dot(_b_2_d, _b_2_d_dot)
    b_2_d_ddot = _unit_vector_ddot(_b_2_d, _b_2_d_dot, _b_2_d_ddot)

    R_d = [cross(b_2_d, b_3_d)  b_2_d  b_3_d]
    R_d_dot = [(cross(b_2_d_dot, b_3_d) + cross(b_2_d, b_3_d_dot)) b_2_d_dot b_3_d_dot]
    R_d_ddot = [(cross(b_2_d_ddot, b_3_d) + cross(b_2_d_dot, b_3_d_dot) + cross(b_2_d, b_3_d_ddot)) b_2_d_ddot b_3_d_ddot]
    ω_d_hat = R_d' * R_d_dot  # guessed from the equation between [1, Eq. (10)] and [1, Eq. (11)].
    ω_d = _vee(ω_d_hat)
    ω_d_dot_hat = R_d_dot' * R_d_dot + R_d' * R_d_ddot
    ω_d_dot = _vee(ω_d_dot_hat)

    e_R = 0.5 * _vee(R_d'*R - R'*R_d)
    e_ω = ω - R'*R_d*ω_d

    f = dot(_b_3_d, R*e_3)
    M = -k_R*e_R - k_ω*e_ω + cross(ω, J*ω) - J*(_hat(ω) * R' * R_d * ω_d - R' * R_d * ω_d_dot)
    ν = [f, M...]
end


"""
R is defined as the rotation matrix in [1, Eqs.(3), (4)].
Please check if it is the transpose of the rotation matrix defined in FSimZoo multicopters.

This assumes that you know the exact equations of motion to get the acceleration `a` and jerk `a_dot`.
For more realistic situations, see `Command(controller::GeometricTrackingController, args...; kwargs...)`.
"""
function Command_Ideal(
        controller::GeometricTrackingController, p, v, R, ω;
        p_d, v_d, a_d, a_d_dot, a_d_ddot,
        b_1_d, b_1_d_dot, b_1_d_ddot,
        m, g, J,
    )
    (; k_p, k_v, k_R, k_ω) = controller
    e_3 = [0, 0, 1]
    ω_hat = _hat(ω)
    R_dot = R*ω_hat  # kinematics; available

    e_p = p - p_d
    e_v = v - v_d
    _b_3_d = -(-k_p*e_p - k_v*e_v - m*g*e_3 + m*a_d)
    f = dot(_b_3_d, R*e_3)

    a = g*e_3 - (1/m)*f*(R*e_3)  # assumption: we know the dynamics
    e_a = a - a_d
    _b_3_d_dot = -(-k_p*e_v - k_v*e_a + m*a_d_dot)
    f_dot = dot(_b_3_d_dot, R*e_3) + dot(_b_3_d, R_dot*e_3)

    a_dot = -(1/m) * (f_dot * R*e_3 + f * R_dot*e_3)
    e_a_dot = a_dot - a_d_dot

    _b_3_d_ddot = -(-k_p*e_a - k_v*e_a_dot + m*a_d_ddot)
    b_3_d = _b_3_d / norm(_b_3_d)
    b_3_d_dot = _unit_vector_dot(_b_3_d, _b_3_d_dot)
    b_3_d_ddot = _unit_vector_ddot(_b_3_d, _b_3_d_dot, _b_3_d_ddot)

    _b_2_d = cross(b_3_d, b_1_d)
    _b_2_d_dot = cross(b_3_d_dot, b_1_d) + cross(b_3_d, b_1_d_dot)
    _b_2_d_ddot = cross(b_3_d_ddot, b_1_d) + cross(b_3_d_dot, b_1_d_dot) + cross(b_3_d, b_1_d_ddot)
    b_2_d = _b_2_d / norm(_b_2_d)
    b_2_d_dot = _unit_vector_dot(_b_2_d, _b_2_d_dot)
    b_2_d_ddot = _unit_vector_ddot(_b_2_d, _b_2_d_dot, _b_2_d_ddot)

    R_d = [cross(b_2_d, b_3_d)  b_2_d  b_3_d]
    R_d_dot = [(cross(b_2_d_dot, b_3_d) + cross(b_2_d, b_3_d_dot)) b_2_d_dot b_3_d_dot]
    R_d_ddot = [(cross(b_2_d_ddot, b_3_d) + cross(b_2_d_dot, b_3_d_dot) + cross(b_2_d, b_3_d_ddot)) b_2_d_ddot b_3_d_ddot]
    ω_d_hat = R_d' * R_d_dot  # guessed from the equation between [1, Eq. (10)] and [1, Eq. (11)].
    ω_d = _vee(ω_d_hat)
    ω_d_dot_hat = R_d_dot' * R_d_dot + R_d' * R_d_ddot
    ω_d_dot = _vee(ω_d_dot_hat)

    e_R = 0.5 * _vee(R_d'*R - R'*R_d)
    e_ω = ω - R'*R_d*ω_d

    M = -k_R*e_R - k_ω*e_ω + cross(ω, J*ω) - J*(_hat(ω) * R' * R_d * ω_d - R' * R_d * ω_d_dot)
    ν = [f, M...]
    return ν
end
