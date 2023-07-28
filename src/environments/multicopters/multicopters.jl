# multicopter
abstract type Multicopter <: AbstractEnv end

"""
Common state structure of Multicopter.
`State(multicopter::Multicopter)` now receives `η` (Euler angles, η=[roll, pitch, yaw]) instead of rotation matrix.
"""
function State(multicopter::Multicopter)
    return function (p=zeros(3), v=zeros(3), η=zeros(3), ω=zeros(3))
        q = euler2quat(η)
        ComponentArray(p=p, v=v, q=q, ω=ω)
    end
end

"""
Common input saturation (default: not applied)
"""
function saturate(multicopter::Multicopter, u)
    # u
    error("Actuator saturation not applied: `saturate`")
end

"""
Common input to force and moment transformation (default: not applied)
"""
function input_to_force_moment(multicopter::Multicopter, u)
    # ν = u
    error("Transformation of input to force and moment not defined: `input_to_force_moment`")
end


"""
skew(x): ℝ³ → ℝ⁹ such that x×y = skew(x)*y
"""
function skew(x)
    [    0 -x[3]  x[2];
      x[3]     0 -x[1];
     -x[2]  x[1]    0]
end


"""
q1, q2: vectors with length of 4 (quaternions)
q: the resulting array (q = [s, v1, v2, v3])
"""
function quaternion_product(q1, q2)
    _q1 = Quaternions.quat(q1...)
    _q2 = Quaternions.quat(q2...)
    _q = _q1 * _q2
    return [_q.s, _q.v1, _q.v2, _q.v3]
end


"""
# Variables
## State
p ∈ ℝ^3: (inertial) position
v ∈ ℝ^3: (inertial) velocity
R ∈ so(3): direction cosine matrix (DCM) that maps a vector read in Inertial (I)-coord.
to the same vector read in Body (B)-coord.
For example, v_B = R*v_I.
Or, it can be interpreted as "inverse rotation" from I-frame to B-frame.
For example, x̂_I = R'*[1, 0, 0] where x̂_I is the x-axis of B-frame read in I-coord.
ω ∈ ℝ^3: angular rate of body frame w.r.t. inertial frame (I to B)

## (Virtual) input
f ∈ ℝ: total thrust
M ∈ ℝ^3: moment

## Parameters
D denotes the drag coefficient matrix [2].

## BE CAREFUL
1) The default state is changed to unit quaternion from rotation matrix.
2) The definition of rotation matrix (`R`) is now the same as the DCM introduced in [1] (it was the DCM's transpose until v0.10).
This is for compatibility with Rotations.jl (rotation matrix from I to B frames).

# Reference
[1] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” in 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, Dec. 2010, pp. 5420–5425. doi: 10.1109/CDC.2010.5717652.
[2] M. Faessler, A. Franchi, and D. Scaramuzza, “Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 620–626, Apr. 2018, doi: 10.1109/LRA.2017.2776353.
"""
function __Dynamics!(multicopter::Multicopter)
    (; m, g, J, D) = multicopter
    J_inv = inv(J)
    @Loggable function dynamics!(dX, X, p, t; f, M)
        e3 = [0, 0, 1]
        (; p, v, q, ω) = X
        @onlylog state = X
        @nested_log :input f, M
        R = quat2dcm(q)
        dX.p = v
        dX.v = -(1/m)*f*R*e3 + g*e3 - R*D*R'*v
        qω_dot = attitude_dynamics(vcat(q, ω); M, J)
        dX.q = qω_dot[1:4]
        dX.ω = qω_dot[5:7]
    end
end

"""
A basic example of dynamics for multicopter considering rotor inputs `u`.
You can use the following closure or extend the above __Dynamics! for more general
models, e.g., faulted multicopters.
"""
function _Dynamics!(multicopter::Multicopter)
    @Loggable function dynamics!(dx, x, p, t; u)
        @nested_onlylog :input u_cmd = u
        @nested_log :input u_saturated = saturate(multicopter, u)
        @nested_log :input ν = input_to_force_moment(multicopter, u_saturated)
        f, M = ν[1], ν[2:4]
        @nested_log __Dynamics!(multicopter)(dx, x, (), t; f=f, M=M)
    end
end


"""
[1] MATLAB, https://kr.mathworks.com/help/aeroblks/6dofquaternion.html#mw_f692de78-a895-4edc-a4a7-118228165a58
"""
function attitude_dynamics(qω; M, J)
    q = qω[1:4]
    ω = qω[5:7]
    Ω = skew(ω)
    q_dot = 0.5 * quaternion_product(q, [0, ω...])
    k = 1
    eps = 1 - sum(q .^ 2)
    q_dot = q_dot + k*eps*q
    ω_dot = inv(J) * (-Ω*J*ω + M)
    vcat(q_dot, ω_dot)
end


# multicopters
include("quadcopters.jl")
include("hexacopters.jl")
