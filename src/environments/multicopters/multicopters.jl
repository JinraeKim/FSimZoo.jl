# multicopter
abstract type Multicopter <: AbstractEnv end

"""
Common state structure of Multicopter
"""
function State(multicopter::Multicopter)
    return function (p=zeros(3), v=zeros(3), R=SMatrix{3, 3}(I), ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
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

## BE CAREFUL
The definition of DCM, R, is the transpose of the DCM introduced in [1].
It is because many packages including `ReferenceFrameRotations.jl` follows the definition of
"DCM such that v_B = R*v_I".

# Reference
[1] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” in 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, Dec. 2010, pp. 5420–5425. doi: 10.1109/CDC.2010.5717652.
"""
function __Dynamics!(multicopter::Multicopter)
    @unpack m, g, J = multicopter
    J_inv = inv(J)
    e3 = [0, 0, 1]
    # skew(x): ℝ³ → ℝ⁹ such that x×y = skew(x)*y
    skew(x) = [    0 -x[3]  x[2];
                x[3]     0 -x[1];
               -x[2]  x[1]    0]
    @Loggable function dynamics!(dX, X, p, t; f, M)
        @unpack p, v, R, ω = X
        @onlylog state = X
        @nested_log :input f, M
        Ω = skew(ω)
        dX.p = v
        dX.v = -(1/m)*f*R'*e3 + g*e3
        dX.R = -Ω*R
        dX.ω = J_inv * (-Ω*J*ω + M)
    end
end

"""
A basic example of dynamics for multicopter considering rotor inputs `u`.
You can use the following closure or extend the abvoe __Dynamics! for more general
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


# multicopters
include("quadcopters.jl")
include("hexacopters.jl")
