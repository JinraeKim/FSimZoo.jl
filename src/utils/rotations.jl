# 3D rotation
"""
Euler angle to unit quaternion, corresponding to ZYX rotation (here, RotXYZ).
η = [roll, pitch, yaw]
q = [s, v1, v2, v3]
R ∈ ℝ^(3×3)
"""
function euler2quat(η)
    tmp = QuatRotation(RotXYZ(η...)).q
    q = [tmp.s, tmp.v1, tmp.v2, tmp.v3]
end


function quat2euler(q)
    tmp = RotXYZ(QuatRotation(q...))
    η = [tmp.theta1, tmp.theta2, tmp.theta3]
end


function dcm2euler(R)
    tmp = RotXYZ(RotMatrix{3}(R))
    η = [tmp.theta1, tmp.theta2, tmp.theta3]
end


function euler2dcm(η)
    RotMatrix{3}(RotXYZ(η...)).mat
end


function quat2dcm(q)
    RotMatrix{3}(QuatRotation(q)).mat
end


function dcm2quat(R)
    tmp = QuatRotation(RotMatrix{3}(R)).q
    q = [tmp.s, tmp.v1, tmp.v2, tmp.v3]
end


# 2D rotation
function angle2rotmatrix2d(θ)
    mat = [
        cos(θ) -sin(θ);
        sin(θ) cos(θ)
    ]
    return mat
end
