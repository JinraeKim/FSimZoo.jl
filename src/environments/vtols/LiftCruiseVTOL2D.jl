"""
# Refs
[1] J. L. Bullock, S. Cheng, A. Patterson, M. J. Acheson, N. Hovakimyan, and I. M. Gregory, “Reference Command Optimization for the Transition Flight Mode of a Lift Plus Cruise Vehicle,” in AIAA SCITECH 2024 Forum, Orlando, FL: American Institute of Aeronautics and Astronautics, Jan. 2024. doi: 10.2514/6.2024-0721.
"""
Base.@kwdef struct LiftCruiseVTOL2D <: AbstractEnv
    m = 0.228*10  # 10x quad
    J = 2.428e-3*10  # 10x quad
    g = 9.81
    CL_0 = 0.4808
    CL_θ = 3.848
    CL_δ_e = 0.2
    CD1_0 = 0.027
    CD1_k = 0.07*10  # 10x worse than cessna
    CD2_0 = 1.85
    δ_e_min = deg2rad(-30)
    δ_e_max = deg2rad(+30)
    θ_min = deg2rad(-60)
    θ_max = deg2rad(+60)
end


function State(plant::LiftCruiseVTOL2D)
    return function (p=zeros(2), θ=0, ṗ=zeros(2), θ̇=0)
        return ComponentArray(; p, θ, ṗ, θ̇)
    end
end


function aerodynamic_forces(plant::LiftCruiseVTOL2D, θ, ṗ; δ_e)
    (; CL_0, CL_θ, CL_δ_e, CD1_0, CD1_k, CD2_0) = plant
    R = angle2rotmatrix2d(θ)
    uv = R'*ṗ
    u, v = uv
    CL = CL_0 + CL_θ * θ + CL_δ_e * δ_e
    CD1 = CD1_0 + CD1_k * CL^2
    CD2 = CD2_0
    L = u^2 * CL
    D1 = u^2 * CD1
    D2 = v^2 * CD2
    return L, D1, D2
end


function Dynamics!(plant::LiftCruiseVTOL2D)
    (; m, g, J) = plant
    @Loggable function dynamics!(dX, X, param, t; u)
        e_2 = [0, 1]
        (; p, θ, ṗ, θ̇) = X
        (; T_r, T_p, M, δ_e) = u
        @log p
        @log ṗ
        @log θ
        @log θ̇
        @log M
        R = angle2rotmatrix2d(θ)
        δ_e_cmd = δ_e
        @log δ_e_cmd
        δ_e = min(plant.δ_e_max, max(plant.δ_e_min, δ_e_cmd))
        @log δ_e
        T_r_cmd = T_r
        T_p_cmd = T_p
        @log T_r_cmd
        @log T_p_cmd
        T_r = max(0, T_r_cmd)
        T_p = max(0, T_p_cmd)
        @log T_r
        @log T_p
        L, D1, D2 = aerodynamic_forces(plant, θ, ṗ; δ_e)
        F = [
            T_p-D1,
            -(T_r+L-D2),
        ]
        p̈ = (1/m) * (R'*F + m*g*e_2)
        @log p̈
        dX.p .= ṗ
        dX.θ = θ̇
        dX.ṗ .= p̈
        dX.θ̇ = M/J
    end
end
