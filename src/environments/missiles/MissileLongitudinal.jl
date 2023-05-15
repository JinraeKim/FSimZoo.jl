"""
# Refs
[1] C. Mracek and J. Cloutier, “Full Envelope Missile Longitudinal Autopilot Design Using the State-Dependent Riccati Equation Method,” Guidance, Navigation, and Control Conference, Aug. 1997, pp. 1697-1705, doi:10.2514/6.1997-3767.
"""
Base.@kwdef struct MissileLongitudinal
    grav = 9.81
    # Physical Configuration (Table 3)
    S_ref = 0.0409 # Reference Area [m] (0.44 ft^2)
    d_ref = 0.2286 # Reference Distance [m] (0.75 ft)
    mass = 204.02 # Mass [kg] (13.98 slug) 
    I_yy = 247.438 # Moment of Inertia [kg * m^2] (182.5 sulg-ft^2)

    # Aerodynamic coefficients (Table 1 in Ref.)
    aₐ = 0.3
    aₘ = 40.44
    aₙ = 19.373
    bₘ = -64.015
    bₙ = -31.023
    cₘ = 2.922
    cₙ = -9.717
    dₘ = -11.803
    dₙ = -1.948
    eₘ = -1.719
end


"""
α: Angle of attack [rad]
q : Pitch rate [rad/s]
mach : Mach number -]
γ : Flight path angle [rad]
"""
function State(env::MissileLongitudinal)
    return function (α=deg2rad(0), q=deg2rad(0), M=4.0, γ=deg2rad(0), h=10_000)
        ComponentArray(α=α, q=q, M=M, γ=γ, h=h)
    end
end


function density(h)
    # Atmospheric coefficients (Table 2 in Ref.)
    tro_alt = 10972.8 # troposphere boundary [m] (36000.0 ft)
    str_alt = 20116.8 # stratosphere boundary [m] (66000.0 ft)
    ρ₀sl = 1.1455 # Air density at sea level [kg/m^3] (2.377e-3 lb-s^2/ft^2)
    ρ₀tr = 0.3415 # Air density at troposphere boundary [kg/m^3] (0.7086e-3 lb-s^2/ft^2)
    Kρsl = 1.1103e-4 # [1/m] (3.36174e-5 1/ft)
    Kρtr = 1.5760e-4 # [1/m] (4.80377e-5 1/ft)

    @assert h >= 0.0 #, "Altitude must be greater than zero"
    @assert h <= str_alt #, "Out of operating altitude range"
    if h <= tro_alt
        return ρ₀sl * exp(-Kρsl * h)
    else
        return ρ₀tr * exp(-Kρtr*(h - tro_alt))
    end
end


function sonic_speed(h)
    # Atmospheric coefficients (Ttable 2 in Ref.)
    tro_alt = 10972.8 # troposphere boundary [m] (36000.0 ft)
    str_alt = 20116.8 # stratosphere boundary [m] (66000.0 ft)
    asl = 340.2787 # Sonic speed at the sea level [m/s] (1116.4 ft/sec)
    atr = 295.0769 # Sonic speed at the troposphere boundary [m/s] (968.1 ft/sec)
    Ka = 1.0358e-2 # [1/m] (0.00410833 1/ft)

    @assert h >= 0.0 #, "Altitude must be greater than zero"
    @assert h <= str_alt #, "Out of operating altitude range"
    if h <= tro_alt
       return asl - Ka * h
    else
       return atr
    end
end


function Dynamics(env::MissileLongitudinal)
    (;
     grav, S_ref, d_ref, mass, I_yy,
     aₐ, aₘ, aₙ, bₘ, bₙ, cₘ, cₙ, dₘ, dₙ, eₘ,
    ) = env

    function (x, params, t; δ)
        (; α, q, M, γ, h) = x

        Ca() = aₐ
        Cz(α, M, δ) = (aₙ*α^3 + bₙ* α * abs(α) + cₙ * (2.0 - M/3.0)*α +dₙ *δ)
        Cm(α, M, δ, q) = (aₘ*α^3 + bₘ* α * abs(α) + cₘ * (-7.0 + 8.0*M/3.0) *α +dₘ *δ + eₘ*q)

        Vₛ = sonic_speed(h)
        ρ = density(h)    

        α_dot = ρ * Vₛ * M * S_ref / (2.0 * mass) * (Cz(α, M, δ) * cos(α) - Ca() * sin(α)) + grav/(Vₛ * M) *cos(γ)+ q
        q_dot = ρ * Vₛ^2 * M^2 * S_ref * d_ref / (2.0 * I_yy) * Cm(α, M, δ, q)
        M_dot = ρ * Vₛ * M^2 * S_ref / (2.0 * mass) * (Cz(α, M, δ) * sin(α) + Ca() * cos(α)) - grav/Vₛ * sin(γ)
        γ_dot = -ρ * Vₛ * M * S_ref / (2.0 * mass) * (Cz(α, M, δ) * cos(α) - Ca() * sin(α)) - grav/(Vₛ * M) * cos(γ)
        h_dot = M * Vₛ * sin(γ)
        dx = [α_dot, q_dot, M_dot, γ_dot, h_dot] 
    end
end


function Dynamics!(env::MissileLongitudinal)
    dynamics = Dynamics(env)
    @Loggable function dynamics!(dx, x, params, t; δ)
        @log state = x
        @log input = δ
        dx .= dynamics(x, params, t; δ=δ)
    end
end
