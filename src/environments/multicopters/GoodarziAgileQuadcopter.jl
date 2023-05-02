"""
# Notes
- [1] (quad-+ configuration); see [2].

# References
[1] F. A. Goodarzi, D. Lee, and T. Lee, “Geometric Adaptive Tracking Control of a Quadrotor Unmanned Aerial Vehicle on SE(3) for Agile Maneuvers,” Journal of Dynamic Systems, Measurement, and Control, vol. 137, no. 9, p. 091007, Sep. 2015, doi: 10.1115/1.4030419.
[2] PX4 Airframes Reference, https://docs.px4.io/master/en/airframes/airframe_reference.html.
# Variables
u ∈ R^4: rotor forces
"""
Base.@kwdef struct GoodarziAgileQuadcopter <: Quadcopter
    J = 1e-2 * [
                5.5711 0.0618 -0.0251;
                0.06177 5.5757 0.0101;
                -0.02502 0.01007 1.05053;
               ]  # kg m^2
    l = 0.169  # m
    kM = 0.1056  # m
    m = 0.755  # kg
    g = 9.81  # m / s^2
    B = [ 1   1             1              1;
         -l   l             0              0;
          0   0             l             -l;
         kM  kM           -kM            -kM]
    # actuator limit
    dim_input = 4
    u_min = zeros(dim_input)
    u_max = 3.2 * ones(dim_input)
end

function saturate(multicopter::GoodarziAgileQuadcopter, u)
    (; u_min, u_max) = multicopter
    u_saturated = clamp.(u, u_min, u_max)
end

function input_to_force_moment(multicopter::GoodarziAgileQuadcopter, u)
    (; B) = multicopter
    ν = B * u
end

function airframe_reference(multicopter::GoodarziAgileQuadcopter)
    :quad_plus
end
