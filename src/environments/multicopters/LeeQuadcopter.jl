"""
# Notes
- [1] (quad-+ configuration); see [2].
# References
[1] T. Lee, M. Leok, and N. H. McClamroch,
“Geometric Tracking Control of a Quadrotor UAV on SE(3),”
in 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, Dec. 2010, pp. 5420–5425. doi: 10.1109/CDC.2010.5717652.
[2] PX4 Airframes Reference, https://docs.px4.io/master/en/airframes/airframe_reference.html.
[3] M. C. Achtelik, K. M. Doth, D. Gurdan, and J. Stumpf, “Design of a multi rotor MAV with regard to efficiency, dynamics and redundancy,” AIAA Guid. Navig. Control Conf. 2014, no. August, pp. 1–17, 2012, doi: 10.2514/6.2012-4779.
[4] https://github.com/fdcl-ftc/fault-tolerant-control/issues/62
# Variables
u ∈ R^6: rotor forces
"""
Base.@kwdef struct LeeQuadcopter <: Quadcopter
    J = diagm([0.0820, 0.0845, 0.1377])  # kg m^2
    l = 0.315  # m
    kM = 8.004e-4  # m
    m = 4.34  # kg
    g = 9.81  # m / s^2
    B = [ 1   1             1              1;
         -l   l             0              0;
          0   0             l             -l;
         kM  kM           -kM            -kM]
    D = diagm(zeros(3))
    # actuator limit
    dim_input = 4
    u_min = zeros(dim_input)
    u_max = (6/4) * 0.6371 * (m*g) * ones(dim_input)  # [3] and [4]; modified for quadcopter
end

function saturate(multicopter::LeeQuadcopter, u)
    (; u_min, u_max) = multicopter
    u_saturated = clamp.(u, u_min, u_max)
end

function input_to_force_moment(multicopter::LeeQuadcopter, u)
    (; B) = multicopter
    ν = B * u
end

function airframe_reference(multicopter::LeeQuadcopter)
    :quad_plus
end
