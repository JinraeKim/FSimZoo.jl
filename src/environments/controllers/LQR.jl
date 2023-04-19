abstract type AbstractLQR <: AbstractController end

"""
Infinite-horizon continuous-time linear quadratic regulator (LQR).
"""
struct LQR <: AbstractLQR
    A
    B
    Q
    R
end

function RunningCost(lqr::LQR)
    (; Q, R) = lqr
    return function (x, u)
        x'*Q*x + u'*R*u
    end
end

function ARE_solution(lqr::LQR)
    (; A, B, Q, R) = lqr
    P, _, _ = MatrixEquations.arec(A, B*inv(R)*B', Q)
    P
end

function optimal_gain(lqr::LQR)
    (; B, R) = lqr
    P = ARE_solution(lqr)
    K = inv(R) * B' * P
end

"""
Minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
"""
function Command(lqr::LQR)
    K = optimal_gain(lqr)
    return function (x)
        -K*x
    end
end

function solutions(lqr::LQR)
    P = ARE_solution(lqr)
    K = optimal_gain(lqr)
    optimal_controller = Command(lqr)
    P, K, optimal_controller
end
