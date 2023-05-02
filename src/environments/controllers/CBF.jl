struct InputAffinePositionCBF <: AbstractEnv
    f
    g
    h
    α1
    α2
end


"""
p in R^3
v in R^3
u: Convex.Variable(m)
"""
function generate_constraint(cbf::InputAffinePositionCBF, p, v, u)
    (; f, g, h, α1, α2) = cbf
    ∇h = p -> ForwardDiff.gradient(h, p)
    h1 = (p, v) -> ∇h(p)' * v + α1(h(p))
    ∇h1_p = (p, v) -> ForwardDiff.gradient(p -> h1(p, v), p)
    ∇h1_v = (p, v) -> ForwardDiff.gradient(v -> h1(p, v), v)
    h2 = (p, v) -> ∇h1_p(p, v)' * v + ∇h1_v(p, v)' * (f(p, v)+g(p, v)*u) + α2(h1(p, v))
    constraint = h2(p, v) >= 0.0
end


"""
p in R^3
v in R^3
u_nom: nominal control input, in R^m
"""
function Command(
        cbf::InputAffinePositionCBF, p, v, u_nom, constraints;
        solver=ECOS.Optimizer, silent_solver=true,
    )
    u = Convex.Variable(length(u_nom))
    cbf_constraint = generate_constraint(cbf, p, v, u)
    constraints = [constraints..., cbf_constraint]
    problem = minimize(sumsquares(u - u_nom), constraints)
    solve!(problem, solver; silent_solver=silent_solver)
    return reshape(u.value, size(u_nom)...)
end
