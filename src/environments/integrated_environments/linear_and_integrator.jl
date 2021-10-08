"""
LinearSystem + SingleIntegrator
"""
function LinearSystem_SingleIntegrator(args_linearsystem)
    linearsystem = LinearSystem(args_linearsystem...)
    integ = SingleIntegrator()
    linearsystem, integ
end

"""
# Notes
- `r` denotes integrand of `integ`.
"""
function State(linearsystem::LinearSystem, integ::SingleIntegrator)
    return function (x0_linearsystem, x0_integrator=[0.0])
        x_linearsystem = State(linearsystem)(x0_linearsystem)
        x_integrator = State(integ)(x0_integrator)
        ComponentArray(x=x_linearsystem, ∫r=x_integrator)
    end
end

"""
# Notes
running_cost: integrand; function of (x, u)
"""
function Dynamics!(linearsystem::LinearSystem, integ::SingleIntegrator, running_cost::Function)
    @Loggable function dynamics!(dX, X, p, t; u)
        @unpack x = X
        @nested_log :linearsystem Dynamics!(linearsystem)(dX.x, X.x, (), t; u=u)
        r = running_cost(x, u)
        @nested_log :integ Dynamics!(integ)(dX.∫r, X.∫r, (), t; u=r)
    end
end
