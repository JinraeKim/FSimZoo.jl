struct LinearSystem <: AbstractEnv
    A
    B
end

function State(env::LinearSystem)
    (; B) = env
    n = size(B)[1]
    return function (x)
        @assert length(x) == n
        x
    end
end

function Params(env::LinearSystem)
    () -> nothing
end

function Dynamics!(env::LinearSystem)
    (; A, B) = env
    @Loggable function dynamics!(dx, x, p, t; u)
        @log state = x
        @log input = u
        dx .= A*x + B*u
    end
end
