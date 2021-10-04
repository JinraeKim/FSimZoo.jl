struct ConstantSystem <: AbstractEnv
end

function State(env::ConstantSystem)
    return function (x)
        x
    end
end

function Dynamics!(env::ConstantSystem)
    @Loggable function dynamics!(dx, x, p, t)
        @log state = x
        # nothing is updated...
    end
end
