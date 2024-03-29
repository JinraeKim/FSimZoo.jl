struct BacksteppingPositionController_StaticAllocator_Multicopter <: AbstractEnv
    controller::BacksteppingPositionController
    allocator::StaticAllocator
    multicopter::Multicopter
end

# outer constructor
function BacksteppingPositionController_StaticAllocator_Multicopter(pos_cmd_func=nothing; controller_kwargs...)
    multicopter = LeeHexacopter()
    (; m, B) = multicopter
    controller = BacksteppingPositionController(m; pos_cmd_func=pos_cmd_func, controller_kwargs...)
    allocator = PseudoInverseAllocator(B)
    env = BacksteppingPositionController_StaticAllocator_Multicopter(controller, allocator, multicopter)
end

function State(env::BacksteppingPositionController_StaticAllocator_Multicopter)
    (; controller, allocator, multicopter) = env
    (; m, g) = multicopter
    function state(; args_multicopter=())
        x_multicopter = State(multicopter)(args_multicopter...)
        pos0 = copy(x_multicopter.p)
        x_controller = State(controller)(pos0, m, g)
        ComponentArray(multicopter=x_multicopter, controller=x_controller)
    end
end

function Dynamics!(env::BacksteppingPositionController_StaticAllocator_Multicopter)
    (; controller, allocator, multicopter) = env
    (; m, J, g) = multicopter
    @Loggable function dynamics!(dx, x, params, t; pos_cmd=nothing)
        (; p, v, q, ω) = x.multicopter
        (; ref_model, Td) = x.controller
        xd, vd, ad, ȧd, äd = ref_model.x_0, ref_model.x_1, ref_model.x_2, ref_model.x_3, ref_model.x_4
        R = quat2dcm(q)
        νd, Ṫd, _... = Command(controller)(
                                           p, v, R, ω,
                                           xd, vd, ad, ȧd, äd, Td,
                                           m, J, g,
                                          )
        u_cmd = Command(allocator)(νd)
        @nested_log :multicopter _Dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; u=u_cmd)
        @nested_log :controller Dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        @nested_log :controller νd
    end
end
