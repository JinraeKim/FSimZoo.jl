using FSimZoo
using Test

@testset "FSimZoo.jl" begin
    include("geometric_tracking.jl")
    include("geometric_tracking_inner_outer.jl")
    include("cbf.jl")
end
