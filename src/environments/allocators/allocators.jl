abstract type AbstractAllocator <: AbstractController end
abstract type StaticAllocator <: AbstractAllocator end

include("PseudoInverseAllocator.jl")
