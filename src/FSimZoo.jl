module FSimZoo

using FSimBase
import FSimBase: State, Params, Dynamics!, Command

using UnPack
using ComponentArrays
using LinearAlgebra
using StaticArrays


## basics
export TwoDimensionalNonlinearPolynomialEnv, LinearSystemEnv, ReferenceModelEnv, MultipleEnvs
## controllers
export LQR, PID, BacksteppingPositionControllerEnv
## multicopters
export MulticopterEnv
export QuadcopterEnv, IslamQuadcopterEnv, GoodarziQuadcopterEnv
export HexacopterEnv, LeeHexacopterEnv
# control allocator
export AbstractAllocator, StaticAllocator
export PseudoInverseAllocator


include("environments/environments.jl")

end
