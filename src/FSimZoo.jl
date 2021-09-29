module FSimZoo

import FSimBase: State, Params, Dynamics!
import FSimBase: Command
using FSimBase

using ComponentArrays
using ForwardDiff
using MatrixEquations
using StaticArrays
using UnPack
using LinearAlgebra

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
# integrated environments
export BacksteppingPositionController_StaticAllocator_MulticopterEnv

## utils
export ned2enu, enu2ned


include("environments/environments.jl")
include("utils/utils.jl")

end
