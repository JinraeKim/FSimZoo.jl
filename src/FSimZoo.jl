module FSimZoo

import FSimBase: State, Params, Dynamics!
import FSimBase: Command
using FSimBase
# using Reexport
# @reexport using FSimBase

using UnPack
using ComponentArrays
using LinearAlgebra
using StaticArrays
using MatrixEquations
using ForwardDiff

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


include("environments/environments.jl")

end
