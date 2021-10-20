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
export TwoDimensionalNonlinearPolynomialSystem, LinearSystem, ReferenceModel, MultipleEnvs
export TwoDimensionalNonlinearOscillator, SingleIntegrator
## controllers
export LQR, PID, BacksteppingPositionController
## multicopters
export Multicopter
export Quadcopter, IslamQuadcopter, GoodarziQuadcopter
export Hexacopter, LeeHexacopter
# control allocator
export AbstractAllocator, StaticAllocator
export PseudoInverseAllocator
# integrated environments
export BacksteppingPositionController_StaticAllocator_Multicopter

## utils
export ned2enu, enu2ned


include("environments/environments.jl")
include("utils/utils.jl")

end
