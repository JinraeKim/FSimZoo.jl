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
export TwoDimensionalNonlinearDTSystem
## controllers
export LQR, PID, BacksteppingPositionController, GeometricTrackingController
## multicopters
export Multicopter
export Quadcopter, IslamQuadcopter, GoodarziQuadcopter, LeeQuadcopter
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
