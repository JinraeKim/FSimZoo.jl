module FSimZoo

import FSimBase: State, Params, Dynamics!
import FSimBase: Command
using FSimBase

using ComponentArrays
using ForwardDiff
using MatrixEquations
using LinearAlgebra
using Convex, ECOS
using Rotations, Quaternions

## basics
export TwoDimensionalNonlinearPolynomialSystem, LinearSystem, ReferenceModel, MultipleEnvs
export TwoDimensionalNonlinearOscillator, SingleIntegrator
export TwoDimensionalNonlinearDTSystem
## controllers
export LQR, PID, BacksteppingPositionController
export GeometricTrackingController, OuterLoopGeometricTrackingController, InnerLoopGeometricTrackingController
export InputAffinePositionCBF
## actuators
export SecondOrderActuator
## fixedwings
export AbstractWingRock, ElzebdaWingRock, TarnWingRock
## multicopters
export Multicopter
export Quadcopter, IslamQuadcopter, GoodarziQuadcopter, LeeQuadcopter, GoodarziAgileQuadcopter
export Hexacopter, LeeHexacopter
## missiles
export MissileLongitudinal
# control allocator
export AbstractAllocator, StaticAllocator
export PseudoInverseAllocator
# integrated environments
export BacksteppingPositionController_StaticAllocator_Multicopter

## utils
export ned2enu, enu2ned
export euler2quat, quat2euler, dcm2euler, euler2dcm, quat2dcm, dcm2quat


include("environments/environments.jl")
include("utils/utils.jl")

end
