# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/25/2014


module SISLES

export
    CorrAEM,
    LLAEM,
    SimplePilotResponse,
    StochasticLinearPR,
    SimpleADM,
    AirSpace,
    SimpleTCASSensor,
    SimpleCASCoord,
    SimpleTCAS,
    CoordSimpleTCAS,
    ACASX,
    TCASSimulator,
    SimpleTCAS_EvU,
    SimpleTCAS_EvE,
    ACASX_EvE,

    addObserver,
    outputGFormatString,

    # Encoutner
    getTrajectory,
    setInitialDistributions,
    validate,
    getInitialSample,

    # Simulator
    simulate


PACKAGE_PATH = Pkg.dir("SISLES", "src")

push!(LOAD_PATH, "$PACKAGE_PATH/include")
push!(LOAD_PATH, "$PACKAGE_PATH/common")
push!(LOAD_PATH, "$PACKAGE_PATH/common/ObserverImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter/CorrAEMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter/LLAEMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN/CorrAEMDBNImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse/SimplePilotResponseImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse/StochasticLinearPRImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel/SimpleADMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/WorldModel")
push!(LOAD_PATH, "$PACKAGE_PATH/WorldModel/AirSpaceImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor/SimpleTCASSensorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor/ACASXSensorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CASCoordination")
push!(LOAD_PATH, "$PACKAGE_PATH/CASCoordination/GenericCoordImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/SimpleTCASImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/CoordSimpleTCASImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/ACASXImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Simulator")
push!(LOAD_PATH, "$PACKAGE_PATH/Simulator/TCASSimulatorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/ACASX_EvE_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/SimpleTCAS_EvE_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/SimpleTCAS_EvU_Impl")


using Util
using Encounter
using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator
using GenerativeModel

end


