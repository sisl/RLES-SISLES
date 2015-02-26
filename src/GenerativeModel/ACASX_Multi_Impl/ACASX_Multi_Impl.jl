# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 02/10/2014

module ACASX_Multi_Impl
#TODO: There's a lot of overlap amongst the GenerativeModels, should consolidate

using AbstractGenerativeModelImpl
using AbstractGenerativeModelInterfaces
using CommonInterfaces
using ObserverImpl

using Base.Test
using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractGenerativeModelInterfaces.get
import AbstractGenerativeModelInterfaces.isEndState

import CommonInterfaces.addObserver

export ACASX_Multi_params, ACASX_Multi, initialize, step, get, isEndState, addObserver

type ACASX_Multi_params
  #global params: remains constant per sim
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  maxSteps::Int64 #maximum number of steps in sim
  number_of_aircraft::Int64 #number of aircraft  #must be 2 for now...
  encounter_seed::Uint64 #Seed for generating encounters
  encounterModel::Symbol #{:PairwiseCorrAEMDBN, :StarDBN}
  pilotResponseModel::Symbol #{:SimplePR, :StochasticLinear, :DetVsNone}

  #these are to define AEM:
  encounter_file::String #Path to encounter file

  #ACASX config
  quant::Int64 #quantization. Typ. quant=25
  libcas_config_file::String #Path to libcas config file

  string_id::String

  ACASX_Multi_params() = new()
end

type ACASX_Multi <: AbstractGenerativeModel
  params::ACASX_Multi_params

  #sim objects: contains state that changes throughout sim run
  em::Union(StarDBN,PairwiseCorrAEMDBN)
  pr::Vector{Union(SimplePilotResponse,StochasticLinearPR,DeterministicPR)}
  dm::Vector{SimpleADM}
  wm::AirSpace
  coord::GenericCoord
  sr::Vector{ACASXSensor}
  cas::Vector{ACASX}

  observer::Observer

  #sim states: changes throughout simulation run
  t_index::Int64 #current time index in the simulation. Starts at 1 and increments by 1.
  #This is different from t which starts at 0 and could increment in the reals.

  #empty constructor
  function ACASX_Multi(p::ACASX_Multi_params)
    sim = new()
    sim.params = p

    if p.encounterModel == :PairwiseCorrAEMDBN
      sim.em = PairwiseCorrAEMDBN(p.number_of_aircraft, p.encounter_file, p.encounter_seed)
    elseif p.encounterModel == :StarDBN
      sim.em = StarDBN(p.number_of_aircraft, p.encounter_file, p.encounter_seed)
    else
      error("ACASX_Multi_Impl: No such encounter model")
    end

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :FiveVsNone
      sim.pr = DeterministicPR[ i==1 ? DeterministicPR(5) :
                                 DeterministicPR(-1) for i=1:p.number_of_aircraft]
    else
      error("ACASX_Multi_Impl: No such pilot response model")
    end

    sim.dm = SimpleADM[ SimpleADM(number_of_substeps=1) for i=1:p.number_of_aircraft ]
    sim.wm = AirSpace(p.number_of_aircraft)

    sim.coord = GenericCoord(p.number_of_aircraft)

    max_intruders = p.number_of_aircraft-1
    sim.sr = ACASXSensor[ ACASXSensor(i,max_intruders) for i=1:p.number_of_aircraft ]
    sim.cas = ACASX[ ACASX(i,p.quant,p.libcas_config_file,p.number_of_aircraft,sim.coord)
                    for i=1:p.number_of_aircraft ]

    sim.observer = Observer()

    #Start time at 1 for easier indexing into arrays according to time
    sim.t_index = 1

    return sim
  end
end

addObserver(sim::ACASX_Multi, f::Function) = _addObserver(sim, f)
addObserver(sim::ACASX_Multi, tag::String, f::Function) = _addObserver(sim, tag, f)

function initialize(sim::ACASX_Multi)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  #Start time at 1 for easier indexing into arrays according to time
  sim.t_index = 1

  EncounterDBN.initialize(aem)

  for i = 1:sim.params.number_of_aircraft
    initial = EncounterDBN.getInitialState(aem, i)
    notifyObserver(sim,"Initial",[i, sim.t_index, initial])

    state = DynamicModel.initialize(adm[i], initial)

    WorldModel.initialize(wm, i, state)

    Sensor.initialize(sr[i])
    notifyObserver(sim,"Sensor",[i, sim.t_index, sr[i]])

    CollisionAvoidanceSystem.initialize(cas[i])
    notifyObserver(sim,"CAS", [i, sim.t_index, cas[i]])

    PilotResponse.initialize(pr[i])
    notifyObserver(sim,"Response",[i, sim.t_index, pr[i]])
  end

  notifyObserver(sim,"WorldModel", [sim.t_index, wm])

  return
end

function step(sim::ACASX_Multi)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  sim.t_index += 1

  logProb = 0.0 #track the probabilities in this update

  cmdLogProb = EncounterDBN.step(aem)
  logProb += cmdLogProb #TODO: decompose this by aircraft?

  states = WorldModel.getAll(wm)

  for i = 1:sim.params.number_of_aircraft

    #intended command
    command = EncounterDBN.get(aem,i)
    notifyObserver(sim,"Command",[i, sim.t_index, command])

    output = Sensor.step(sr[i], states)
    notifyObserver(sim,"Sensor",[i, sim.t_index, sr[i]])

    RA = CollisionAvoidanceSystem.step(cas[i], output)
    notifyObserver(sim,"CAS", [i, sim.t_index, cas[i]])

    response = PilotResponse.step(pr[i], command, RA)
    logProb += response.logProb
    notifyObserver(sim,"Response",[i, sim.t_index, pr[i]])

    state = DynamicModel.step(adm[i], response)
    WorldModel.step(wm, i, state)

  end

  WorldModel.updateAll(wm)
  notifyObserver(sim,"WorldModel", [sim.t_index, wm])

  return logProb
end

function getvhdist(wm::AbstractWorldModel)

  states = WorldModel.getAll(wm) #states::Vector{ASWMState}

  #[(vdist,hdist)]
  vhdist = [(abs(s2.h-s1.h),norm([(s2.x-s1.x),(s2.y-s1.y)])) for s1 = states, s2 = states]
  for i = 1:length(states)
    vhdist[i,i] = (typemax(Float64),typemax(Float64))
  end

  return vhdist
end

function isNMAC(sim::ACASX_Multi)
  vhdist = getvhdist(sim.wm)
  nmac_test = map((vhd)->vhd[2] <= sim.params.nmac_r && vhd[1] <= sim.params.nmac_h,vhdist)
  return any(nmac_test)
end

isTerminal(sim::ACASX_Multi) = sim.t_index > sim.params.maxSteps

function isEndState(sim::ACASX_Multi)
  return isNMAC(sim) || isTerminal(sim)
end

end #module

