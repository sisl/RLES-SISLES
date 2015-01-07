# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 12/11/2014

module ACASX_EvE_Impl

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

export ACASX_EvE_params, ACASX_EvE, initialize, step, get, isEndState, addObserver

type ACASX_EvE_params
  #global params: remains constant per sim
  encounter_number::Int64 #encounter number in file
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  maxSteps::Int64 #maximum number of steps in sim
  number_of_aircraft::Int64 #number of aircraft  #must be 2 for now...
  encounter_seed::Uint64 #Seed for generating encounters
  pilotResponseModel::Symbol #{:SimplePR, :StochasticLinear, :DetVsNone}

  #Defines behavior of CorrAEMDBN.  Read from file or generate samples on-the-fly
  command_method::Symbol #:DBN=sampled from DBN or :ENC=from encounter file

  #these are to define CorrAEM:
  encounter_file::String #Path to encounter file
  initial_sample_file::String #Path to initial sample file
  transition_sample_file::String #Path to transition sample file

  #ACASX config
  quant::Int64 #quantization. Typ. quant=25
  libcas_config_file::String #Path to libcas config file

  ACASX_EvE_params() = new()
end

type ACASX_EvE <: AbstractGenerativeModel
  params::ACASX_EvE_params

  #sim objects: contains state that changes throughout sim run
  em::CorrAEMDBN
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
  function ACASX_EvE(p::ACASX_EvE_params)
    @test p.number_of_aircraft == 2 #need to revisit the code if this is not true

    sim = new()
    sim.params = p

    srand(p.encounter_seed) #There's a rand inside generateEncounter, need to control it
    sim.em = CorrAEMDBN(p.number_of_aircraft, p.encounter_file, p.initial_sample_file,
                    p.transition_sample_file,
                    p.encounter_number,p.encounter_seed,p.command_method)

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :FiveVsNone
      sim.pr = DeterministicPR[ i==1 ? DeterministicPR(5) :
                                 DeterministicPR(-1) for i=1:p.number_of_aircraft]
    else
      error("ACASX_EvE_Impl: No such pilot response model")
    end

    sim.dm = SimpleADM[ SimpleADM(number_of_substeps=1) for i=1:p.number_of_aircraft ]
    sim.wm = AirSpace(p.number_of_aircraft)

    sim.coord = GenericCoord(p.number_of_aircraft)

    max_intruders = p.number_of_aircraft-1
    sim.sr = ACASXSensor[ ACASXSensor(1,max_intruders), ACASXSensor(2,max_intruders) ]
    sim.cas = ACASX[ ACASX(1,p.quant,p.libcas_config_file,p.number_of_aircraft,sim.coord),
                    ACASX(2,p.quant,p.libcas_config_file,p.number_of_aircraft,sim.coord) ]

    sim.observer = Observer()

    #Start time at 1 for easier indexing into arrays according to time
    sim.t_index = 1

    return sim
  end
end

addObserver(sim::ACASX_EvE, f::Function) = _addObserver(sim, f)
addObserver(sim::ACASX_EvE, tag::String, f::Function) = _addObserver(sim, tag, f)

function initialize(sim::ACASX_EvE)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  #Start time at 1 for easier indexing into arrays according to time
  sim.t_index = 1

  EncounterDBN.initialize(aem)

  for i = 1:sim.params.number_of_aircraft
    initial = EncounterDBN.getInitialState(aem, i)
    notifyObserver(sim,"Command",[i, sim.t_index, initial])

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

function step(sim::ACASX_EvE)
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
  states_1,states_2 = WorldModel.getAll(wm) #states::Vector{ASWMState}
  x1, y1, h1 = states_1.x, states_1.y, states_1.h
  x2, y2, h2 = states_2.x, states_2.y, states_2.h

  vdist = abs(h2-h1)
  hdist = norm([(x2-x1),(y2-y1)])

  return vdist,hdist
end

function isNMAC(sim::ACASX_EvE)
  vdist,hdist = getvhdist(sim.wm)
  return  hdist <= sim.params.nmac_r && vdist <= sim.params.nmac_h
end

isTerminal(sim::ACASX_EvE) = sim.t_index > sim.params.maxSteps

function isEndState(sim::ACASX_EvE)
  return isNMAC(sim) || isTerminal(sim)
end

end #module

