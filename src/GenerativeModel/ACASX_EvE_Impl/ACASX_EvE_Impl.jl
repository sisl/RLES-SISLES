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
  num_aircraft::Int64 #number of aircraft  #must be 2 for now...
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
  libcas::String #Path to libcas library
  libcas_config::String #Path to libcas config file

  ACASX_EvE_params() = new()
end

type ACASX_EvE <: AbstractGenerativeModel
  params::ACASX_EvE_params

  #sim objects: contains state that changes throughout sim run
  em::CorrAEMDBN
  pr::Vector{Union(SimplePilotResponse,StochasticLinearPR,LLDetPR)}
  dm::Vector{LLADM}
  wm::AirSpace
  coord::GenericCoord
  sr::Vector{ACASXSensor}
  cas::Vector{ACASX}

  observer::Observer
  string_id::String

  #sim states: changes throughout simulation run
  t_index::Int64 #current time index in the simulation. Starts at 1 and increments by 1.
  #This is different from t which starts at 0 and could increment in the reals.

  #empty constructor
  function ACASX_EvE(p::ACASX_EvE_params)
    @test p.num_aircraft == 2 #need to revisit the code if this is not true

    sim = new()
    sim.params = p

    sim.em = CorrAEMDBN(p.num_aircraft, p.encounter_file, p.initial_sample_file,
                    p.transition_sample_file,
                    p.encounter_number, p.encounter_seed, p.command_method)

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.num_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.num_aircraft ]
    elseif p.pilotResponseModel == :FiveVsNone
      sim.pr = LLDetPR[ i==1 ? LLDetPR(5,3) :
                                 LLDetPR(-1,-1) for i=1:p.num_aircraft]
    elseif p.pilotResponseModel == :ICAO_all
      sim.pr = LLDetPR[ LLDetPR(5,3) for i=1:p.num_aircraft]
    else
      error("ACASX_EvE_Impl: No such pilot response model")
    end

    sim.dm = LLADM[ LLADM() for i=1:p.num_aircraft]

    sim.wm = AirSpace(p.num_aircraft)
    sim.coord = GenericCoord(p.num_aircraft)

    max_intruders = p.num_aircraft-1
    sim.sr = ACASXSensor[ ACASXSensor(i,max_intruders) for i=1:p.num_aircraft ]
    sim.cas = ACASX[ ACASX(i, p.libcas, p.libcas_config, p.quant, p.num_aircraft, sim.coord)
                    for i=1:p.num_aircraft ]

    sim.observer = Observer()
    sim.string_id = "ACASX_EvE_$(p.encounter_number)"

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

  for i = 1:sim.params.num_aircraft
    #TODO: clean up this structure
    initial = EncounterDBN.getInitialState(aem, i)
    notifyObserver(sim,"Initial", Any[i, sim.t_index, aem])

    Sensor.initialize(sr[i])
    notifyObserver(sim,"Sensor", Any[i, sim.t_index, sr[i]])

    CollisionAvoidanceSystem.initialize(cas[i])
    notifyObserver(sim,"CAS", Any[i, sim.t_index, cas[i]])

    PilotResponse.initialize(pr[i])
    notifyObserver(sim,"Response", Any[i, sim.t_index, pr[i]])

    state = DynamicModel.initialize(adm[i], initial)
    notifyObserver(sim,"Dynamics",Any[i, sim.t_index, adm[i]])

    WorldModel.initialize(wm, i, state)
  end

  notifyObserver(sim,"WorldModel", Any[sim.t_index, wm])

  return
end

function step(sim::ACASX_EvE)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  sim.t_index += 1

  logProb = 0.0 #track the probabilities in this update

  cmdLogProb = EncounterDBN.step(aem)
  logProb += cmdLogProb #TODO: decompose this by aircraft?

  states = WorldModel.getAll(wm)

  for i = 1:sim.params.num_aircraft

    #intended command
    command = EncounterDBN.get(aem,i)
    notifyObserver(sim,"Command",Any[i, sim.t_index, command])

    output = Sensor.step(sr[i], states)
    notifyObserver(sim,"Sensor",Any[i, sim.t_index, sr[i]])

    RA = CollisionAvoidanceSystem.step(cas[i], output)
    notifyObserver(sim,"CAS", Any[i, sim.t_index, cas[i]])

    response = PilotResponse.step(pr[i], command, RA)
    logProb += response.logProb
    notifyObserver(sim,"Response",Any[i, sim.t_index, pr[i]])

    state = DynamicModel.step(adm[i], response)
    WorldModel.step(wm, i, state)
    notifyObserver(sim,"Dynamics",Any[i, sim.t_index, adm[i]])

  end

  WorldModel.updateAll(wm)
  notifyObserver(sim,"WorldModel", Any[sim.t_index, wm])

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

function isNMAC(sim::ACASX_EvE)
  vhdist = getvhdist(sim.wm)
  nmac_test = map((vhd)->vhd[2] <= sim.params.nmac_r && vhd[1] <= sim.params.nmac_h,vhdist)
  return any(nmac_test)
end

isTerminal(sim::ACASX_EvE) = sim.t_index > sim.params.maxSteps

function isEndState(sim::ACASX_EvE)
  return isNMAC(sim) || isTerminal(sim)
end

end #module

