# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014

module SimpleTCAS_EvU_Impl

using AbstractGenerativeModelImpl
using AbstractGenerativeModelInterfaces
using CommonInterfaces

using Base.Test
using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CollisionAvoidanceSystem
using Simulator

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractGenerativeModelInterfaces.get
import AbstractGenerativeModelInterfaces.isEndState

export SimpleTCAS_EvU_params, SimpleTCAS_EvU, initialize, step, get, isEndState

type SimpleTCAS_EvU_params
  #global params: remains constant per sim
  encounter_number::Int64 #encounter number in file
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  maxSteps::Int64 #maximum number of steps in sim
  number_of_aircraft::Int64 #number of aircraft  #must be 2 for now...
  encounter_seed::Uint64 #Seed for generating encounters
  pilotResponseModel::Symbol #{:SimplePR, :StochasticLinear}

  #Defines behavior of CorrAEMDBN.  Read from file or generate samples on-the-fly
  command_method::Symbol #:DBN=sampled from DBN or :ENC=from encounter file

  #these are to define CorrAEM:
  encounter_file::String #Path to encounter file
  initial_sample_file::String #Path to initial sample file
  transition_sample_file::String #Path to transition sample file

  SimpleTCAS_EvU_params() = new()
end

type SimpleTCAS_EvU <: AbstractGenerativeModel
  params::SimpleTCAS_EvU_params

  #sim objects: contains state that changes throughout sim run
  em::CorrAEMDBN
  pr::Vector{Union(SimplePilotResponse,StochasticLinearPR)}
  dm::Vector{SimpleADM}
  wm::AirSpace
  sr::Vector{Union(SimpleTCASSensor,Nothing)}
  cas::Vector{Union(SimpleTCAS,Nothing)}

  #sim states: changes throughout simulation run
  t_index::Int64 #current time index in the simulation. Starts at 1 and increments by 1.
  #This is different from t which starts at 0 and could increment in the reals.

  #empty constructor
  function SimpleTCAS_EvU(p::SimpleTCAS_EvU_params)
    @test p.number_of_aircraft == 2 #need to revisit the code if this is not true

    sim = new()
    sim.params = p

    srand(p.encounter_seed) #There's a rand inside generateEncounter, need to control it
    sim.em = CorrAEMDBN(p.number_of_aircraft, p.encounter_file, p.initial_sample_file,
                    p.transition_sample_file,
                    p.encounter_number,p.command_method)

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.number_of_aircraft ]
    else
      error("SimpleTCAS_EvU_Impl: No such pilot response model")
    end

    sim.dm = SimpleADM[ SimpleADM(number_of_substeps=1) for i=1:p.number_of_aircraft ]
    sim.wm = AirSpace(p.number_of_aircraft)

    sim.sr = Union(SimpleTCASSensor,Nothing)[ SimpleTCASSensor(1), nothing ]
    sim.cas = Union(SimpleTCAS,Nothing)[ SimpleTCAS(), nothing ]

    #Start time at 1 for easier indexing into arrays according to time
    sim.t_index = 1

    return sim
  end
end

function initialize(sim::SimpleTCAS_EvU)

  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  #Start time at 1 for easier indexing into arrays according to time
  sim.t_index = 1

  EncounterDBN.initialize(aem)

  for i = 1:sim.params.number_of_aircraft
    initial = EncounterDBN.getInitialState(aem, i)
    state = DynamicModel.initialize(adm[i], initial)
    WorldModel.initialize(wm, i, state)

    # If aircraft has a CAS
    if sr[i] != nothing && cas[i] != nothing
      Sensor.initialize(sr[i])
      CollisionAvoidanceSystem.initialize(cas[i])
    end

    PilotResponse.initialize(pr[i])
  end

  return
end

function step(sim::SimpleTCAS_EvU)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  sim.t_index += 1

  logProb = 0.0 #track the probabilities in this update

  cmdLogProb = EncounterDBN.step(aem)
  logProb += cmdLogProb #TODO: decompose this by aircraft?

  states = WorldModel.getAll(wm)

  for i = 1:sim.params.number_of_aircraft
    #intended command
    command = EncounterDBN.get(aem,i)

    #If aircraft is equipped with a CAS
    if sr[i] != nothing && cas[i] != nothing
      output = Sensor.step(sr[i], states)
      RA = CollisionAvoidanceSystem.step(cas[i], output)
    else
      RA = nothing
    end

    response = PilotResponse.step(pr[i], command, RA)
    logProb += response.logProb #this will break if response is not SimplePRCommand

    state = DynamicModel.step(adm[i], response)
    WorldModel.step(wm, i, state)

  end

  WorldModel.updateAll(wm)

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

function isNMAC(sim::SimpleTCAS_EvU)
  vdist,hdist = getvhdist(sim.wm)
  return  hdist <= sim.params.nmac_r && vdist <= sim.params.nmac_h
end

isTerminal(sim::SimpleTCAS_EvU) = sim.t_index > sim.params.maxSteps

isEndState(sim::SimpleTCAS_EvU) = isNMAC(sim) || isTerminal(sim)

end #module



