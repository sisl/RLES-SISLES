# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 02/10/2014

module ACASX_Multi_Impl

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

export ACASX_Multi_params, ACASX_Multi, initialize, step, isEndState, addObserver

type ACASX_Multi_params
  #global params: remains constant per sim
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  max_steps::Int64 #maximum number of steps in sim
  num_aircraft::Int64 #number of aircraft  #must be 2 for now...
  encounter_seed::Uint64 #Seed for generating encounters
  encounterModel::Symbol #{:PairwiseCorrAEMDBN, :StarDBN}
  pilotResponseModel::Symbol #{:SimplePR, :StochasticLinear, :DetVsNone}
  end_on_nmac::Bool #end scenario on nmac

  #these are to define AEM:
  encounter_file::String #Path to encounter file

  #ACASX config
  quant::Int64 #quantization. Typ. quant=25
  libcas::String #Path to libcas library
  libcas_config::String #Path to libcas config file

  ACASX_Multi_params() = new()
end

type ACASX_Multi <: AbstractGenerativeModel
  params::ACASX_Multi_params

  #sim objects: contains state that changes throughout sim run
  em::Union(StarDBN,PairwiseCorrAEMDBN)
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

  vmd::Float64 #minimum vertical distance so far
  hmd::Float64 #minimum horizontal distance so far
  md::Float64 #combined miss distance metric
  md_time::Int64 #time index at which hmd and vmd are taken

  step_logProb::Float64 #cumulative probability of this step()

  #empty constructor
  function ACASX_Multi(p::ACASX_Multi_params)
    sim = new()
    sim.params = p

    if p.encounterModel == :PairwiseCorrAEMDBN
      sim.em = PairwiseCorrAEMDBN(p.num_aircraft, p.encounter_file, p.encounter_seed)
    elseif p.encounterModel == :StarDBN
      sim.em = StarDBN(p.num_aircraft, p.encounter_file, p.encounter_seed)
    else
      error("ACASX_Multi_Impl: No such encounter model")
    end

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.num_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.num_aircraft ]
    elseif p.pilotResponseModel == :FiveVsNone
      sim.pr = LLDetPR[ i==1 ? LLDetPR(5,3) : LLDetPR(-1,-1) for i=1:p.num_aircraft]
    elseif p.pilotResponseModel == :ICAO_all
      sim.pr = LLDetPR[ LLDetPR(5,3) for i=1:p.num_aircraft]
    else
      error("ACASX_Multi_Impl: No such pilot response model")
    end

    sim.dm = LLADM[ LLADM() for i=1:p.num_aircraft ]
    sim.wm = AirSpace(p.num_aircraft)

    sim.coord = GenericCoord(p.num_aircraft)

    max_intruders = p.num_aircraft-1
    sim.sr = ACASXSensor[ ACASXSensor(i,max_intruders) for i=1:p.num_aircraft ]
    sim.cas = ACASX[ ACASX(i, p.libcas, p.libcas_config, p.quant, p.num_aircraft, sim.coord)
                    for i=1:p.num_aircraft ]

    sim.vmd = typemax(Float64)
    sim.hmd = typemax(Float64)
    sim.md = typemax(Float64)
    sim.md_time = 0

    sim.step_logProb = 0.0

    sim.observer = Observer()
    sim.string_id = "ACASX_Multi_$(p.encounter_seed)"

    #Start time at 1 for easier indexing into arrays according to time
    sim.t_index = 1

    return sim
  end
end

import ACASX_Common

addObserver(sim::ACASX_Multi, f::Function) = ACASX_Common.addObserver(sim, f::Function)
addObserver(sim::ACASX_Multi, tag::String, f::Function) = ACASX_Common.addObserver(sim, tag::String, f::Function)

initialize(sim::ACASX_Multi) = ACASX_Common.initialize(sim)

step(sim::ACASX_Multi) = ACASX_Common.step(sim)

isEndState(sim::ACASX_Multi) = ACASX_Common.isEndState(sim)

end #module

