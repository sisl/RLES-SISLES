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

export ACASX_EvE_params, ACASX_EvE, initialize, step, isEndState, addObserver

type ACASX_EvE_params
  #global params: remains constant per sim
  encounter_number::Int64 #encounter number in file
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  max_steps::Int64 #maximum number of steps in sim
  num_aircraft::Int64 #number of aircraft  #must be 2 for now...
  encounter_seed::Uint64 #Seed for generating encounters
  pilotResponseModel::Symbol #{:SimplePR, :StochasticLinear, :DetVsNone}
  end_on_nmac::Bool #end scenario on nmac

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

  vmd::Float64 #minimum vertical distance so far
  hmd::Float64 #minimum horizontal distance so far
  md::Float64 #combined miss distance metric
  md_time::Int64 #time index at which vmd and hmd are taken
  label_as_nmac::Bool #label this trajectory as nmac (different from NMAC_occurred that only looks at md,
                      #this one also needs to pass label filters)

  step_logProb::Float64 #cumulative probability of this step()

  #empty constructor
  function ACASX_EvE(p::ACASX_EvE_params)
    @test p.num_aircraft == 2 #need to revisit the code if this is not true

    sim = new()
    sim.params = p

    sim.em = CorrAEMDBN(p.num_aircraft, p.encounter_file, p.initial_sample_file,
                    p.transition_sample_file,
                    p.encounter_number, p.encounter_seed, p.command_method)

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i = 1 : p.num_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i = 1 : p.num_aircraft ]
    elseif p.pilotResponseModel == :FiveVsNone
      sim.pr = LLDetPR[ i == 1 ? LLDetPR(5, 3) : LLDetPR(-1, -1) for i = 1 : p.num_aircraft]
    elseif p.pilotResponseModel == :ICAO_all
      sim.pr = LLDetPR[ LLDetPR(5, 3) for i = 1 : p.num_aircraft]
    else
      error("ACASX_EvE_Impl: No such pilot response model")
    end

    sim.dm = LLADM[ LLADM() for i = 1 : p.num_aircraft]

    sim.wm = AirSpace(p.num_aircraft)
    sim.coord = GenericCoord(p.num_aircraft)

    max_intruders = p.num_aircraft - 1
    sim.sr = ACASXSensor[ ACASXSensor(i, max_intruders) for i = 1 : p.num_aircraft ]
    sim.cas = ACASX[ ACASX(i, p.libcas, p.libcas_config, p.quant, p.num_aircraft, sim.coord)
                    for i = 1 : p.num_aircraft]

    sim.vmd = typemax(Float64)
    sim.hmd = typemax(Float64)
    sim.md = typemax(Float64)
    sim.md_time = 0
    label_as_nmac = false

    sim.step_logProb = 0.0

    sim.observer = Observer()
    sim.string_id = "ACASX_EvE_$(p.encounter_number)"

    sim.t_index = 0

    return sim
  end
end

#TODO: combine some more.  Perhaps use an abstract type for all ACASX GMs.

import ACASX_Common

addObserver(sim::ACASX_EvE, f::Function) = ACASX_Common.addObserver(sim, f::Function)
addObserver(sim::ACASX_EvE, tag::String, f::Function) = ACASX_Common.addObserver(sim, tag::String, f::Function)

initialize(sim::ACASX_EvE) = ACASX_Common.initialize(sim)

step(sim::ACASX_EvE) = ACASX_Common.step(sim)

isEndState(sim::ACASX_EvE) = ACASX_Common.isEndState(sim)

end #module

