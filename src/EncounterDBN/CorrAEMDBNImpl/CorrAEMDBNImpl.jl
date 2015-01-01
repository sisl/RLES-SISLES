# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 12/15/2014


# Correlated Encounter Model for Cooperative Aircraft in the National Airspace
# exposed as DBN.  Samples are generated at runtime at each step.

module CorrAEMDBNImpl

export
    AddObserver,

    getInitialState,
    initialize,
    step,
    get,

    CorrAEMDBN

using AbstractEncounterDBNImpl
using AbstractEncounterDBNInterfaces
using CommonInterfaces
using ObserverImpl
using Util
using Encounter
using CorrAEMImpl
using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractEncounterDBNInterfaces.get
import CorrAEMImpl.getInitialState

include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_sample.jl"))

type CorrAEMDBN <: AbstractEncounterDBN

  number_of_aircraft::Int64

  encounter_file::String
  initial_sample_file::String
  transition_sample_file::String

  encounter_number::Int64
  command_method::Symbol #:ENC = read from encounter file, :DBN = generate from DBN on-the-fly

  aem::CorrAEM
  dirichlet_transition

  #initial states
  init_aem_dstate::Array{Int64,1} #discrete current state
  init_aem_dyn_cstate::Array{Float64,1} #continuous of current dynamic variables

  #current states
  t::Int64
  aem_dstate::Vector{Int64} #discrete current state
  aem_dyn_cstate::Vector{Float64} #continuous of current dynamic variables

  #pre-allocated output to avoid repeating reallocations
  output_commands::Vector{CorrAEMCommand}

  logProb::Float64 #log probability of output

  function CorrAEMDBN(number_of_aircraft::Int,encounter_file::String,initial_sample_file::String,
                      transition_sample_file::String,
                      encounter_number::Int,command_method::Symbol)
    dbn = new()

    @test number_of_aircraft == 2 #need to revisit the code if this is not true
    dbn.number_of_aircraft     = number_of_aircraft

    dbn.encounter_file         = encounter_file
    dbn.initial_sample_file    = initial_sample_file
    dbn.transition_sample_file = transition_sample_file
    dbn.encounter_number       = encounter_number
    dbn.command_method         = command_method

    dbn.aem = CorrAEM(encounter_file,initial_sample_file,transition_sample_file)

    dbn.t = 0

    generateEncounter(dbn.aem,sample_number=encounter_number) #To optimize: This allocates a lot of memory

    #compute initial states of variables
    dynamic_variables0 = dbn.aem.parameters.temporal_map[:,1]
    aem_initial_unconverted = unconvertUnitsAemState(dbn.aem.initial)
    aem_initial_dstate = Int64[ val2ind(dbn.aem.parameters.boundaries[i],
                                        dbn.aem.parameters.r_transition[i],val)
                               for (i,val) in enumerate(aem_initial_unconverted)]
    dbn.init_aem_dstate = [aem_initial_dstate,aem_initial_dstate[dynamic_variables0]] #bins, [11:14] are updated with time, append space for t+1 variables
    dbn.init_aem_dyn_cstate = dbn.aem.initial[dynamic_variables0] #continuous variables.
    dbn.dirichlet_transition = bn_dirichlet_prior(dbn.aem.parameters.N_transition)

    dbn.output_commands = CorrAEMCommand[ CorrAEMCommand(0.,0.,0.,0.) for i = 1:number_of_aircraft ]
    dbn.logProb = 0.0

    return dbn
  end

end

addObserver(dbn::CorrAEMDBN, f::Function) = _addObserver(aem, f)
addObserver(dbn::CorrAEMDBN, tag::String, f::Function) = _addObserver(aem, tag, f)

function initialize(dbn::CorrAEMDBN)
  #reset to initial state
  dbn.aem_dstate = deepcopy(dbn.init_aem_dstate)
  dbn.aem_dyn_cstate = deepcopy(dbn.init_aem_dyn_cstate)
  dbn.t = 0
end

function getInitialState(dbn::CorrAEMDBN, index::Int)
  return getInitialState(dbn.aem,index)
end

function step(dbn::CorrAEMDBN)
  if dbn.command_method == :DBN
    logProb = dbn_step(dbn)
  elseif dbn.command_method == :ENC
    logProb = enc_step(dbn)
  else
    error("CorrAEMDBNImpl::Step: No such command method")
  end
  dbn.t += 1

  return logProb
end

function dbn_step(dbn::CorrAEMDBN)
  aem = dbn.aem
  p = aem.parameters
  aem_dstate = dbn.aem_dstate #entire state, discrete bins
  aem_dyn_cstate = dbn.aem_dyn_cstate #dynamic states, continuous

  logProb = 0.0

  dynamic_variables0 = p.temporal_map[:,1] #[hdot_1(t), hdot_2(t), psidot_1(t), psidot_2(t)]
  dynamic_variables1 = p.temporal_map[:,2] #[hdot_1(t+1), hdot_2(t+1), psidot_1(t+1), psidot_2(t+1)]

  for (o,i) in enumerate(dynamic_variables1)
    parents = p.G_transition[:, i]
    if !isempty(find(parents))
      j = asub2ind(p.r_transition[parents], aem_dstate[parents]')
      weights = p.N_transition[i][:, j] + dbn.dirichlet_transition[i][:, j]
      weights /= sum(weights)
      aem_dstate[i] = select_random(weights)
      logProb += log(weights[aem_dstate[i]])
      #Resampling and dediscretizing process
      i_t = dynamic_variables0[o]
      if (aem_dstate[i] != aem_dstate[i_t]) || #compare to state at last time step, #Different bin, do resample
        (aem_dstate[i] == aem_dstate[i_t] && rand() < p.resample_rates[i_t]) #Same bin but meets resample rate
        aem_dyn_cstate[o] = dediscretize(aem_dstate[i],p.boundaries[i_t],p.zero_bins[i_t])
        if in(i,[17,18]) #these need unit conversion
          aem_dyn_cstate[o] /= 60 #convert units
        end
      end
      #Else same bin and does not meet rate, just set equal to previous (no update)
    end
  end

  # update x(t) with x(t+1)
  aem_dstate[dynamic_variables0] = aem_dstate[dynamic_variables1]

  #push to sim
  dbn.aem_dstate = aem_dstate
  dbn.aem_dyn_cstate = aem_dyn_cstate

  #Just a reminder, this will break if number_of_aircraft != 2
  @test dbn.number_of_aircraft == 2

  dbn.output_commands[1].t = dbn.t
  dbn.output_commands[1].v_d = aem.initial[9]
  dbn.output_commands[1].h_d = dbn.aem_dyn_cstate[1]
  dbn.output_commands[1].psi_d = dbn.aem_dyn_cstate[3]

  dbn.output_commands[2].t = dbn.t
  dbn.output_commands[2].v_d = aem.initial[10]
  dbn.output_commands[2].h_d = dbn.aem_dyn_cstate[2]
  dbn.output_commands[2].psi_d = dbn.aem_dyn_cstate[4]

  #return
  dbn.logProb = logProb
end

import Base.convert
convert(::Type{Vector{Float64}},command_1::CorrAEMCommand,command_2::CorrAEMCommand) =
  [ command_1.h_d, command_2.h_d, command_1.psi_d, command_2.psi_d ]

function enc_step(dbn::CorrAEMDBN)
  aem = dbn.aem
  p = aem.parameters

  #Just a reminder, this will break if number_of_aircraft != 2
  @test dbn.number_of_aircraft == 2

  command_1 = Encounter.step(aem,1)
  command_2 = Encounter.step(aem,2)

  dynamic_variables0 = p.temporal_map[:,1] #[hdot_1(t), hdot_2(t), psidot_1(t), psidot_2(t)]
  dynamic_variables1 = p.temporal_map[:,2] #[hdot_1(t+1), hdot_2(t+1), psidot_1(t+1), psidot_2(t+1)]

  #prepare t+1 from encounter commands
  aem_dyn_cstate = convert(Vector{Float64},command_1,command_2)
  aem_dstate = dbn.aem_dstate #only copies pointer
  #load into the (t+1) slots
  #boundaries are specified at t though
  aem_dyn_cstate_unconverted = unconvertUnitsDynVars(aem_dyn_cstate) #need to unconvert units
  aem_dstate[dynamic_variables1] = Int64[ val2ind(aem.parameters.boundaries[i],
                                                 aem.parameters.r_transition[i],
                                                 aem_dyn_cstate_unconverted[o])
                                        for (o,i) in enumerate(dynamic_variables0) ]

  # compute the probability of this transition
  logProb = 0.0
  for (o,i) in enumerate(dynamic_variables1)
    parents = p.G_transition[:, i]
    if !isempty(find(parents))
      j = asub2ind(p.r_transition[parents], aem_dstate[parents]')
      weights = p.N_transition[i][:, j] + dbn.dirichlet_transition[i][:, j]
      weights /= sum(weights)
      logProb += log(weights[aem_dstate[i]]) #probability from discrete sampling process

      #probability from continuous sampling process
      i_t = dynamic_variables0[o]
      if aem_dstate[i] == aem_dstate[i_t] != p.zero_bins[i_t] #same bin but not the zero bin
        if isapprox(aem_dyn_cstate[o],dbn.aem_dyn_cstate[o],atol=0.0001)
          #Same bin and no resample
          logProb += log(1.0-p.resample_rates[i_t])
        else
          #Same bin and got resampled
          logProb += log(p.resample_rates[i_t])
        end
      end
    end
  end

  # update x(t) with x(t+1)
  aem_dstate[dynamic_variables0] = aem_dstate[dynamic_variables1]

  #push to sim
  dbn.aem_dstate = aem_dstate
  dbn.aem_dyn_cstate = aem_dyn_cstate

  #Just a reminder, this will break if number_of_aircraft != 2
  @test dbn.number_of_aircraft == 2

  dbn.output_commands[1].t = dbn.t
  dbn.output_commands[1].v_d = aem.v_init[1]
  dbn.output_commands[1].h_d = dbn.aem_dyn_cstate[1]
  dbn.output_commands[1].psi_d = dbn.aem_dyn_cstate[3]

  dbn.output_commands[2].t = dbn.t
  dbn.output_commands[2].v_d = aem.v_init[2]
  dbn.output_commands[2].h_d = dbn.aem_dyn_cstate[2]
  dbn.output_commands[2].psi_d = dbn.aem_dyn_cstate[4]

  #return
  dbn.logProb = logProb
end

function get(dbn::CorrAEMDBN, aircraft_number::Int)
  return dbn.output_commands[aircraft_number]
end

function val2ind(boundariesi,ri,value)
  if !isempty(boundariesi)
    index = findfirst(x -> (x > value), boundariesi) - 1

    if index == -1
      index = ri
    end
  else
    index = value
  end
  return index
end

function dediscretize(dstate::Int64,boundaries::Vector{Float64},zero_bin::Int64)
  val_min = boundaries[dstate]
  val_max = boundaries[dstate+1]

  return dstate == zero_bin ? 0.0 : val_min + (val_max - val_min) * rand()
end

function unconvertUnitsDynVars(v)
  return [v[1:2]*60.0,v[3:end]]
end

function unconvertUnitsAemState(state_)
  state = deepcopy(state_)

  state[7] /= 1.68780
  state[8] /= 1.68780
  state[9] /= 1.68780
  state[10] /= 1.68780
  state[11] *= 60
  state[12] *= 60
  state[15] /= 6076.12

  return state
end

function asub2ind(siz, x)
# ASUB2IND Linear index from multiple subscripts.
#   Returns a linder index from multiple subscripts assuming a matrix of a
#   specified size.
#
#   NDX = ASUB2IND(SIZ,X) returns the linear index NDX of the element in a
#   matrix of dimension SIZ associated with subscripts specified in X.

    k = [1, cumprod(siz[1:end-1])]
    ndx = k' * (x' - 1) + 1

    return ndx[1]
end

#=
function fastcopy(em::CorrAEM)
  emcopy = CorrAEM()

  for sym in names(em)
    if isdefined(em,sym)
      if sym == :parameters
        #:parameters is very big.  Since it does not change once loaded, just copy the pointer
        emcopy.(sym) = em.(sym)
      else
        emcopy.(sym) = deepcopy(em.(sym)) #deepcopy everything else
      end
    end
  end
  return emcopy
end
=#

end #module


