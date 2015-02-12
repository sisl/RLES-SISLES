# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 12/15/2014


# Simple multiple aircraft encounter model that points all aircraft toward origin
# DBN samples are generated at runtime at each step.

module StarDBNImpl

export
    AddObserver,

    getInitialState,
    initialize,
    step,
    get,

    StarDBN

using AbstractEncounterDBNImpl
using AbstractEncounterDBNInterfaces
using CommonInterfaces
using ObserverImpl
using Util
using Encounter
import CorrAEMImpl: CorrAEMParameters, CorrAEMInitialState, CorrAEMCommand
using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractEncounterDBNInterfaces.get
import AbstractEncounterDBNInterfaces.getInitialState

include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_sample.jl"))
include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_load_params.jl"))

const TCA = 40.0 #time of closest approach in seconds
const V_MIN = 400
const V_MAX = 600
const VDOT_MIN = -2.0
const VDOT_MAX = 2.0
const H_MIN = 7000
const H_MAX = 8000
const HDOT_MIN = -10.0
const HDOT_MAX = 10.0
const PSIDOT_MIN = -0.0
const PSIDOT_MAX = 0.0
const L_MIN = 1
const L_MAX = 5

type StarDBN <: AbstractEncounterDBN

  number_of_aircraft::Int64

  parameter_file::String
  parameters::CorrAEMParameters

  dirichlet_initial
  dirichlet_transition

  t::Int64

  #initial state
  initial_states::Vector{CorrAEMInitialState}

  #initial command
  initial_commands_d::Vector{Vector{Int64}} #discrete current state
  initial_commands::Vector{Vector{Float64}} #continuous of current dynamic variables

  #current command
  commands_d::Vector{Vector{Int64}} #discrete current state
  commands::Vector{Vector{Float64}} #continuous of current dynamic variables

  #pre-allocated output to avoid repeating reallocations
  output_commands::Vector{CorrAEMCommand}

  logProb::Float64 #log probability of output

  function StarDBN(number_of_aircraft::Int, parameter_file::String, encounter_seed::Uint64)

    dbn = new()

    dbn.number_of_aircraft     = number_of_aircraft

    dbn.parameter_file = parameter_file
    dbn.parameters = CorrAEMParameters()
    em_read(dbn.parameters,dbn.parameter_file)

    dbn.dirichlet_initial = bn_dirichlet_prior(dbn.parameters.N_initial)
    dbn.dirichlet_transition = bn_dirichlet_prior(dbn.parameters.N_transition)

    dbn.t = 0

    srand(encounter_seed) #There's a rand inside generateEncounter
    generateEncounter(dbn) #sets initial_states, initial_commands_d, initial_commands

    dbn.output_commands = CorrAEMCommand[ CorrAEMCommand(0.,0.,0.,0.) for i = 1:number_of_aircraft ]
    dbn.logProb = 0.0

    return dbn
  end

end

const map_G2L = [2 => 1, 9 => 2, 11 => 3, 13 => 4, 17 => 5, 19 => 6] #global to local
const map_L2G = [1 => 2, 2 => 9, 3 => 11, 4 => 13, 5=> 17, 6 => 19] #local to global
const map_var2ind_L = [:L => 1, :v_d => 2, :h_d0 => 3, :psi_d0 => 4, :h_d1 => 5, :psi_d1 => 6] #variable names to local
const map_ind2var_L = [1 => :L, 2 => :v_d, 3 => :h_d0, 4 => :psi_d0, 5 => :h_d1, 6 => :psi_d1] #local to variable names
const temporal_map = [3 5; 4 6] #[dynamic_variables0; dynamic_variables1]

convert_units(v::Vector{Float64}) = Float64[convert_units(v[i],map_ind2var_L[i]) for i=1:endof(v)]
unconvert_units(v::Vector{Float64}) = Float64[unconvert_units(v[i],map_ind2var_L[i]) for i=1:endof(v)]

function convert_units(x::Float64,var::Symbol)

  if var == :v_d0 || var == :v_d1
    return x * 1.68780
  elseif var == :h_d0 || var == :h_d1
    return x / 60
  else
    return x
  end

end

function unconvert_units(x::Float64,var::Symbol)

  if var == :v_d0 || var == :v_d1
    return x / 1.68780
  elseif var == :h_d0 || var == :h_d1
    return x * 60
  else
    return x
  end

end

function generateEncounter(dbn::StarDBN)
  #initial aircraft states - place in star pattern heading towards origin
  dbn.initial_states = Array(CorrAEMInitialState,dbn.number_of_aircraft)

  for i = 1:dbn.number_of_aircraft

    t = 0
    v = V_MIN + rand() * (V_MAX - V_MIN)
    h = H_MIN + rand() * (H_MAX - H_MIN)
    h_d = HDOT_MIN + rand() * (HDOT_MAX - HDOT_MIN)
    psi = (i-1)*360.0/dbn.number_of_aircraft #absolute approach angle to collision point
    x = v * TCA * cosd(psi+180)
    y = v * TCA * sind(psi+180)

    dbn.initial_states[i] = CorrAEMInitialState(t,x,y,h,v,psi,h_d)
  end

  #initial aircraft commands
  dbn.initial_commands = Array(Vector{Float64},dbn.number_of_aircraft) #[L,v_d,h_d,psi_d]
  dbn.initial_commands_d = Array(Vector{Int64},dbn.number_of_aircraft) #[L,v_d,h_d,psi_d,[hd_tp1,psid_tp1]]

  for i = 1:dbn.number_of_aircraft
    h_d = dbn.initial_states[i].h_d         #defined before
    psi_d = PSIDOT_MIN + rand() * (PSIDOT_MAX - PSIDOT_MIN)
    L = rand(L_MIN:L_MAX) #randint
    v_d = VDOT_MIN + rand() * (VDOT_MAX - VDOT_MIN)

    dbn.initial_commands[i] = Float64[L, v_d, h_d, psi_d]
    initial_commands_d = discretize(dbn.parameters,unconvert_units(dbn.initial_commands[i]))

    dynamic_variables1 = temporal_map[:,2]
    dbn.initial_commands_d[i] = [ initial_commands_d, int64(zeros(dynamic_variables1)) ]
  end
end

addObserver(dbn::StarDBN, f::Function) = _addObserver(aem, f)
addObserver(dbn::StarDBN, tag::String, f::Function) = _addObserver(aem, tag, f)

function initialize(dbn::StarDBN)
  #reset to initial state
  dbn.commands_d = deepcopy(dbn.initial_commands_d)
  dbn.commands = deepcopy(dbn.initial_commands)
  dbn.t = 0

end

function getInitialState(dbn::StarDBN, index::Int)
  return dbn.initial_states[index]
end

function step(dbn::StarDBN)
  logProb = 0.0 #to accumulate over each aircraft

  for i = 1:dbn.number_of_aircraft
    logProb += step_dbn(dbn.parameters,dbn.dirichlet_transition,
                        dbn.commands_d[i],dbn.commands[i])

    dbn.output_commands[i].t = dbn.t
    dbn.output_commands[i].v_d = dbn.commands[i][map_var2ind_L[:v_d]]
    dbn.output_commands[i].h_d = dbn.commands[i][map_var2ind_L[:h_d0]]
    dbn.output_commands[i].psi_d = dbn.commands[i][map_var2ind_L[:psi_d0]]
  end

  dbn.t += 1

  return logProb
end

function step_dbn(p::CorrAEMParameters, dirichlet_transition,
                         command_d::Vector{Int64}, command::Vector{Float64})
  logProb = 0.0

  dynamic_variables0 = temporal_map[:,1] #[hdot_1(t), psidot_1(t)]
  dynamic_variables1 = temporal_map[:,2] #[hdot_1(t+1), psidot_1(t+1)]

  for (o,i_L) in enumerate(dynamic_variables1)
    i_G = map_L2G[i_L]
    parents_G = p.G_transition[:, i_G]
    if !isempty(find(parents_G))
      parents_L = Int64[map_G2L[iparents] for iparents in find(parents_G)]
      j_G = asub2ind(p.r_transition[parents_G], command_d[parents_L]')
      weights = p.N_transition[i_G][:, j_G] + dirichlet_transition[i_G][:, j_G]
      weights /= sum(weights)
      command_d[i_L] = select_random(weights)
      logProb += log(weights[command_d[i_L]])
      #Resampling and dediscretizing process
      i0_L = dynamic_variables0[o]
      i0_G = map_L2G[i0_L]
      if (command_d[i_L] != command_d[i0_L]) || #compare to state at last time step, #Different bin, do resample
        (command_d[i_L] == command_d[i0_L] && rand() < p.resample_rates[i0_G]) #Same bin but meets resample rate
        command[i0_L] = dediscretize(command_d[i_L],p.boundaries[i0_G],p.zero_bins[i0_G])
        command[i0_L] = convert_units(command[i0_L],map_ind2var_L[i0_L])
      end
      #Else same bin and does not meet rate, just set equal to previous (no update)
    end
  end

  # update x(t) with x(t+1)
  command_d[dynamic_variables0] = command_d[dynamic_variables1]

  #return
  return logProb
end

function get(dbn::StarDBN, aircraft_number::Int)
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

function discretize(p::CorrAEMParameters,v::Vector{Float64})
  return Int64[ val2ind(p.boundaries[map_L2G[i]],
                        p.r_transition[map_L2G[i]],val)
               for (i,val) in enumerate(v) ]
end

function dediscretize(dval::Int64,boundaries::Vector{Float64},zero_bin::Int64)
  val_min = boundaries[dval]
  val_max = boundaries[dval+1]

  return dval == zero_bin ? 0.0 : val_min +  rand() * (val_max - val_min)
end

function asub2ind(siz, x)
# ASUB2IND Linear index from multiple subscripts.
#   Returns a linear index from multiple subscripts assuming a matrix of a
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


