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

    StarDBNParams,
    StarDBN

using AbstractEncounterDBNImpl
using AbstractEncounterDBNInterfaces
using CommonInterfaces
using ObserverImpl
using Util
using Encounter
import CorrAEMImpl: CorrAEMParameters, CorrAEMInitialState, CorrAEMCommand

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractEncounterDBNInterfaces.get
import AbstractEncounterDBNInterfaces.getInitialState

include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_sample.jl"))
include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_load_params.jl"))

const MAP_G2L = Dict(2 => 1, 9 => 2, 11 => 3, 13 => 4, 17 => 5, 19 => 6) #global to local
const MAP_L2G = Dict(1 => 2, 2 => 9, 3 => 11, 4 => 13, 5=> 17, 6 => 19) #local to global
const MAP_VAR2IND_L = Dict(:L => 1, :v_d => 2, :h_d0 => 3, :psi_d0 => 4, :h_d1 => 5, :psi_d1 => 6) #variable names to local
const MAP_IND2VAR_L = Dict(1 => :L, 2 => :v_d, 3 => :h_d0, 4 => :psi_d0, 5 => :h_d1, 6 => :psi_d1) #local to variable names
const TEMPORAL_MAP = [3 5; 4 6] #[dynamic_variables0; dynamic_variables1]

immutable StarDBNParams

  tca::Float64 #time of closest approach in seconds
  v_min::Float64
  v_max::Float64
  vdot_min::Float64
  vdot_max::Float64
  h_min::Float64 #feet
  h_max::Float64 #feet
  hdot_min::Float64 #feet per second
  hdot_max::Float64 #feet per second
  psidot_min::Float64 #degrees
  psidot_max::Float64 #degrees
  l_min::Int64
  l_max::Int64

end

StarDBNParams(; tca::Float64 = 40.0, v_min::Float64 = 400.0, v_max::Float64 = 600.0,
              vdot_min::Float64 = -2.0, vdot_max::Float64 = 2.0, h_min::Float64 = 7000.0,
              h_max::Float64 = 8000.0, hdot_min::Float64 = -10.0, hdot_max::Float64 = 10.0,
              psidot_min::Float64 = 0.0, psidot_max::Float64 = 0.0, l_min::Int64 = 1,
              l_max::Int64 = 5) =
  StarDBNParams(tca, v_min, v_max, vdot_min, vdot_max, h_min, h_max, hdot_min, hdot_max,
                psidot_min, psidot_max, l_min, l_max)

type StarDBN <: AbstractEncounterDBN

  number_of_aircraft::Int64

  parameters::StarDBNParams

  parameter_file::ASCIIString
  aem_parameters::CorrAEMParameters

  encounter_seed::UInt64

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

  #caching and reuse
  dynamic_variables0::Vector{Int64}
  dynamic_variables1::Vector{Int64}
  parents_cache::Dict{Int64, Vector{Bool}}
  weights_cache::Dict{Tuple{Int64,Int64}, Vector{Float64}}
  cumweights_cache::Dict{Tuple{Int64,Int64}, Vector{Float64}}

  #pre-allocated output to avoid repeating reallocations
  output_commands::Vector{CorrAEMCommand}

  logProb::Float64 #log probability of output

  function StarDBN(number_of_aircraft::Int,
                   parameter_file::AbstractString,
                   encounter_seed::UInt64,
                   p::StarDBNParams = StarDBNParams())

    dbn = new()

    dbn.number_of_aircraft = number_of_aircraft

    dbn.parameters = p

    dbn.parameter_file = parameter_file
    dbn.aem_parameters = CorrAEMParameters()
    em_read(dbn.aem_parameters, dbn.parameter_file)

    dbn.encounter_seed = encounter_seed

    dbn.dirichlet_transition = bn_dirichlet_prior(dbn.aem_parameters.N_transition)

    dbn.t = 0

    #compute initial states of variables
    dbn.dynamic_variables0 = TEMPORAL_MAP[:, 1]
    dbn.dynamic_variables1 = TEMPORAL_MAP[:, 2]

    srand(encounter_seed) #There's a rand inside generateEncounter
    generateEncounter(dbn) #sets initial_states, initial_commands_d, initial_commands

    dbn.commands_d = deepcopy(dbn.initial_commands_d)
    dbn.commands = deepcopy(dbn.initial_commands)

    #precompute and cache these quantities
    dbn.parents_cache = Dict{Int64, Vector{Bool}}()
    dbn.weights_cache = Dict{(Int64, Int64), Vector{Float64}}()
    dbn.cumweights_cache = Dict{(Int64, Int64), Vector{Float64}}()

    for i = 1:length(dbn.aem_parameters.N_transition)
      dbn.parents_cache[i] = dbn.aem_parameters.G_transition[:, i]

      for j = 1:1:size(dbn.dirichlet_transition[i], 2)
        dbn.weights_cache[(i, j)] = dbn.aem_parameters.N_transition[i][:, j] + dbn.dirichlet_transition[i][:, j]
        dbn.weights_cache[(i, j)] /= sum(dbn.weights_cache[(i, j)])
        dbn.cumweights_cache[(i, j)] = cumsum(dbn.weights_cache[(i, j)])
      end
    end

    dbn.output_commands = CorrAEMCommand[ CorrAEMCommand(0.0, 0.0, 0.0, 0.0) for i = 1:number_of_aircraft ]
    dbn.logProb = 0.0

    return dbn
  end

end

convert_units(v::Vector{Float64}) = Float64[convert_units(v[i], MAP_IND2VAR_L[i]) for i = 1:endof(v)]

unconvert_units(v::Vector{Float64}) = Float64[unconvert_units(v[i], MAP_IND2VAR_L[i]) for i = 1:endof(v)]

function convert_units(x::Float64, var::Symbol)
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
  p = dbn.parameters

  #initial aircraft states - place in star pattern heading towards origin
  dbn.initial_states = Array(CorrAEMInitialState, dbn.number_of_aircraft)

  for i = 1:dbn.number_of_aircraft
    t = 0
    v = p.v_min + rand() * (p.v_max - p.v_min)
    h = p.h_min + rand() * (p.h_max - p.h_min)
    h_d = p.hdot_min + rand() * (p.hdot_max - p.hdot_min)
    psi = (i - 1) * 360.0 / dbn.number_of_aircraft #absolute approach angle to collision point
    psi = to_plusminus_180(psi) #make sure we're in [-180,180]
    x = v * p.tca * sind(psi + 180)
    y = v * p.tca * cosd(psi + 180)  #FIXME: v is horizontal airspeed, not total airspeed here

    dbn.initial_states[i] = CorrAEMInitialState(t, x, y, h, v, psi, h_d)
  end

  #initial aircraft commands
  dbn.initial_commands = Array(Vector{Float64}, dbn.number_of_aircraft) #[L,v_d,h_d,psi_d]
  dbn.initial_commands_d = Array(Vector{Int64}, dbn.number_of_aircraft) #[L,v_d,h_d,psi_d,[hd_tp1,psid_tp1]]

  for i = 1:dbn.number_of_aircraft
    h_d = dbn.initial_states[i].h_d         #defined before
    psi_d = p.psidot_min + rand() * (p.psidot_max - p.psidot_min)
    L = rand(p.l_min:p.l_max) #randint
    v_d = p.vdot_min + rand() * (p.vdot_max - p.vdot_min)

    dbn.initial_commands[i] = Float64[L, v_d, h_d, psi_d]
    initial_commands_d = discretize(dbn.aem_parameters, unconvert_units(dbn.initial_commands[i]))

    dbn.initial_commands_d[i] = [initial_commands_d, int64(zeros(dbn.dynamic_variables1))]
  end
end

addObserver(dbn::StarDBN, f::Function) = _addObserver(aem, f)
addObserver(dbn::StarDBN, tag::AbstractString, f::Function) = _addObserver(aem, tag, f)

function initialize(dbn::StarDBN)
  #reset to initial state
  for i = 1:dbn.number_of_aircraft
    copy!(dbn.commands_d[i], dbn.initial_commands_d[i])
    copy!(dbn.commands[i], dbn.initial_commands[i])
  end
  dbn.t = 0
end

getInitialState(dbn::StarDBN, index::Int) = dbn.initial_states[index]

function step(dbn::StarDBN)
  logProb = 0.0 #to accumulate over each aircraft

  for i = 1:dbn.number_of_aircraft

    logProb += step_dbn(dbn, dbn.commands_d[i], dbn.commands[i])

    dbn.output_commands[i].t = dbn.t
    dbn.output_commands[i].v_d = dbn.commands[i][MAP_VAR2IND_L[:v_d]]
    dbn.output_commands[i].h_d = dbn.commands[i][MAP_VAR2IND_L[:h_d0]]
    dbn.output_commands[i].psi_d = dbn.commands[i][MAP_VAR2IND_L[:psi_d0]]

  end

  dbn.t += 1

  return logProb
end

function step_dbn(dbn::StarDBN, command_d::Vector{Int64}, command::Vector{Float64})
  #_G is global (to the file), _L is local (to this module)

  p = dbn.aem_parameters
  logProb = 0.0

  for (o,i_L) in enumerate(dbn.dynamic_variables1)

    i_G = MAP_L2G[i_L]
    j_G = 1
    if !isempty(find(dbn.parents_cache[i_G]))
      parents_L = map(i -> MAP_G2L[i], find(dbn.parents_cache[i_G]))
      j_G = sub2ind(p.r_transition[dbn.parents_cache[i_G]], command_d[parents_L])
    end

    command_d[i_L] = select_random_cumweights(dbn.cumweights_cache[(i_G, j_G)])
    logProb += log(dbn.weights_cache[(i_G, j_G)][command_d[i_L]]) #prob for picking bin

    #Resampling and dediscretizing process
    i0_L = dbn.dynamic_variables0[o]
    i0_G = MAP_L2G[i0_L]

    if command_d[i_L] != command_d[i0_L] #Different bin than previous, resample with prob 1

      command[i0_L], prob = dediscretize(command_d[i_L], p.boundaries[i0_G], p.zero_bins[i0_G])
      command[i0_L] = convert_units(command[i0_L], MAP_IND2VAR_L[i0_L])
      logProb += log(prob)

    elseif rand() < p.resample_rates[i0_G] #command_d[i_L] == command_d[i0_L] same bin and meets rate

      logProb += log(p.resample_rates[i0_G]) #prob of meeting resample rate

      command[i0_L], prob = dediscretize(command_d[i_L], p.boundaries[i0_G], p.zero_bins[i0_G])
      command[i0_L] = convert_units(command[i0_L], MAP_IND2VAR_L[i0_L])
      logProb += log(prob)

    else #command_d[i_L] == command_d[i0_L] same bin, but does not meet rate

      logProb += log(1.0 - p.resample_rates[i0_G]) #prob of not meeting resample rate

      #don't resample state
    end
  end

  # update x(t) with x(t+1)
  command_d[dbn.dynamic_variables0] = command_d[dbn.dynamic_variables1]

  return logProb
end

get(dbn::StarDBN, aircraft_number::Int) = dbn.output_commands[aircraft_number]

function val2ind(boundariesi, ri, value)
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

function discretize(p::CorrAEMParameters, v::Vector{Float64})
  Int64[val2ind(p.boundaries[MAP_L2G[i]], p.r_transition[MAP_L2G[i]], val) for (i, val) in enumerate(v)]
end

function dediscretize(dval::Int64, boundaries::Vector{Float64}, zero_bin::Int64)
  val_min = boundaries[dval]
  val_max = boundaries[dval + 1]

  if dval == zero_bin
    val = 0.0
    prob = 1.0
  elseif val_max == val_min
    val = val_min
    prob = 1.0
  else
    val = val_min +  rand() * (val_max - val_min)
    prob = 1.0 / (val_max - val_min)
    #this is a density so it won't be normalized to [0,1]
  end

  return (val, prob)
end

function select_random_cumweights(cweights::Vector{Float64})
  r = cweights[end] * rand()

  return findfirst(x -> (x >= r), cweights)
end

#mods x to the range [-b, b]
function to_plusminus_b(x::AbstractFloat, b::AbstractFloat)
  z = mod(x, 2 * b)

  return (z > b) ? (z - 2 * b) : z
end

to_plusminus_180(x::AbstractFloat) = to_plusminus_b(x, 180.0)

end #module


