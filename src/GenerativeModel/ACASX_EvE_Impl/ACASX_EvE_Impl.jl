# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014


module ACASX_EvE_Impl

using Base.Test
using SISLES
using Encounter
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator

SISLES_PATH = Pkg.dir("SISLES", "src")
include("$SISLES_PATH/Encounter/CorrAEMImpl/corr_aem_sample.jl")

using RNGTools
using MDP

import Base.convert

type EncSimStates
  #sim states: changes throughout simulation run
  actionCounter::Uint32 #global for new random seed
  aem_dstate::Array{Int64,1} #discrete current state
  aem_dyn_cstate::Array{Float64,1} #continuous of current dynamic variables
  t::Int64 #current time in the simulation
  vmd::Float64 #minimum vertical distance so far
  hmd::Float64 #minimum horizontal distance so far
  md::Float64 #combined miss distance metric
  dirichlet_transition #dirichlet priors for transition dbn

  # To save reallocation
  commands::Vector{CorrAEMCommand}
end

EncSimStates() = EncSimStates(uint32(0),Int64[],Float64[],0,0.,0.,0.,0,CorrAEMCommand[])

type EncSimObjs
  #sim objects: contains state that changes throughout sim run
  em::CorrAEM   #Will break if you use another model
  pr::Vector{AbstractPilotResponse}
  dm::Vector{AbstractDynamicModel}
  wm::AbstractWorldModel
  sr::Vector{Union(AbstractSensor,Nothing)}
  cas::Vector{Union(AbstractCollisionAvoidanceSystem,Nothing)}
  coord::Union(AbstractCASCoord,Nothing)

  EncSimObjs() = new()
end

type EncSimCache
  encounterNumber::Int64
  encounterSeed::Uint64
  em::Union(AbstractEncounterModel,Nothing)
  dirichlet_transition
end

EncSimCache() = EncSimCache(0,0,nothing,nothing)

immutable EncSimParams
  #global params: remains constant per sim
  encounterNumber::Int64 #encounter number in file
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  maxsteps::Int64 #maximum number of steps in sim
  nmac_reward::Float64 #reward bonus for achieving an NMAC
  number_of_aircraft::Int64 #number of aircraft  #FIXME: This scenario will break if number_of_aircraft != 2
  command_method::String #'DBN'=sampled from DBN or 'ENC'=from encounter file
  encounterSeed::Uint64 #Seed for generating encounters
  actionSeed::Union(Nothing,Uint32) #Seed for generating actions, nothing=don't reset
end

type EncSim <: Params
  state::EncSimStates

  obj::EncSimObjs

  cache::EncSimCache

  param::EncSimParams

  #empty constructor
  function EncSim(p::EncSimParams)
    sim = new()
    sim.state = EncSimStates()
    sim.obj = EncSimObjs()
    sim.cache = EncSimCache()
    sim.param = p

    return sim
  end
end

type ESAction <: Action
  seed::Union(Uint32,Vector{Uint32})
  ESAction() = new(uint32(0))
  ESAction(s::Union(Uint32,Vector{Uint32})) = new(s)
end

type ESState <: State
  t::Int64 #sanity check that at least the time corresponds
  parent::Union(Nothing,ESState) #parent state, root=nothing
  action::ESAction #action taken from parent, root=0
end

export EncSim, EncSimObjs, EncSimParams, ESState, ESAction, getInitialState, getNextState,
isEndState, randomPolicy, sim_observer, SimLog, getPathOfActions

type SimLog <: AbstractLog
  sim::EncSim
  AC1_trajectory::Array{Float64,2} #ac1
  AC2_trajectory::Array{Float64,2} #ac2
  rai_1::Int64
  rai_2::Int64
  actions_trajectory::Array{ESAction,1} #actions taken along trajectory
  i::Int64 #current entry
  SimLog(sim::EncSim) = new(sim,Array(Float64,sim.param.maxsteps+1,4),
                            Array(Float64,sim.param.maxsteps+1,4),
                            sim.param.maxsteps+2,sim.param.maxsteps+2,
                            Array(ESAction,sim.param.maxsteps+1),1)
end

import Base.push!
function push!(log::SimLog,s::ESState)
  log.AC1_trajectory[log.i,:], log.AC2_trajectory[log.i,:],ra1,ra2 = sim_observer(log.sim,s)
  log.rai_1 = ra1 ? min(log.i,log.rai_1) : log.rai_1
  log.rai_2 = ra2 ? min(log.i,log.rai_2) : log.rai_2
  log.actions_trajectory[log.i] = s.action #the first one is meaningless for replay
  log.i += 1

  #values 1:i-1 are valid.  Avoids on-the-fly allocation.
end

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

customcopy(em::CorrAEM) = fastcopy(em)
customcopy(em::AbstractEncounterModel) = deepcopy(em)

function initialize!(sim::EncSim)
  srand(sim.param.encounterSeed) #There's a rand inside generateEncounter

  if (sim.cache.em != nothing
      && sim.cache.dirichlet_transition != nothing
      && sim.cache.encounterNumber == sim.param.encounterNumber
      && sim.cache.encounterSeed == sim.param.encounterSeed)
    # If encounter is same as in cache, just reload it

    sim.obj.em = customcopy(sim.cache.em)
    sim.state.dirichlet_transition = sim.cache.dirichlet_transition
  else
    # Generate the encounter and load it in the cache
    generateEncounter(sim.obj.em,sample_number=sim.param.encounterNumber) #FIXME: This allocates a lot of memory
    sim.cache.encounterNumber = sim.param.encounterNumber
    sim.cache.encounterSeed = sim.param.encounterSeed
    sim.cache.em = customcopy(sim.obj.em)

    sim.state.dirichlet_transition = bn_dirichlet_prior(sim.obj.em.parameters.N_transition)
    sim.cache.dirichlet_transition = sim.state.dirichlet_transition #doesn't change over run, just copy pointer
  end

  sim.state.aem_dyn_cstate = sim.obj.em.initial[11:14] #TODO: remove hardcoding.  #slice will auto make deepcopy
  if sim.param.actionSeed != nothing #reset if specified
    sim.state.actionCounter = sim.param.actionSeed
  else #otherwise, randomize
    sim.state.actionCounter = uint32(hash(time()))
  end
end

convert(::Type{StochasticLinearPRCommand}, command::Union(CorrAEMCommand, LLAEMCommand)) = StochasticLinearPRCommand(command.t, command.v_d, command.h_d, command.psi_d, 1.0)
convert(::Type{SimpleADMCommand}, command::StochasticLinearPRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)

function convert(::Type{ACASXSensorInput}, states::Vector{ASWMState})
  states_ = Array(ACASXSensorState, length(input))

  for i = 1:length(input)
    states_[i] = ACASXSensorState(states[i].x,states[i].y,states[i].h,states[i].vx,states[i].vy,states[i].vh)
  end

  return ACASXSensorInput(states_)
end

function getvhdist(wm::AbstractWorldModel,s::ESState)
  states_1,states_2 = WorldModel.getAll(wm) #states::Vector{ASWMState}
  x1, y1, h1 = states_1.x, states_1.y, states_1.h
  x2, y2, h2 = states_2.x, states_2.y, states_2.h

  vdist = abs(h2-h1)
  hdist = norm([(x2-x1),(y2-y1)])

  return vdist,hdist
end

function isNMAC(sim::EncSim,s::ESState)
  vdist,hdist = getvhdist(sim.obj.wm,s)
  return  hdist <= sim.param.nmac_r && vdist <= sim.param.nmac_h
end

function isTerminal(sim::EncSim,s::ESState)
  states_1,states_2 = WorldModel.getAll(sim.obj.wm) #states::Vector{ASWMState}
  t = states_1.t

  return t >= sim.param.maxsteps
end

isEndState(sim::EncSim,s::ESState) = isNMAC(sim,s) || isTerminal(sim,s)

function getInitialState(sim::EncSim,rng::AbstractRNG)
  initialize!(sim)
  aem = sim.obj.em
  wm, pr, adm, cas, sr, coord = sim.obj.wm, sim.obj.pr, sim.obj.dm, sim.obj.cas, sim.obj.sr, sim.obj.coord

  for i = 1:sim.param.number_of_aircraft
    initial = Encounter.getInitialState(aem, i)
    state = DynamicModel.initialize(adm[i], convert(SimpleADMInitialState, initial))
    WorldModel.initialize(wm, i, convert(ASWMState, state))

    # If aircraft has a CAS
    if sr[i] != nothing && cas[i] != nothing
      Sensor.initialize(sr[i])
      CollisionAvoidanceSystem.initialize(cas[i])
    end

    PilotResponse.initialize(pr[i])
  end

  if coord != nothing
    CASCoordination.initialize(coord)
  end

  sim.state.t = 0
  dynamic_variables0 = aem.parameters.temporal_map[:,1]
  aem_initial_unconverted = unconvertUnitsAemState(aem.initial) #FIXME: do these need to be deepcopied?
  aem_initial_dstate = Int64[ val2ind(aem.parameters.boundaries[i],aem.parameters.r_transition[i],val) for (i,val) in enumerate(aem_initial_unconverted)]
  sim.state.aem_dstate = [aem_initial_dstate,aem_initial_dstate[dynamic_variables0]] #bins, [11:14] are updated with time, append space for t+1 variables
  sim.state.aem_dyn_cstate = aem.initial[11:14] #continuous variables
  s1 = ESState(sim.state.t,nothing,ESAction())
  @test check_state(wm,s1) ? true : error("check_state in getInitialState()")

  sim.state.vmd, sim.state.hmd = getvhdist(wm,s1)
  sim.state.md = getMissDistance(sim.param.nmac_h,sim.param.nmac_r,sim.state.vmd,sim.state.hmd)

  sim.state.commands = Array(CorrAEMCommand,sim.param.number_of_aircraft)

  return s1
end

function update_dynamic_vars!(sim::EncSim)
  t = sim.state.t
  p = sim.obj.em.parameters
  G_transition = p.G_transition
  N_transition = p.N_transition
  r = p.r_transition
  temporal_map = p.temporal_map
  boundaries = p.boundaries
  zero_bins = p.zero_bins
  resample_rates = p.resample_rates
  dirichlet_transition = sim.state.dirichlet_transition
  aem_dstate = sim.state.aem_dstate
  aem_dyn_cstate = sim.state.aem_dyn_cstate

  logProb = 0.0
  dynamic_variables1 = temporal_map[:,2] #[hdot_1(t+1), hdot_2(t+1), psidot_1(t+1), psidot_2(t+1)]
  for (o,i) in enumerate(dynamic_variables1)
    parents = G_transition[:, i]
    j = 1
    if !isempty(find(parents))
      j = asub2ind(r[parents], aem_dstate[parents]')
      weights = N_transition[i][:, j] + dirichlet_transition[i][:, j]
      weights /= sum(weights)
      aem_dstate[i] = select_random(weights)
      logProb += log(weights[aem_dstate[i]])
      #Resampling and dediscretizing process
      i_t = temporal_map[o,1]
      if aem_dstate[i] != aem_dstate[i_t] #compare to state at last time step
        #Different bin, do resample
        aem_dyn_cstate[o] = dediscretize(aem_dstate[i],boundaries[i_t],zero_bins[i_t])
        if in(i,[17,18]) #these need unit conversion
          aem_dyn_cstate[o] /= 60 #convert units
        end
      elseif rand() < resample_rates[i_t]
        #Same bin and meets probabilistic rate, do resample
        aem_dyn_cstate[o] = dediscretize(aem_dstate[i],boundaries[i_t],zero_bins[i_t])
        if in(i,[17,18]) #these need unit conversion
          aem_dyn_cstate[o] /= 60 #convert units
        end
      end
      #Else same bin and does not meet rate, just set equal to previous (no update)
    end
  end

  # update x(t) with x(t+1)
  aem_dstate[temporal_map[:, 1]] = aem_dstate[temporal_map[:, 2]]

  #push to sim
  sim.state.aem_dstate = aem_dstate
  sim.state.aem_dyn_cstate = aem_dyn_cstate

  return logProb
end

function dediscretize(dstate::Int64,boundaries::Vector{Float64},zero_bin::Int64)
  val_min = boundaries[dstate]
  val_max = boundaries[dstate+1]

  return dstate == zero_bin ? 0.0 : val_min + (val_max - val_min) * rand()
end

isActivated(cas::SimpleTCAS) = cas.b_TCAS_activated
isActivated(cas::CoordSimpleTCAS) = cas.simpleCAS.b_TCAS_activated
isActivated(cas::ACASX) = cas.outputVals.alarm

function sim_observer(sim::EncSim,s::ESState)
  adm = sim.obj.dm
  cas = sim.obj.cas

  state_1, state_2 = adm[1].state, adm[2].state
  ac1 = deepcopy([state_1.t, state_1.x, state_1.y, state_1.h])
  ac2 = deepcopy([state_2.t, state_2.x, state_2.y, state_2.h])
  ra1 = cas[1] != nothing ? isActivated(cas[1]) : false
  ra2 = cas[2] != nothing ? isActivated(cas[2]) : false

  return ac1,ac2,ra1,ra2
end

function check_state(wm::AbstractWorldModel,s::ESState)
  state_1,state_2 = WorldModel.getAll(wm)

  return s.t == state_1.t == state_2.t
end

function getNextState(sim::EncSim,s0::ESState,a0::ESAction,rng::AbstractRNG)
  wm, pr, adm, cas, sr = sim.obj.wm, sim.obj.pr, sim.obj.dm, sim.obj.cas, sim.obj.sr

  @test check_state(wm,s0) ? true : error("check_state at getNextState()")

  set_gv_rng_state(a0.seed)

  logProb = 0.0 #track the probabilities in this update

  #FIXME: This will break if number_of_aircraft is not 2
  if sim.param.command_method == "DBN"
    sim.state.commands[1],sim.state.commands[2],cmdLogProb =
      updatePilotCommands!(sim) #generate from the previous pilot commands
  elseif sim.param.command_method == "ENC"
    sim.state.commands[1],sim.state.commands[2],cmdLogProb =
      updatePilotCommandsFromEncounter!(sim)
  else
    error("Error in getNextState(): No such command_method!")
  end
  logProb += cmdLogProb

  states = WorldModel.getAll(wm)

  for i = 1:sim.param.number_of_aircraft
    #If aircraft is equipped with a CAS
    if sr[i] != nothing && cas[i] != nothing
      output = Sensor.step(sr[i], convert(SimpleTCASSensorInput, states))
      RA = CollisionAvoidanceSystem.step(cas[i], convert(SimpleTCASInput, output))
    else
      RA = nothing
    end

    response = PilotResponse.step(pr[i], convert(StochasticLinearPRCommand, sim.state.commands[i]), convert(SimplePRResolutionAdvisory, RA))
    logProb += log(response.prob) #this will break if response is not SimplePRCommand
    state = DynamicModel.step(adm[i], convert(SimpleADMCommand, response))
    WorldModel.step(wm, i, convert(ASWMState, state))
  end

  WorldModel.updateAll(wm)

  sim.state.t += 1
  s1 = ESState(sim.state.t,s0,a0)

  #update miss distances
  vdist,hdist = getvhdist(wm,s1)
  md = getMissDistance(sim.param.nmac_h,sim.param.nmac_r,vdist,hdist)
  if  md < sim.state.md
    sim.state.vmd = vdist
    sim.state.hmd = hdist
    sim.state.md = md
  end

  r = getReward(sim,logProb,s1)

  return s1, r
end

getMissDistance(nmac_h,nmac_r,vmd,hmd) = max(hmd*(nmac_h/nmac_r),vmd)

function getReward(sim::EncSim,logProb::Float64,s::ESState)
  reward = logProb

  if isNMAC(sim,s)
    reward += sim.param.nmac_reward
  elseif isTerminal(sim,s)
    reward += -sim.state.md
  end

  return reward
end

function randomPolicy(sim::EncSim,s0::ESState)
  sim.state.actionCounter = uint32(sim.state.actionCounter+1) #return the next random seed

  return ESAction(sim.state.actionCounter)
end

function updatePilotCommands!(sim::EncSim)
  t = sim.state.t
  logProb = update_dynamic_vars!(sim)

  return CorrAEMCommand(t,sim.obj.em.initial[9],sim.state.aem_dyn_cstate[1],sim.state.aem_dyn_cstate[3]),
          CorrAEMCommand(t,sim.obj.em.initial[10],sim.state.aem_dyn_cstate[2],sim.state.aem_dyn_cstate[4]), logProb
end

convert(::Type{Vector{Float64}},command_1::CorrAEMCommand,command_2::CorrAEMCommand) =
  [ command_1.h_d, command_2.h_d, command_1.psi_d, command_2.psi_d ]

function unconvertUnitsAemState(state0)
  state1 = deepcopy(state0)

  state1[7] /= 1.68780
  state1[8] /= 1.68780
  state1[9] /= 1.68780
  state1[10] /= 1.68780
  state1[11] *= 60
  state1[12] *= 60
  state1[15] /= 6076.12

  return state1
end

function unconvertUnitsDynVars(v)
  return [v[1:2]*60.0,v[3:end]]
end

function updatePilotCommandsFromEncounter!(sim::EncSim)
  aem = sim.obj.em
  temporal_map = sim.obj.em.parameters.temporal_map
  command_1 = Encounter.step(aem,1)
  command_2 = Encounter.step(aem,2)

  dynamic_variables0 = temporal_map[:,1]
  aem_dyn_cstate = convert(Vector{Float64},command_1,command_2)
  aem_dstate = sim.state.aem_dstate
  #load into the (t+1) slots
  aem_dyn_cstate_unconverted = unconvertUnitsDynVars(aem_dyn_cstate) #need to unconvert units
  aem_dstate[dynamic_variables0] = Int64[ val2ind(aem.parameters.boundaries[i],
                                                 aem.parameters.r_transition[i],
                                                 aem_dyn_cstate_unconverted[o])
                                        for (o,i) in enumerate(dynamic_variables0) ]

  # compute the probability of this transition
  p = aem.parameters
  G_transition = p.G_transition
  N_transition = p.N_transition
  r = p.r_transition
  temporal_map = p.temporal_map
  boundaries = p.boundaries
  zero_bins = p.zero_bins
  resample_rates = p.resample_rates
  dirichlet_transition = sim.state.dirichlet_transition

  logProb = 0.0
  dynamic_variables1 = temporal_map[:,2] #[hdot_1(t+1), hdot_2(t+1), psidot_1(t+1), psidot_2(t+1)]
  for (o,i) in enumerate(dynamic_variables1)
    parents = G_transition[:, i]
    j = 1
    if !isempty(find(parents))
      j = asub2ind(r[parents], aem_dstate[parents]')
      weights = N_transition[i][:, j] + dirichlet_transition[i][:, j]
      weights /= sum(weights)
      logProb += log(weights[aem_dstate[i]]) #probability from discrete sampling process

      #probability from continuous sampling process
      i_t = temporal_map[o,1]
      if aem_dstate[i] == aem_dstate[i_t]
        if isapprox(aem_dyn_cstate[o],sim.state.aem_dyn_cstate[o],atol=0.0001)
          #Same bin and no resample
          logProb += log(1.0-resample_rates[i_t])
        else
          #Same bin and got resampled
          logProb += log(resample_rates[i_t])
        end
      end
    end
  end

  # update x(t) with x(t+1)
  aem_dstate[temporal_map[:, 1]] = aem_dstate[temporal_map[:, 2]]

  #push to sim
  sim.state.aem_dstate = aem_dstate
  sim.state.aem_dyn_cstate = aem_dyn_cstate

  return command_1,command_2,logProb
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

function actionsToThisState(state::ESState)
  revActions = ESAction[]
  s = state
  while s.parent != nothing
    push!(revActions,s.action)
    s = s.parent
  end

  return reverse(revActions)
end

end #module



