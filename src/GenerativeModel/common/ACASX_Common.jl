module ACASX_Common

using ObserverImpl

using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator

addObserver(sim, f::Function) = _addObserver(sim, f)
addObserver(sim, tag::String, f::Function) = _addObserver(sim, tag, f)

function initialize(sim)

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

  notifyObserver(sim,"CAS_info", Any[sim.cas[1]])

  # reset miss distances
  sim.vmd = typemax(Float64)
  sim.hmd = typemax(Float64)
  sim.md = typemax(Float64)

  #reset the probability
  sim.step_logProb = 0.0

  return
end

function step(sim)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  sim.t_index += 1

  sim.step_logProb = 0.0 #track the probabilities in this update

  cmdLogProb = EncounterDBN.step(aem)
  sim.step_logProb += cmdLogProb #TODO: decompose this by aircraft?

  states = WorldModel.getAll(wm)

  for i = 1:sim.params.num_aircraft

    #intended command
    command = EncounterDBN.get(aem, i)
    notifyObserver(sim, "Command", Any[i, sim.t_index, command])

    output = Sensor.step(sr[i], states)
    notifyObserver(sim, "Sensor",Any[i, sim.t_index, sr[i]])

    RA = CollisionAvoidanceSystem.step(cas[i], output)
    notifyObserver(sim, "CAS", Any[i, sim.t_index, cas[i]])

    response = PilotResponse.step(pr[i], command, RA)
    sim.step_logProb += response.logProb
    notifyObserver(sim, "Response", Any[i, sim.t_index, pr[i]])

    state = DynamicModel.step(adm[i], response)
    WorldModel.step(wm, i, state)
    notifyObserver(sim, "Dynamics", Any[i, sim.t_index, adm[i]])

  end

  WorldModel.updateAll(wm)
  notifyObserver(sim,"WorldModel", Any[sim.t_index, wm])

  #check and update miss distances
  vhdist = getvhdist(sim.wm)
  mds = getMissDistance(sim.params.nmac_h, sim.params.nmac_r, vhdist)
  md, index = findmin(mds)

  if md < sim.md
    sim.vmd = vhdist[index][1]
    sim.hmd = vhdist[index][2]
    sim.md = md
  end

  return
end

function getvhdist(wm::AbstractWorldModel)

  states = WorldModel.getAll(wm) #states::Vector{ASWMState}

  #[(vdist,hdist)]
  vhdist = [(abs(s2.h - s1.h),norm([(s2.x - s1.x),(s2.y - s1.y)])) for s1 = states, s2 = states]
  for i = 1 : length(states)
    vhdist[i, i] = (typemax(Float64), typemax(Float64))
  end

  return vhdist
end

function isNMAC(sim)

  vhdist = getvhdist(sim.wm)
  nmac_test = map((vhd) -> vhd[2] <= sim.params.nmac_r && vhd[1] <= sim.params.nmac_h, vhdist)
  return any(nmac_test)
end

isTerminal(sim) = sim.t_index > sim.params.max_steps

isEndState(sim) = isNMAC(sim) || isTerminal(sim)

getMissDistance(nmac_h::Float64, nmac_r::Float64, vhmd) = map((vh) -> max(vh[2] * (nmac_h / nmac_r), vh[1]), vhmd)

end
