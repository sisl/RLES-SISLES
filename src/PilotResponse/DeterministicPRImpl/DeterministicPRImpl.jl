# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module DeterministicPRImpl

export
    initialize,
    step,

    updatePilotResponse,

    DeterministicPR,
    DeterministicPRRA,
    DeterministicPRCommand


using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractPilotResponseInterfaces.updatePilotResponse


type DeterministicPRCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64

    logProb::Float64 #log probability of generating this command
end

type DeterministicPRRA

  alarm::Bool #alarm flags changes in RA
  ra_active::Bool #true if RA is on
  target_rate::Float64
  dh_min::Float64
  dh_max::Float64
end

type DeterministicPR <: AbstractPilotResponse

  follow::Bool
  target_resp_time::Int64
  timer::Int64
  currentRA::Union(DeterministicPRRA,Nothing)
  queuedRA::Union(DeterministicPRRA,Nothing)
  output::DeterministicPRCommand

  function DeterministicPR(target_resp_time::Int64)

    obj = new()
    obj.follow = false
    obj.target_resp_time = target_resp_time
    obj.timer = -1 #use -1 as timer off state
    obj.currentRA = nothing
    obj.queuedRA = nothing
    obj.output = DeterministicPRCommand(0.0, 0.0, 0.0, 0.0, 0.0)

    return obj
  end
end

function updatePilotResponse(pr::DeterministicPR, update::DeterministicPRCommand, RA::DeterministicPRRA)

  t, v_d, h_d, psi_d = update.t, update.v_d, update.h_d, update.psi_d

  #decrement the timer
  pr.timer = max(-1,pr.timer-1) #don't go lower than -1=off

  if RA.ra_active && (pr.queuedRA==nothing || RA.alarm) #there was a change in the RA
    pr.queuedRA = deepcopy(RA) #do this RA when timer finishes
    pr.timer = pr.target_resp_time #set the timer
  end

  # timer is activated
  if pr.timer == 0
    pr.follow = true
    pr.currentRA = pr.queuedRA
  end

  # coc, then reset
  if !RA.ra_active
    pr.follow = false
    pr.currentRA = nothing
    pr.queuedRA = nothing
    pr.timer = -1
  end

  if pr.follow
    h_d = pr.currentRA.target_rate
  end

  pr.output.t = t
  pr.output.v_d = v_d
  pr.output.h_d = h_d
  pr.output.psi_d = psi_d
  pr.output.logProb = 0.0 #probability 1

  return pr.output
end

step(pr::DeterministicPR, update, RA) = step(pr,convert(DeterministicPRCommand, update), convert(DeterministicPRRA,RA))

step(pr::DeterministicPR, update::DeterministicPRCommand, RA::DeterministicPRRA) = updatePilotResponse(pr, update, RA)

function initialize(pr::DeterministicPR)

  pr.follow = false
  pr.currentRA = nothing
  pr.queuedRA = nothing
  pr.timer = -1

end

end


