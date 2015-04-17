# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 04/16/2014
# Renamed from DeterministicPR and added changes to match LL pilot response/dynamics

module LLDetPRImpl

export
initialize,
step,

updatePilotResponse,

LLDetPR,
LLDetPRCommand,
LLDetPRRA,
LLDetPROutput

using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractPilotResponseInterfaces.updatePilotResponse

using Base.Test
import Base.isequal

type LLDetPRCommand

  t::Float64
  v_d::Float64             #commanded acceleration (ft/s^2)
  h_d::Float64             #commanded vertical rate (ft/s)
  psi_d::Float64           #commanded turn rate (deg/s)

end

type LLDetPROutput

  t::Float64
  v_d::Float64             #commanded acceleration (ft/s^2)
  h_d::Float64             #commanded vertical rate (ft/s)
  psi_d::Float64           #commanded turn rate (deg/s)
  dh_min::Float64          #dh_min of advisory being followed (ft/s)
  dh_max::Float64          #dh_max of advisory being followed (ft/s)
  target_rate::Float64     #target_rate of advisory being followed (ft/s)
  ddh::Float64             #ddh of advisory being followed (ft/s^2)

  logProb::Float64 #log probability of generating this command
end

LLDetPROutput() = LLDetPROutput(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)

type LLDetPRRA

  dh_min::Float64
  dh_max::Float64
  target_rate::Float64
  ddh::Float64

end

type QueueEntry

  t::Int64          #time left before reaching front of queue (s)
  RA::LLDetPRRA     #associated RA

end

type LLDetPR <: AbstractPilotResponse

  initial_resp_time::Int64 #parameter
  subsequent_resp_time::Int64 #parameter

  state::Symbol #{none,follow,stay}
  queue::Vector{QueueEntry}
  timer::Int64

  output::LLDetPROutput #preallocate

  function LLDetPR(initial_resp_time::Int64,subsequent_resp_time::Int64)

    obj = new()
    obj.initial_resp_time = initial_resp_time
    obj.subsequent_resp_time = subsequent_resp_time

    obj.state = :none
    obj.queue = QueueEntry[]
    obj.timer = 0

    obj.output = LLDetPROutput()

    return obj
  end
end

function isequal(x::LLDetPRRA,y::LLDetPRRA)

  return x.dh_min == y.dh_min && x.dh_max == y.dh_max && x.target_rate == y.target_rate
end

function add_to_queue!(pr::LLDetPR,q::Vector{QueueEntry},RA::LLDetPRRA)

  t_left = isempty(q) ? pr.initial_resp_time : pr.subsequent_resp_time

  filter!(x->x.t < t_left,q) #all elements should have a time smaller than what's being added

  el = QueueEntry(t_left,RA)
  push!(q,el)

end

#TODO: replace with findlast in 0.4
function findlastzero(q::Vector{QueueEntry})

  idx = find(x->x.t==0,q)

  return length(idx) > 0 ? idx[end] : 0 #index of the last zero.  Returns 0 if empty.
end

function ischange(q::Vector{QueueEntry},RA::LLDetPRRA)

  if isempty(q)
    return true
  end

  return !isequal(q[end].RA,RA)
end

function updatePilotResponse(pr::LLDetPR, update::LLDetPRCommand, RA::LLDetPRRA)

  t, v_d, h_d, psi_d = update.t, update.v_d, update.h_d, update.psi_d

  @test RA.dh_min <= RA.target_rate <= RA.dh_max #this should always hold

  #decrement timer for all ra's in queue
  for i=1:endof(pr.queue)
    pr.queue[i].t = max(0,pr.queue[i].t-1) #TODO: remove hardcoding of t-1
  end

  #incorporate the new RA
  if RA.dh_min <= -9999.0 && 9999.0 <= RA.dh_max  #TODO: put these hardcodings in a common place
    #coc
    empty!(pr.queue)
    pr.state = :none
  elseif ischange(pr.queue,RA)
    #RA
    add_to_queue!(pr,pr.queue,RA)
  end

  #shift items to the next one with time 0
  last_index = findlastzero(pr.queue)
  splice!(pr.queue,1:last_index-1)

  pr.timer = 0

  #default
  dh_min = -9999.0
  dh_max = 9999.0
  target_rate = 0.0
  ddh = 0.0

  if !isempty(pr.queue)
    if pr.queue[1].t == 0 #there is an ra in the queue and its time has expired
      h_d = pr.queue[1].RA.target_rate
      dh_min = pr.queue[1].RA.dh_min
      dh_max = pr.queue[1].RA.dh_max
      target_rate = pr.queue[1].RA.target_rate
      ddh = pr.queue[1].RA.ddh

      if length(pr.queue) > 1
        #this is not the last item
        pr.state = :stay
        pr.timer = pr.queue[2].t
      else
        #this is the last item
        pr.state = :follow
      end
    else
      #still waiting for the first item
      pr.state = :none
      pr.timer = pr.queue[1].t
    end
  end

  pr.output.t = t
  pr.output.v_d = v_d
  pr.output.h_d = h_d
  pr.output.psi_d = psi_d
  pr.output.dh_min = dh_min
  pr.output.dh_max = dh_max
  pr.output.target_rate = target_rate
  pr.output.ddh = ddh
  pr.output.logProb = 0.0 #probability 1

  return pr.output
end

step(pr::LLDetPR, update, RA) = step(pr,convert(LLDetPRCommand, update), convert(LLDetPRRA,RA))

step(pr::LLDetPR, update::LLDetPRCommand, RA::LLDetPRRA) = updatePilotResponse(pr, update, RA)

function initialize(pr::LLDetPR)

  pr.state = :none
  empty!(pr.queue)

  pr.output.t = 0.0
  pr.output.v_d = 0.0
  pr.output.h_d = 0.0
  pr.output.psi_d = 0.0
  pr.output.dh_min = -9999.0  #TODO: remove these hardcodings into a central location
  pr.output.dh_max = 9999.0
  pr.output.target_rate = 0.0
  pr.output.ddh = 0.0
  pr.output.logProb = 0.0

  nothing
end

end


