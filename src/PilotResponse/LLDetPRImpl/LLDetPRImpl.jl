# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 04/16/2014
# Renamed from DeterministicPR and added changes to match LL pilot response/dynamics

module LLDetPRImpl

export
    initialize,
    update,

    updatePilotResponse,

    LLDetPR,
    LLDetPRAircraft,
    LLDetPRCommand,
    LLDetPRRA,
    LLDetPROutput

using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractPilotResponseInterfaces.updatePilotResponse

import Base: isequal, ==

type LLDetPRAircraft
    vh::Float64             #vertical rate of aircraft (ft/s)
end

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

LLDetPROutput() = LLDetPROutput(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

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
    initial_resp_time::Int64 #initial response delay
    subsequent_resp_time::Int64 #subsequent response delay
    limit_hd::Bool #limit pilot command rate to (dh_min,dh_max)
    state::Symbol #{none,follow,stay}
    queue::Vector{QueueEntry}
    timer::Int64
    COC_RA::LLDetPRRA
    output::LLDetPROutput #preallocate

    function LLDetPR(initial_resp_time::Int64, subsequent_resp_time::Int64; limit_hd::Bool=true)
        obj = new()
        obj.initial_resp_time = initial_resp_time
        obj.subsequent_resp_time = subsequent_resp_time
        obj.limit_hd = limit_hd;
        obj.state = :none
        obj.queue = QueueEntry[]
        obj.timer = 0
        obj.COC_RA = LLDetPRRA(-9999.0, 9999.0, 0.0, 0.0) #COC TODO: Remove hardcoding of 9999.0, or at least centralize it
        obj.output = LLDetPROutput()
        obj
    end
end

==(x::LLDetPRRA, y::LLDetPRRA) = isequal(x, y)
function isequal(x::LLDetPRRA, y::LLDetPRRA)
  x.dh_min == y.dh_min && x.dh_max == y.dh_max && x.target_rate == y.target_rate
end

function add_to_queue!(q::Vector{QueueEntry}, RA::LLDetPRRA,
                       queuetime::Int64)
  if queuetime < 0
    return
  end
  filter!(x -> x.t < queuetime, q) #all elements should have a time smaller than what's being added
  el = QueueEntry(queuetime, RA)
  push!(q, el)
end

#TODO: replace with findlast when we switch to julia 0.4
#function findlastzero(q::Vector{QueueEntry})
  #idx = find(x -> x.t == 0, q)
  #return length(idx) > 0 ? idx[end] : 0 #index of the last zero.  Returns 0 if empty.
#end

isfirstRA(pr::LLDetPR) = isequal(pr.queue[1].RA, pr.COC_RA) && length(pr.queue) == 1

function updatePilotResponse(pr::LLDetPR, command::LLDetPRCommand, RA::LLDetPRRA, 
    ac_state::LLDetPRAircraft)
    t, v_d, h_d, psi_d = command.t, command.v_d, command.h_d, command.psi_d

    @assert RA.dh_min <= RA.target_rate <= RA.dh_max #this should always hold

    #decrement timer for all ra's in queue
    for i = 1:endof(pr.queue)
        pr.queue[i].t = max(0, pr.queue[i].t - 1) #TODO: remove hardcoding of t-1 for step size
    end

    #incorporate the new RA if it's new.  Could be coc
    if !isequal(pr.queue[end].RA, RA)
        if isequal(RA, pr.COC_RA)
            queuetime = 0 #COC's happen immediately
        elseif isfirstRA(pr)
            queuetime = pr.initial_resp_time
        else
            queuetime = pr.subsequent_resp_time
        end
        add_to_queue!(pr.queue, RA, queuetime)
    end

    #shift items to the next one with time = 0
    last_index = findlast(map(x->x.t, pr.queue), 0)
    splice!(pr.queue, 1:(last_index - 1))

    @assert !isempty(pr.queue) #queue should never be empty
    @assert pr.queue[1].t == 0 #first item should always have its time expired

    dh_min      = pr.queue[1].RA.dh_min
    dh_max      = pr.queue[1].RA.dh_max
    target_rate = pr.queue[1].RA.target_rate
    ddh         = pr.queue[1].RA.ddh

    if isequal(pr.queue[1].RA, pr.COC_RA)
        #currently doing COC
        pr.state = :none

        #restrict pilot to not accelerate against any pending RA's
        if pr.limit_hd && length(pr.queue) > 1 #currently COC, but RA is queued
            h_d = pr.queue[2].RA.target_rate > ac_state.vh ? 
                max(ac_state.vh, h_d) : min(ac_state.vh, h_d)
        end 
    else
        h_d = pr.queue[1].RA.target_rate #follow RA
    
        if length(pr.queue) > 1
            #currently following an RA and there is another RA queued
            pr.state = :stay
        else
            #currently following an RA and there are no other RA's queued
            pr.state = :follow
        end
    end

    # if there is a next item, show its timer, otherwise 0
    pr.timer = length(pr.queue) > 1 ? pr.queue[2].t : 0

    pr.output.t = t
    pr.output.v_d = v_d
    pr.output.h_d = h_d
    pr.output.psi_d = psi_d
    pr.output.dh_min = dh_min
    pr.output.dh_max = dh_max
    pr.output.target_rate = target_rate
    pr.output.ddh = ddh
    pr.output.logProb = 0.0 #probability = 1

    pr.output
end

function update(pr::LLDetPR, command, RA, ac_state) 
    update(pr, convert(LLDetPRCommand, command), convert(LLDetPRRA, RA), 
        convert(LLDetPRAircraft, ac_state))
end
function update(pr::LLDetPR, command::LLDetPRCommand, RA::LLDetPRRA, ac_state::LLDetPRAircraft) 
    updatePilotResponse(pr, command, RA, ac_state)
end

function initialize(pr::LLDetPR)
    pr.state = :none
    empty!(pr.queue)
    add_to_queue!(pr.queue, pr.COC_RA, 0)

    pr.output.t = 0.0
    pr.output.v_d = 0.0
    pr.output.h_d = 0.0
    pr.output.psi_d = 0.0
    pr.output.dh_min = -9999.0
    pr.output.dh_max = 9999.0
    pr.output.target_rate = 0.0
    pr.output.ddh = 0.0
    pr.output.logProb = 0.0

    nothing
end

end #module


