# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 10/13/2014

# Reference: Chryssanthacopoulos, Kochenderfer - Collision Avoidance System Optimization with
# Probabilistic Pilot Response Models. American Control Conference. 2011.


module StochasticLinearPRImpl

export
    initialize,
    step,

    updatePilotResponse,

    StochasticLinearPR,
    StochasticLinearPRCommand,
    StochasticLinearPRRA

using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractPilotResponseInterfaces.updatePilotResponse

function generateProbabilityDict()
  #key = (activeRA,response,currentRA,reversal_true/false)
  #value = (probability vector, output values vector)
  # where output values vector is an array of tuples (activeRA,response) corresponding
  # to each probability given in probability vector
  d = Dict{(Symbol,Symbol,Symbol,Bool),(Vector{Float64},Vector{(Symbol,Symbol)})}()

  # all entries going to :none go there with prob 1
  d[(:none,:none,:none,true)] = ([1.0],[(:none,:none)])
  d[(:none,:none,:none,false)] = ([1.0],[(:none,:none)])
  d[(:climb,:none,:none,true)] = ([1.0],[(:none,:none)])
  d[(:climb,:none,:none,false)] = ([1.0],[(:none,:none)])
  d[(:climb,:climb,:none,true)] = ([1.0],[(:none,:none)])
  d[(:climb,:climb,:none,false)] = ([1.0],[(:none,:none)])
  d[(:descend,:none,:none,true)] = ([1.0],[(:none,:none)])
  d[(:descend,:none,:none,false)] = ([1.0],[(:none,:none)])
  d[(:descend,:descend,:none,true)] = ([1.0],[(:none,:none)])
  d[(:descend,:descend,:none,false)] = ([1.0],[(:none,:none)])

  # first RA - climb
  d[(:none,:none,:climb,false)] = ([1/6,5/6],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:none,:climb,false)] = ([1/6,5/6],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:none,:descend,false)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:climb,:climb,:climb,false)] = ([1.0],[(:climb,:climb)])
  d[(:climb,:climb,:descend,false)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])

  # first RA - descend
  d[(:none,:none,:descend,false)] = ([1/6,5/6],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:none,:descend,false)] = ([1/6,5/6],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:none,:climb,false)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])
  d[(:descend,:descend,:descend,false)] = ([1.0],[(:descend,:descend)])
  d[(:descend,:descend,:climb,false)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])

  # reversal RA
  d[(:descend,:none,:descend,true)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:descend,:descend,true)] = ([1.0],[(:descend,:descend)])
  d[(:climb,:none,:climb,true)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:climb,:climb,true)] = ([1.0],[(:climb,:climb)])

  # reversal RA - not explicit in paper
  d[(:climb,:none,:descend,true)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:climb,:climb,:descend,true)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:none,:climb,true)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])
  d[(:descend,:descend,:climb,true)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])

  return d
end

const probabilityDict = generateProbabilityDict() #generate it once and store it

type StochasticLinearPRCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64

    logProb::Float64 #log probability of generating this command
end

type StochasticLinearPRRA
  explicit_ra::Bool #true to force target_rate to be followed
  target_rate::Float64
  dh_min::Float64 #min bound on h_d
  dh_max::Float64 #max bound on h_d
end

type StochasticLinearPR <: AbstractPilotResponse

  activeRA::Symbol #RA currently displaying = [:none, :climb, :descend]
  response::Symbol #current pilot response = [:none, :climb, :descend]
  reversal::Bool #is the activeRA a reversal?

  probTable::Dict{(Symbol,Symbol,Symbol,Bool),(Vector{Float64},Vector{(Symbol,Symbol)})} #transition prob table

  function StochasticLinearPR()

    obj = new()
    obj.activeRA = :none
    obj.response = :none
    obj.reversal = false
    obj.probTable = probabilityDict

    return obj
  end
end

function updatePilotResponse(pr::StochasticLinearPR, update::StochasticLinearPRCommand, RA::StochasticLinearPRRA)

  t, v_d, h_d, psi_d = update.t, update.v_d, update.h_d, update.psi_d

  #Two ways to signal an active RA
  #1. explicit_ra is true.  e.g., SimpleTCAS will use this signal that commanded h_d is valid
  #2. if intended h_d is outside of RA bounds, then consider RA as active.  e.g., ACASX
  # provides dh_min and dh_max as well as a target_rate
  if RA.explicit_ra || !(RA.dh_min <= update.h_d <= RA.dh_max)

    ra = RA.target_rate >= 0 ? :climb : :descend

    #Debug
    #@show ra
    #@show pr.reversal
    #@show pr.activeRA
    #@show RA.h_d

    pr.reversal |=  (pr.activeRA == :descend && ra == :climb) || (pr.activeRA == :climb && ra == :descend)
  else
    ra = :none
    pr.reversal = false
  end

  probabilities,values = pr.probTable[(pr.activeRA,pr.response,ra,pr.reversal)]
  index = select_random(probabilities)
  pr.activeRA,pr.response = values[index]

  if pr.response != :none
    h_d = RA.target_rate
  end

  return StochasticLinearPRCommand(t, v_d, h_d, psi_d, log(probabilities[index]))
end

step(pr::StochasticLinearPR, update, RA) = step(pr,
                                                convert(StochasticLinearPRCommand, update),
                                                convert(StochasticLinearPRRA,RA))

step(pr::StochasticLinearPR,
     update::StochasticLinearPRCommand,
     RA::StochasticLinearPRRA) = updatePilotResponse(pr, update, RA)

function initialize(pr::StochasticLinearPR)

    pr.activeRA = :none
    pr.response = :none
    pr.reversal = false

end

function select_random(weights)
# SELECT_RANDOM Randomly selects an index according to specified weights.
#   Returns a randomly selected index according to the distribution
#   specified by a vector of weights.
#
#   INDEX = SELECT_RANDOM(WEIGHTS) returns a scalar index INDEX selected
#   randomly according to the specified weights WEIGHTS represented as an
#   array.

    s = cumsum(weights)
    r = s[end] * rand()
    index = findfirst(x -> (x >= r), s)

    return index
end

end


