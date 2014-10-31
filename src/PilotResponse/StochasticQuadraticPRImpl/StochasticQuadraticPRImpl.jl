# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 10/13/2014


module StochasticQuadraticPRImpl

export
    initialize,
    step,

    updatePilotResponse,

    StochasticQuadraticPR,
    StochasticQuadraticPRResolutionAdvisory,
    StochasticQuadraticPRCommand


using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractPilotResponseInterfaces.updatePilotResponse


type StochasticQuadraticPRResolutionAdvisory

    h_d::Float64
end

type StochasticQuadraticPRCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64
end

type StochasticQuadraticPR <: AbstractPilotResponse

  activeRA::Symbol
  response::Symbol
  reversal::Bool
  probTable::Dict{Vector{Symbol},(Vector{Float64},Vector{Symbol})}

  function StochasticQuadraticPR()

    obj = new()
    obj.activeRA = :none
    obj.response = :none
    obj.reversal = false
    obj.probTable = getProbabilityDict()

    return obj
  end
end

function getProbabilityDict()
  #key = (activeRA,response,currentRA,reversal_true/false)
  #value = (probability vector, output values vector)
  # where output values vector is an array of tuples (activeRA,response) corresponding
  # to each probability given in probability vector
  d = Dict{(Symbol,Symbol,Symbol,Bool),(Vector{Float64},Vector{(Symbol,Symbol)})}()

  # all entries going to :none go there with prob 1
  d[(:none,:none,:none,true)] = ([1.0],[(:none,:none)])
  d[(:none_none,:none,false)] = ([1.0],[(:none,:none)])
  d[(:climb,:none,:none,true)] = ([1.0],[:none,:none])
  d[(:climb,:none,:none,false)] = ([1.0],[(:none,:none)])
  d[(:climb,:climb,:none,true)] = ([1.0],[(:none,:none)])
  d[(:climb,:climb,:none,false)] = ([1.0],[(:none,:none)])
  d[(:descend,:none,:none,true)] = ([1.0],[(:none,:none)])
  d[(:descend,:none,:none,false)] = ([1.0],[(:none,:none)])
  d[(:descend,:descend,:none,true)] = ([1.0],[:none,:none])
  d[(:descend,:descend,:none,false)] = ([1.0],[(:none,:none)])

  # climb first, descend as reversal
  d[(:none,:none,:climb,false)] = ([1/6,5/6],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:none,:climb,false)] = ([1/6,5/6],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:none,:descend,false)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:climb,:climb,:climb,false)] = ([1.0],[(:climb,:climb)])
  d[(:climb,:climb,:descend,false)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:none,:descend,true)] = ([1/4,3/4],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:descend,:descend,true)] = ([1.0],[(:descend,:descend)])

  # descend first, climb as reversal
  d[(:none_none,:descend,false)] = ([1/6,5/6],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:none,:descend,false)] = ([1/6,5/6],[(:descend,:descend),(:descend,:none)])
  d[(:descend,:none,:climb,false)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])
  d[(:descend,:descend,:descend,false)] = ([1.0],[(:descend,:descend)])
  d[(:descend,:descend,:climb,false)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:none,:climb,true)] = ([1/4,3/4],[(:climb,:climb),(:climb,:none)])
  d[(:climb,:climb,:climb,true)] = ([1.0],[(:climb,:climb)])

  return d
end

function updatePilotResponse(pr::StochasticQuadraticPR, update::StochasticQuadraticPRCommand, RA::Union(StochasticQuadraticPRResolutionAdvisory, Nothing))

  t, v_d, h_d, psi_d = update.t, update.v_d, update.h_d, update.psi_d

  if RA == nothing
    ra = :none
    pr.reversal = false
  else
    ra = RA.h_d >= 0 ? :climb : :descend
    pr.reversal |=  (pr.activeRA == :descend && ra == :climb) || (pr.activeRA == :climb && ra == :descend)
  end

  probabilities,values = pr.probTable[(pr.activeRA,pr.response,ra,reversal)]
  index = select_random(probabilities)
  pr.activeRA,pr.response = values[index]

  if pr.response != :none
    h_d = RA.h_d
  end

  return StochasticQuadraticPRCommand(t, v_d, h_d, psi_d)
end

step(pr::StochasticQuadraticPR, update::StochasticQuadraticPRCommand, RA::Union(StochasticQuadraticPRResolutionAdvisory, Nothing)) = updatePilotResponse(pr, update, RA)

function initialize(pr::StochasticQuadraticPR)

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


