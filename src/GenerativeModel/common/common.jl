using Encounter
using Sensor
using CollisionAvoidanceSystem
using PilotResponse
using DynamicModel
using WorldModel

import Base.convert

#The fact that these are distributed between the simulators (i.e., TCASSimulatorImpl.jl),
#is a bit messy.
convert(::Type{StochasticLinearPRCommand}, command::Union(CorrAEMCommand, LLAEMCommand)) = StochasticLinearPRCommand(command.t, command.v_d, command.h_d, command.psi_d, 1.0)

convert(::Type{SimpleADMCommand}, command::StochasticLinearPRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)

function convert(::Type{ACASXSensorInput}, states::Vector{ASWMState})

  sr_states = ACASXSensorState[ ACASXSensorState(state.x,state.y,state.h,state.vx,state.vy,state.vh)
                               for state in states]

  return ACASXSensorInput(sr_states)
end

function convert(::Type{SimplePRResolutionAdvisory},RA::ACASXOutput)
  return SimplePRResolutionAdvisory(RA.target_rate)
end
