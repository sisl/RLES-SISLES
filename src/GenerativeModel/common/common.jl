using Encounter
using Sensor
using CollisionAvoidanceSystem
using PilotResponse
using DynamicModel
using WorldModel

import Base.convert

#The fact that these are distributed between the simulators (i.e., TCASSimulatorImpl.jl),
#is a bit messy.
convert(::Type{StochasticLinearPRCommand}, command::Union(CorrAEMCommand, LLAEMCommand)) = StochasticLinearPRCommand(command.t, command.v_d, command.h_d, command.psi_d, 0.0)

convert(::Type{DeterministicPRCommand}, command::Union(CorrAEMCommand, LLAEMCommand)) = DeterministicPRCommand(command.t, command.v_d, command.h_d, command.psi_d, 0.0)

convert(::Type{SimpleADMCommand}, command::StochasticLinearPRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)

convert(::Type{SimpleADMCommand}, command::DeterministicPRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)

function convert(::Type{ACASXSensorInput}, states::Vector{ASWMState})

  sr_states = ACASXSensorState[ ACASXSensorState(state.x,state.y,state.h,state.vx,state.vy,state.vh)
                               for state in states]

  return ACASXSensorInput(sr_states)
end

function convert(::Type{StochasticLinearPRRA},RA::ACASXOutput)
  ra_active = (RA.dh_min > -9999.0 || RA.dh_max < 9999.0)
  return StochasticLinearPRRA(ra_active,RA.target_rate,RA.dh_min,RA.dh_max)
end

function convert(::Type{StochasticLinearPRRA},RA::Union(SimpleTCASResolutionAdvisory,Nothing))
  ra_active = RA != nothing
  return StochasticLinearPRRA(ra_active,ra_active ? RA.h_d : 0.0,-9999.0,9999.0)
end

function convert(::Type{DeterministicPRRA},RA::ACASXOutput)
  return DeterministicPRRA(RA.dh_min,RA.dh_max,RA.target_rate)
end

function convert(::Type{DeterministicPRRA},RA::Union(SimpleTCASResolutionAdvisory,Nothing))
  ra_active = RA != nothing
  if !ra_active
    #not active
    return DeterministicPRRA(-9999.0,9999.0,0.0)
  else
    #RA
    if RA.h_d > 0
      #climb
      return DeterministicPRRA(RA.h_d, 9999.0, RA.h_d)
    elseif RA.h_d < 0
      #descend
      return DeterministicPRRA(-9999.0, RA.h_d, RA.h_d)
    else
      #level
      return DeterministicPRRA(RA.h_d, RA.h_d, RA.h_d)
    end
  end

  error("Shouldn't have gotten here")
end
