using Encounter
using PilotResponse
using DynamicModel

import Base.convert

#The fact that these are distributed between the simulators (i.e., TCASSimulatorImpl.jl),
#is a bit messy.
convert(::Type{StochasticLinearPRCommand}, command::Union(CorrAEMCommand, LLAEMCommand)) = StochasticLinearPRCommand(command.t, command.v_d, command.h_d, command.psi_d, 1.0)
convert(::Type{SimpleADMCommand}, command::StochasticLinearPRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)
