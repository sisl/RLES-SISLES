# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module TCASSimulatorImpl

export
    simulate,
    TCASSimulator


using AbstractSimulatorImpl

using Encounter
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CollisionAvoidanceSystem


type SimulationParameters

    em::AbstractEncounterModel
    pr::Vector{AbstractPilotResponse}
    dm::Vector{AbstractDynamicModel}
    wm::AbstractWorldModel
    sr::Vector{AbstractSensor}
    cas::Vector{AbstractCollisionAvoidanceSystem}

    number_of_aircraft::Int


    SimulationParameters() = new()
end

type TCASSimulator <: AbstractSimulator

    parameters::SimulationParameters

    time_step::Float64


    function TCASSimulator()

        parameters = SimulationParameters()
        time_step = 1.

        return new(parameters, time_step)
    end
end


import Base.convert

convert(::Type{SimpleADMInitialState}, state::Union(CorrAEMInitialState, LLAEMInitialState)) = SimpleADMInitialState(state.t, state.x, state.y, state.h, state.v, state.psi, state.h_d)

convert(::Type{ASWMState}, state::SimpleADMOutputState) = ASWMState(state.t, state.x, state.y, state.h, state.vx, state.vy, state.vh)

convert(::Type{SimplePRCommand}, command::Union(CorrAEMCommand, LLAEMCommand)) = SimplePRCommand(command.t, command.v_d, command.h_d, command.psi_d)

convert(::Type{SimpleADMCommand}, command::SimplePRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)

function convert(::Type{SimpleTCASSensorInput}, states::Vector{ASWMState})

    states_ = Array(SimpleTCASSRState, length(states))

    for i = 1:length(states)
        states_[i] = SimpleTCASSRState(states[i].t, states[i].x, states[i].y, states[i].h, states[i].vx, states[i].vy, states[i].vh)
    end

    return SimpleTCASSensorInput(states_)
end

convert(::Type{SimpleTCASInput}, output::SimpleTCASSensorOutput) = SimpleTCASInput(output.t, output.r, output.r_d, output.a, output.a_d, output.h, output.h_d)

convert(::Type{SimplePRResolutionAdvisory}, RA::SimpleTCASResolutionAdvisory) = SimplePRResolutionAdvisory(RA.h_d)
convert(::Type{SimplePRResolutionAdvisory}, RA::Nothing) = nothing


function simulate(sim::AbstractSimulator; bTCAS = false, sample_number = 0)

    aem = sim.parameters.em
    pr = sim.parameters.pr
    adm = sim.parameters.dm
    as = sim.parameters.wm
    sr = sim.parameters.sr
    cas = sim.parameters.cas

    number_of_aircraft = sim.parameters.number_of_aircraft


    if sample_number == 0
        generateEncounter(aem)
    else
        generateEncounter(aem, sample_number)
    end

    for i = 1:number_of_aircraft
        initial = Encounter.getInitialState(aem, i)
        state = DynamicModel.initialize(adm[i], convert(SimpleADMInitialState, initial))
        WorldModel.initialize(as, i, convert(ASWMState, state))

        Sensor.initialize(sr[i])
        CollisionAvoidanceSystem.initialize(cas[i])
        PilotResponse.initialize(pr[i])
    end

    command = nothing

    while true
        for i = 1:number_of_aircraft
            command = Encounter.step(aem, i)

            if command == nothing
                break
            end

            if bTCAS
                states = WorldModel.getAll(as)
                output = Sensor.step(sr[i], convert(SimpleTCASSensorInput, states))
                RA = CollisionAvoidanceSystem.step(cas[i], convert(SimpleTCASInput, output))
            else
                RA = nothing
            end

            response = PilotResponse.step(pr[i], convert(SimplePRCommand, command), convert(SimplePRResolutionAdvisory, RA))
            state = DynamicModel.step(adm[i], convert(SimpleADMCommand, response))
            WorldModel.step(as, i, convert(ASWMState, state))
        end

        if command == nothing
            break
        end

        WorldModel.updateAll(as)
    end
end

end


