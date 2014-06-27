# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module SimplePilotResponseImpl

export
    initialize,
    step,

    updatePilotResponse,

    SimplePilotResponse,
    SimplePRResolutionAdvisory,
    SimplePRCommand


using AbstractPilotResponseImpl
using Interfaces

import Interfaces.step
import Interfaces.updatePilotResponse


type SimplePRResolutionAdvisory

    h_d::Float64
end

type SimplePRCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64
end

type SimplePilotResponse <: AbstractPilotResponse

    b_CAS_activated::Bool
    RA::SimplePRResolutionAdvisory

    function SimplePilotResponse()

        obj = new()

        obj.b_CAS_activated = false

        return obj
    end
end

function updatePilotResponse(pr::SimplePilotResponse, update::SimplePRCommand, RA::Union(SimplePRResolutionAdvisory, Nothing))

    t, v_d, h_d, psi_d = update.t, update.v_d, update.h_d, update.psi_d

    if RA != nothing && pr.b_CAS_activated == false
        pr.RA = RA
        pr.b_CAS_activated = true
    end

    if pr.b_CAS_activated
        h_d = pr.RA.h_d
    end

    return SimplePRCommand(t, v_d, h_d, psi_d)
end

step(pr::SimplePilotResponse, update::SimplePRCommand, RA::Union(SimplePRResolutionAdvisory, Nothing)) = updatePilotResponse(pr, update, RA)

function initialize(pr::SimplePilotResponse)

    pr.b_CAS_activated = false
end

end


