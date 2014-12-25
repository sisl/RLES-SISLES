# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module PilotResponse

export
    AbstractPilotResponse,

    initialize,
    step,

    updatePilotResponse,

    SimplePilotResponse,
    SimplePRResolutionAdvisory,
    SimplePRCommand,

    StochasticLinearPR,
    StochasticLinearPRCommand,
    StochasticLinearPRRA


using AbstractPilotResponseImpl

using SimplePilotResponseImpl

using StochasticLinearPRImpl

end


