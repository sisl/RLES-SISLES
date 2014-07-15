# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


module Encounter

export
    AbstractEncounterModel,
    AddObserver,

    step,

    generateEncounter,
    getInitialState,
    getNextCommand,
    getTrajectory,

    CorrAEM,
    CorrAEMInitialState,
    CorrAEMCommand,
    generateEncountersToFile,
    setInitialDistributions,
    validate,
    getInitialSample,

    LLAEM,
    LLAEMInitialState,
    LLAEMCommand


using AbstractEncounterModelImpl

using CorrAEMImpl
using LLAEMImpl

end


