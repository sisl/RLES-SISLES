# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


module Encounter

export
    AbstractEncounterModel,
    AddObserver,

    step,

    generateEncountersToFile,
    generateEncounter,
    getInitialState,
    getNextCommand,
    getTrajectory,

    CorrAEM,
    CorrAEMInitialState,
    CorrAEMCommand,
    validate,

    LLAEM,
    LLAEMInitialState,
    LLAEMCommand


using AbstractEncounterModelImpl

using CorrAEMImpl
using LLAEMImpl

end


