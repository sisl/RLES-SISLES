# Author: Ritchie Lee, ritchie.lee@sv.cmu.reduce
# Date: 12/15/2014


module EncounterDBN

export
    AbstractEncounterDBN,
    AddObserver,

    getInitialState,
    initialize,
    step,
    get,

    CorrAEMDBN,
    StarDBN,
    PairwiseCorrAEMDBN

import CommonInterfaces: initialize, step

using AbstractEncounterDBNImpl
import AbstractEncounterDBNInterfaces: getInitialState, get

using CorrAEMDBNImpl

using StarDBNImpl

using PairwiseCorrAEMDBNImpl

end


