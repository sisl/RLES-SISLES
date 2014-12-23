# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014


module CASCoordination

export
    AbstractCASCoord,

    initialize,
    step,

    setRecord,
    getRecord,
    getAllRecords,

    GenericCoord

using AbstractCASCoordImpl

using GenericCoordImpl

end


