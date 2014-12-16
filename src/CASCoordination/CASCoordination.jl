# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014


module CASCoordination

export
    AbstractCASCoord,
    AbstractCoordRecord,

    initialize,
    step,

    setRecord,
    getRecord,
    getAllRecords,

    GenericCoord,
    GenericCoordRecord

using AbstractCASCoordImpl

using GenericCoordImpl

end


