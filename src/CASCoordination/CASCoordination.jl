# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014


module CASCoordination

export
    AbstractCASCoord,

    initialize,
    step,

    updateCoordObj,
    getCoordObj,
    getAllRecords,
    getAll,

    SimpleCASCoord,
    SimpleCASCoordRecord

using AbstractCASCoordImpl

using SimpleCASCoordImpl

end


