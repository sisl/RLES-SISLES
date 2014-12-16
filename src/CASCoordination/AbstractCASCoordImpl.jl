# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014


module AbstractCASCoordImpl

export AbstractCASCoord, AbstractCoordRecord

abstract AbstractCASCoord

abstract AbstractCoordRecord

end


module AbstractCASCoordInterfaces

export
    setRecord,
    getRecord,
    getAllRecords

function setRecord()
end

function getRecord()
end

function getAllRecords()
end

end


