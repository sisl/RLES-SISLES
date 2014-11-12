# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014


module AbstractCASCoordImpl

export AbstractCASCoord

abstract AbstractCASCoord

end


module AbstractCASCoordInterfaces

export
    updateRecord,
    getRecord,
    getAllRecords,
    getAll

function updateRecord()
end

function getRecord()
end

function getAllRecords()
end

function getAll()
end

end


