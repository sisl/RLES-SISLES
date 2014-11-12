# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014

module SimpleCASCoordImpl

export
    initialize,
    step,

    updateCoordObj,
    getCoordObj,
    getAllRecords,
    getAll,

    SimpleCASCoord,
    SimpleCASCoordRecord


using AbstractCASCoordImpl
using AbstractCASCoordInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractCASCoordInterfaces.updateCoordObj
import AbstractCASCoordInterfaces.getCoordObj
import AbstractCASCoordInterfaces.getAllRecords
import AbstractCASCoordInterfaces.getAll

type SimpleCASCoordRecord
  coord_obj     #Consider typing this
end

SimpleCASCoordRecord() = SimpleCASCoordRecord(nothing) #defaults to nothing = no sync objects

type SimpleCASCoord <: AbstractCASCoord

  num_records::Int64
  records::Vector{SimpleCASCoordRecord}

  function SimpleCASCoord(num_records::Int)
    obj = new()

    obj.num_records = num_records
    obj.records = SimpleCASCoordRecord[ SimpleCASCoordRecord() for i=1:num_records]

    return obj
  end
end

function updateCoordObj(coord::SimpleCASCoord, record_number::Int, coord_obj)

    coord.records[record_number].coord_obj = coord_obj
end

step(coord::SimpleCASCoord, record_number::Int, coord_obj) = updateRecord(coord, record_number, coord_obj)

function initialize(coord::SimpleCASCoord)

  for r in coord.records
    r.coord_obj = nothing  #reset to nothing
  end
end

getCoordObj(coord::SimpleCASCoord,record_number::Int) = coord.records[record_number].coord_obj

getAllRecords(coord::SimpleCASCoord) = coord.records

getAll(coord::SimpleCASCoord) = getAllSyncObjs(coord)

end


