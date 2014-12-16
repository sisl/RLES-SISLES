# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014

module GenericCoordImpl

export
    initialize,
    step,

    setRecord,
    getRecord,
    getAllRecords,

    GenericCoord,
    GenericCoordRecord


using AbstractCASCoordImpl
using AbstractCASCoordInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractCASCoordInterfaces.setRecord
import AbstractCASCoordInterfaces.getRecord
import AbstractCASCoordInterfaces.getAllRecords

type GenericCoordRecord
  record::Union(AbstractCoordRecord,Nothing)
end

GenericCoordRecord() = GenericCoordRecord(nothing) #defaults to nothing = no sync objects

type GenericCoord <: AbstractCASCoord

  num_records::Int64
  records::Vector{GenericCoordRecord}

  function GenericCoord(num_records::Int)
    obj = new()

    obj.num_records = num_records
    obj.records = GenericCoordRecord[ GenericCoordRecord() for i=1:num_records]

    return obj
  end
end

function setRecord(coord::GenericCoord, record_number::Int, record::AbstractCoordRecord)

    coord.records[record_number].record = record
end

step(coord::GenericCoord, record_number::Int, record::AbstractCoordRecord) = setRecord(coord, record_number, coord_obj)

function initialize(coord::GenericCoord)

  for r in coord.records
    r.record = nothing  #reset to nothing
  end
end

getRecord(coord::GenericCoord,record_number::Int) = coord.records[record_number].record

getAllRecords(coord::GenericCoord) = coord.records

end


