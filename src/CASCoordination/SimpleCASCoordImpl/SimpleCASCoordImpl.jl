# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014

module SimpleCASCoordImpl

export
    initialize,
    step,

    updateRecord,
    getRecord,
    getAllRecords,
    getAll,

    SimpleCASCoord,
    SimpleCASCoordRecord


using AbstractCASCoordImpl
using AbstractCASCoordInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractCASCoordInterfaces.updateRecord
import AbstractCASCoordInterfaces.getRecord
import AbstractCASCoordInterfaces.getAllRecords
import AbstractCASCoordInterfaces.getAll

type SimpleCASCoordRecord
  RA     #Consider typing this in the future

  SimpleCASCoordRecord() = SimplCASCoordRecord(nothing) #defaults to nothing = no RA
end

type SimpleCASCoord <: AbstractCASCoord

    num_records::Int64
    records::Vector{SimpleCASCoordRecord}

    function SimpleCASCoord(num_records::Int)
        obj = new()

        obj.num_records = num_records
        obj.records = Array(SimpleCASCoordRecord, num_records)

        return obj
    end
end


function updateRecord(coord::SimpleCASCoord, record_number::Int, RA)

    coord.records[record_number] = RA
end

step(coord::SimpleCASCoord, record_number::Int, RA) = updateRecord(coord, record_number, RA)

function initialize(coord::SimpleCASCoord)

  for r in coord.records
    r.RA = nothing  #reset all RA's to nothing
  end
end

getRecord(coord::SimpleCASCoord,record_number::Int) = coord.records[record_number]

function getAllRecords(coord::SimpleCASCoord)

    return coord.records
end

getAll(coord::SimpleCASCoord) = getAllRecords(coord)

end


