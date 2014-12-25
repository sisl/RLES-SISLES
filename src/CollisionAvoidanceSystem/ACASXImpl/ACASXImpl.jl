# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/21/2014

#ACASX implementation: based on CCAS interface to libCAS

module ACASXImpl

export
    AddObserver,

    initialize,
    step,

    ACASX,
    ACASXInput,
    ACASXOutput


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces
using ObserverImpl

#using Util
using Base.Test

import CommonInterfaces.initialize
import CommonInterfaces.step

using AbstractCASCoordImpl

using CASCoordination
using CCAS

typealias ACASXInput InputVals
typealias ACASXOutput OutputVals

type ACASXCoordRecordIntruder
  id::Uint32
  cvc::Uint8
  vrc::Uint8
  vsb::Uint8

  ACASXCoordRecordIntruder(id::Int) = new(id,0x0,0x0,0x0)
end

type ACASXCoordRecord
  my_id::Uint32
  modes::Uint32
  equipage::Int32
  quant::Uint8
  sensitivity_index::Uint8
  protection_mode::Uint8

  intruders::Vector{ACASXCoordRecordIntruder}
end

type ACASX <: AbstractCollisionAvoidanceSystem
  my_id::Int64 #aircraft number
  max_intruders::Int64
  constants::Constants
  casShared::CASShared
  outputVals::OutputVals
  coord::AbstractCASCoord

  function ACASX(aircraft_number::Int64,quant::Int64,config_file::String,nAircraft::Int,coord::AbstractCASCoord)
    cas = new()
    cas.my_id = aircraft_number
    cas.max_intruders = nAircraft - 1
    cas.constants = Constants(quant, config_file, cas.max_intruders)
    cas.casShared = CASShared(cas.constants,CCAS.LIBCAS)

    @test cas.max_intruders == max_intruders(cas.casShared) #Will fail if library did not open properly

    cas.outputVals = OutputVals(cas.max_intruders)
    cas.coord = coord

    setRecord(cas.coord, cas.my_id,
              ACASXCoordRecord(cas.my_id,
                               cas.my_id,
                               EQUIPAGE.EQUIPAGE_ATCRBS,
                               25,
                               0x0,
                               0x0,
                               ACASXCoordRecordIntruder[ACASXCoordRecordIntruder(i)
                                                        for i=1:cas.max_intruders]))

    reset(cas.casShared)

    return cas
  end
end

getListId(list_owner_id::Integer,id::Integer) = id < list_owner_id ? id : id - 1

function step(cas::ACASX, inputVals::ACASXInput)
  #Grab from coordination object
  coord_records = CASCoordination.getAll(cas.coord)

  #Augment the input with info from coord
  #note: looping over intruders skips self
  for intruder in inputVals.intruders

    my_list_id = getListId(intruder.id,cas.my_id)
    record = coord_records[intruder.id]

    #intruder-shared
    intruder.equipage             = record.equipage
    intruder.quant                = record.quant
    intruder.sensitivity_index    = record.sensitivity_index
    intruder.protection_mode      = record.protection_mode

    #intruder-specific
    intruder_self = record.intruders[my_list_id] #self in intruders' record

    @test cas.my_id == intruder_self.id #sanity check the record

    intruder.cvc = intruder_self.cvc
    intruder.vrc = intruder_self.vrc
    intruder.vsb = intruder_self.vsb

  end

  update!(cas.casShared,inputVals,cas.outputVals) #cas.outputVals is modified in place

  my_record = getRecord(cas.coord, cas.my_id)

  #update my record in coordination object
  #intruder-shared
  my_record.sensitivity_index = cas.outputVals.sensitivity_index
  #others don't need to be update...

  #intruder-specific
  for i = 1:endof(my_record.intruders)
    my_record.intruders[i].id           = cas.outputVals.intruders[i].id
    my_record.intruders[i].cvc          = cas.outputVals.intruders[i].cvc
    my_record.intruders[i].vrc          = cas.outputVals.intruders[i].vrc
    my_record.intruders[i].vsb          = cas.outputVals.intruders[i].vsb
  end

  setRecord(cas.coord, cas.my_id, my_record) #push back to coord

  #debug:
  #if cas.outputVals.alarm && cas.outputVals.target_rate != 0.0
  #  @show cas.my_id
  #  xdump(cas.outputVals)
  #  xdump(cas.outputVals.intruders[1])
  #end

  return cas.outputVals
end

function initialize(cas::ACASX)
    reset(cas.casShared)
end

end


