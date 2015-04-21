# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/21/2014

#ACASX implementation: based on CCAS interface to libCAS

module ACASXImpl

export
    addObserver,

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
import CCAS.reset!

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
  version::String
  inputVals::InputVals
  outputVals::OutputVals
  coord::AbstractCASCoord

  function ACASX(aircraft_id::Int64, libcas::String, config_file::String, quant::Int64, num_aircraft::Int, coord::AbstractCASCoord)
    cas = new()
    cas.my_id = aircraft_id
    cas.max_intruders = num_aircraft - 1
    cas.constants = Constants(quant, config_file, cas.max_intruders)
    cas.casShared = CASShared(libcas, cas.constants)
    cas.version = version(cas.casShared)

    @test cas.max_intruders == max_intruders(cas.casShared) #Will fail if library did not open properly

    cas.coord = coord

    cas.inputVals = InputVals(cas.max_intruders)
    cas.outputVals = OutputVals(cas.max_intruders)
    setRecord(cas.coord, cas.my_id,
              ACASXCoordRecord(cas.my_id,
                               cas.my_id,
                               EQUIPAGE.EQUIPAGE_TCAS,
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
  #others (equipage,quant,protection_mode) don't need to be updated...

  #intruder-specific
  for i = 1:endof(my_record.intruders)
    my_record.intruders[i].id           = cas.outputVals.intruders[i].id
    my_record.intruders[i].cvc          = cas.outputVals.intruders[i].cvc
    my_record.intruders[i].vrc          = cas.outputVals.intruders[i].vrc
    my_record.intruders[i].vsb          = cas.outputVals.intruders[i].vsb
  end

  setRecord(cas.coord, cas.my_id, my_record) #push back to coord

  cas.inputVals = inputVals #keep a record

  return cas.outputVals
end

function initialize(cas::ACASX)

  reset!(cas.inputVals)
  reset!(cas.outputVals)
  reset!(cas,getRecord(cas.coord, cas.my_id))
  reset(cas.casShared)
end

function reset!(cas::ACASX,rec::ACASXCoordRecord)
  rec.my_id = cas.my_id
  rec.modes = cas.my_id
  rec.equipage = EQUIPAGE.EQUIPAGE_TCAS
  rec.quant = 25
  rec.sensitivity_index = 0x0
  rec.protection_mode = 0x0

  for i=1:endof(rec.intruders)
    rec.intruders[i].id = uint32(i)
    rec.intruders[i].cvc = 0x0
    rec.intruders[i].vrc = 0x0
    rec.intruders[i].vsb = 0x0
  end
end

end


