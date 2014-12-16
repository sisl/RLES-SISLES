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

type ACASXCoordRecord <: AbstractCoordRecord
  id::Uint32
  modes::Uint32
  cvc::Uint8
  vrc::Uint8
  vsb::Uint8
  equipage::Int32
  quant::Uint8
  sensitivity_index::Uint8
  protection_mode::Uint8
end

type ACASXCoord <: AbstractCASCoord
  my_id::Int64
  intruders::ACASXCoordRecord
end

type ACASX <: AbstractCollisionAvoidanceSystem
  my_id::Int64 #aircraft number
  max_intruders::Int64
  constants::Constants
  casShared::CASShared
  outputVals::OutputVals
  coord::AbstractCASCoord

  function ACASX(quant::Int64,config_file::String,nAircraft::Int,coord::AbstractCASCoord)
    obj = new()
    obj.max_intruders = nAircraft - 1
    obj.constants = Constants(quant, config_file, obj.max_intruders)
    obj.casShared = CASShared(obj.constants,CCAS.LIBCAS)

    @test obj.max_intruders == max_intruders(obj.casShared) #Will fail if library did not open properly

    obj.outputVals = OutputVals(obj.max_intruders)
    obj.coord = coord

    reset(obj.casShared)

    return obj
  end
end

function step(cas::ACASX, inputVals::ACASXInput)

  #Grab from coordination object
  coord_records = coord.getAll()

  for (i,record) in enumerate(coord_records)
    if i != cas.my_id
      #find my corresponding record
      inputVals.intruders[intr_i].cvc = record.intruder[myid].cvc
      inputVals.intruders[intr_i].vrc = record.intruder[myid].vrc
      inputVals.intruders[intr_i].vsb = record.intruder[myid].vsb
      inputVals.intruders[intr_i].equipage = record.intruder[myid].equipage
      inputVals.intruders[intr_i].quant = record.intruder[myid].quant
      inputVals.intruders[intr_i].sensitivity_index = record.intruder[myid].sensitivity_index
      inputVals.intruders[intr_i].protection_mode = record.intruder[myid].protection_mode
    end
  end

  update!(cas.casShared,inputVals,cas.outputVals)

  my_record = coord.getRecord(coord, my_id)

  #update coordination object
  for i = 1:endof(coord_records)
    my_record.intruders[i].id     = outputVals.intruders[i].id
    my_record.intruders[i].modes  = outputVals.intruders[i].modes
    my_record.intruders[i].cvc    = outputVals.intruders[i].cvc
    my_record.intruders[i].vrc    = outputVals.intruders[i].vrc
    my_record.intruders[i].vsb    = outputVals.intruders[i].vsb
    my_record.intruders[i].equipage   = outputVals.intruders[i].equipage
    my_record.intruders[i].quant      = outputVals.intruders[i].quant
    my_record.intruders[i].sensitivity_index   = outputVals.intruders[i].sensitivity_index
    my_record.intruders[i].protection_mode     = outputVals.intruders[i].protection_mode
  end

  myrecord = ACASXCoord(my_id,intruders)
  setrecord(coord, my_id, coord_obj)

  return cas.outputVals
end

function initialize(cas::ACASX)
    reset(cas.casShared)
end

end


