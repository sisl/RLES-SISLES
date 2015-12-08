# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/21/2014

#ACASX implementation: based on CCAS interface to libCAS

module ACASX_ADD_Impl

export
    addObserver,

    ACASX_ADD,
    ACASXInput,
    ACASXOutput


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces
using ObserverImpl

using AbstractCASCoordImpl

using ACASXCommonImpl
using CASCoordination
using CASInterface

type ACASX_ADD <: AbstractCollisionAvoidanceSystem
  my_id::Int64 #aircraft number
  max_intruders::Int64
  version::String
  equipage::EQUIPAGE
  input::Input
  output::Output
  coord::AbstractCASCoord

  function ACASX_ADD(aircraft_id::Int64, quant::Int64,
                 num_aircraft::Int, coord::AbstractCASCoord, equipage::EQUIPAGE=EQUIPAGE_TCAS)
    cas = new()
    cas.my_id = aircraft_id
    cas.max_intruders = num_aircraft - 1
    cas.version = "n/a"
    cas.equipage = equipage
    cas.coord = coord
    cas.input = Input(cas.max_intruders)
    cas.output = Output(cas.max_intruders)
    setRecord(cas.coord, cas.my_id,
              ACASXCoordRecord(cas.my_id, equipage, quant, cas.max_intruders))
    reset(cas.casShared)
    return cas
  end
end

function step(cas::ACASX_ADD, input::ACASXInput)
  ACASXCommonImpl.update_from_coord!(input, cas.coord, cas.my_id)

  #call custom update here...
  #my_custom_update()

  ACASXCommonImpl.update_to_coord!(cas.coord, cas.my_id, cas.output)
  cas.input = input #keep a record
  return cas.output
end

function initialize(cas::ACASX_ADD)
  ACASXCommonImpl.initialize(cas)
end

end #module


