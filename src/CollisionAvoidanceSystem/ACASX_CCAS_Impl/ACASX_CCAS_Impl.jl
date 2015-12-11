# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/21/2014

#ACASX implementation: based on CCAS interface to libCAS

module ACASX_CCAS_Impl

export
    addObserver,

    initialize,
    step,

    ACASX_CCAS,
    ACASXInput,
    ACASXOutput


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces
using ObserverImpl

import CommonInterfaces.initialize
import CommonInterfaces.step

using AbstractCASCoordImpl

using ACASXCommonImpl
using CASCoordination
using CASInterface
using CCAS

type ACASX_CCAS <: AbstractCollisionAvoidanceSystem
  my_id::Int64 #aircraft number
  max_intruders::Int64
  quant::Int64
  constants::Constants
  casShared::CASShared
  version::String
  equipage::EQUIPAGE
  input::Input
  output::Output
  coord::AbstractCASCoord

  function ACASX_CCAS(aircraft_id::Int64, libcas::String, config_file::String,
                 num_aircraft::Int, coord::AbstractCASCoord, equipage::EQUIPAGE=EQUIPAGE_TCAS)
    cas = new()
    cas.my_id = aircraft_id
    cas.max_intruders = num_aircraft - 1
    cas.quant = equipage == EQUIPAGE_TCAS ? 25 : 100
    cas.constants = Constants(cas.quant, config_file, cas.max_intruders)
    cas.casShared = CASShared(libcas, cas.constants)
    cas.version = version(cas.casShared)
    cas.equipage = equipage

    @assert cas.max_intruders == max_intruders(cas.casShared) #Will fail if library did not open properly

    cas.coord = coord
    cas.input = Input(cas.max_intruders)
    cas.output = Output(cas.max_intruders)
    setRecord(cas.coord, cas.my_id,
              ACASXCoordRecord(cas.my_id, equipage, cas.quant, cas.max_intruders))
    reset(cas.casShared)
    return cas
  end
end

function step(cas::ACASX_CCAS, input::ACASXInput)
  ACASXCommonImpl.update_from_coord!(input, cas.coord, cas.my_id)
  if cas.equipage == EQUIPAGE_TCAS
    update!(cas.casShared, input, cas.output) #cas.output is modified in place
    ACASXCommonImpl.update_to_coord!(cas.coord, cas.my_id, cas.output)
  end

  cas.input = input #keep a record
  return cas.output
end

function initialize(cas::ACASX_CCAS)
  ACASXCommonImpl.initialize(cas)
  reset(cas.casShared)
end

end #module


