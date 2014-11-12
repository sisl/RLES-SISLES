# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014

#Piggyback and add coordination to SimpleTCAS

module CoordSimpleTCASImpl

export
    AddObserver,

    initialize,
    step,

    testThreat,
    selectRA,

    CoordSimpleTCAS,
    CoordSimpleTCASInput,
    CoordSimpleTCASResolutionAdvisory


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces
using ObserverImpl

using Util
using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractCollisionAvoidanceSystemInterfaces.testThreat
import AbstractCollisionAvoidanceSystemInterfaces.selectRA

using AbstractCASCoordImpl

using SimpleTCASImpl

type CoordSimpleTCAS <: AbstractCollisionAvoidanceSystem
  aircraft_number::Int64
  simpleCAS::SimpleTCAS
  coordination::AbstractCASCoord

  CoordSimpleTCAS(aircraft_number::Int,coordination::AbstractCASCoord) = CoordSimpleTCAS(aircraft_number,SimpleTCAS(),coordination)
end

addObserver(cas::CoordSimpleTCAS, f::Function) = _addObserver(cas, f)
addObserver(cas::CoordSimpleTCAS, tag::String, f::Function) = _addObserver(cas, tag, f)

testThreat(cas::CoordSimpleTCAS, input::SimpleTCASInput) = SimpleTCAS.testThreat(cas.simpleCAS, input)

function selectRA(cas::CoordSimpleTCAS, input::SimpleTCASInput)

  #Make a two aircraft assumption for now
  if cas.aircraft_number == 1
    intruder_number = 2
  else
    intruder_number = 1
  end

  intruder_record = getRecord(cas.coordination,intruder_number)

  if intruder_record.RA != nothing
    h_d = intruder_record.RA.h_d < 0 ? 1500 / 60 : -1500 / 60  #Do opposite sense, default strength
  else
    h_d = selectRA(cas.simpleCAS, input)
  end

  return h_d
end

function step(cas::CoordSimpleTCAS, input::SimpleTCASInput)

  if cas.simpleCAS.b_TCAS_activated == false
    cas.simpleCAS.b_TCAS_activated = testThreat(cas, input)

    if cas.simpleCAS.b_TCAS_activated
      cas.simpleCAS.RA = SimpleTCASResolutionAdvisory(selectRA(cas, input))
      updateRecord(cas.coordination,aircraft_number,cas.simpleCAS.RA)
    end
  end

  return cas.simpleCAS.RA
end

function initialize(cas::CoordSimpleTCAS)
    initialize(cas.simpleCAS)
end

end


