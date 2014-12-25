# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


module CollisionAvoidanceSystem

export
    AbstractCollisionAvoidanceSystem,

    initialize,
    step,

    testThreat,
    selectRA,

    SimpleTCAS,
    SimpleTCASInput,
    SimpleTCASResolutionAdvisory,

    CoordSimpleTCAS,

    ACASX,
    ACASXInput,
    ACASXOutput


using AbstractCollisionAvoidanceSystemImpl

using SimpleTCASImpl

using CoordSimpleTCASImpl

using ACASXImpl

end


