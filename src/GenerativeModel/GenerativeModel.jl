# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014


module GenerativeModel

export
    AbstractGenerativeModel,
    initialize,
    step,
    get,
    isEndState,

    SimpleTCAS_EvU_params,
    SimpleTCAS_EvU,

    SimpleTCAS_EvE_params,
    SimpleTCAS_EvE


using AbstractGenerativeModelImpl

include("common/common.jl")

using SimpleTCAS_EvU_Impl

using SimpleTCAS_EvE_Impl

end


