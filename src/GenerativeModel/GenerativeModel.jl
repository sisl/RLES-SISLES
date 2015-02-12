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
    SimpleTCAS_EvE,

    ACASX_EvE_params,
    ACASX_EvE,

    ACASX_Multi_params,
    ACASX_Multi


using AbstractGenerativeModelImpl

include("common/common.jl")

using SimpleTCAS_EvU_Impl

using SimpleTCAS_EvE_Impl

using ACASX_EvE_Impl

using ACASX_Multi_Impl

end


