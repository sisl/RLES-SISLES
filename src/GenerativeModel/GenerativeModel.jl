# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014


module GenerativeModel

export
    AbstractGenerativeModel,
    initialize,
    step,
    get,

    SimpleTCAS_EvU_params,
    SimpleTCAS_EvU


using AbstractGenerativeModelImpl

using SimpleTCAS_EvU_Impl

end


