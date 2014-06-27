# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


module SimpleTCASImpl

export
    initialize,
    step,

    testThreat,
    selectRA,

    SimpleTCAS,
    SimpleTCASInput,
    SimpleTCASResolutionAdvisory


using AbstractCollisionAvoidanceSystemImpl
using Interfaces

using Util

import Interfaces.step
import Interfaces.testThreat
import Interfaces.selectRA


type SimpleTCASInput

    t::Float64

    r::Float64
    r_d::Float64

    a::Float64
    a_d::Float64

    h::Vector{Float64}
    h_d::Vector{Float64}
end

type SimpleTCASResolutionAdvisory

    h_d::Float64
end

type SimpleTCAS <: AbstractCollisionAvoidanceSystem

    b_TCAS_activated::Bool
    RA::Union(SimpleTCASResolutionAdvisory, Nothing)


    function SimpleTCAS()

        obj = new()

        obj.b_TCAS_activated = false
        obj.RA = nothing

        return obj
    end
end


function testThreat(cas::SimpleTCAS, input::SimpleTCASInput)

    b_TCAS_activated = false

    r = input.r
    r_dot = input.r_d

    a = input.a
    a_dot = input.a_d

    sl, tau, dmod, zthr, alim = simple_TCAS_thresholds(input.h[1])

    if ((r_dot < 0 && -(r - dmod) / r_dot <= tau) || r < dmod) && ((a_dot < 0 && -a / a_dot <= tau) || a <= zthr)
        if false
            @printf(STDERR, "%s\n", input.t)
            @printf(STDERR, "%s\n", outputGFormatString(input.h[1]))
            @printf(STDERR, "%s\n", sl)
            if r_dot < 0 && -(r - dmod) / r_dot <= tau
                @printf(STDERR, "H1 ")
            end
            if r < dmod
                @printf(STDERR, "H2 ")
            end
            if a_dot < 0 && -a / a_dot <= tau
                @printf(STDERR, "V1 ")
            end
            if a <= zthr
                @printf(STDERR, "V2 ")
            end
            @printf(STDERR, "\n")
        end

        b_TCAS_activated = true
    end

    return b_TCAS_activated
end

function selectRA(cas::SimpleTCAS, input::SimpleTCASInput)

    a = input.a
    a_dot = input.a_d

    h1 = input.h[1]
    h2 = input.h[2]

    h1_dot = input.h_d[1]
    h2_dot = input.h_d[2]

    sl, tau, dmod, zthr, alim = simple_TCAS_thresholds(input.h[1])

    ascend_cross = false
    ascend_dist = 0
    ascend_alim = false

    descend_cross = false
    descend_dist = 0
    descend_alim = false

    t_ = 40 - input.t

    h_dot_ascend = 1500 / 60

    h1_cpa = h1 + h_dot_ascend * t_
    h2_cpa = h2 + h2_dot * t_

    if (h1 <= h2 && h1_cpa >= h2_cpa) || (h1 >= h2 && h1_cpa <= h2_cpa)
        ascend_cross = true
    end

    ascend_dist = abs(h1_cpa - h2_cpa)
    if ascend_dist >= alim
        ascend_alim = true
    end

    h1_dot_descend = - 1500 / 60

    h1_cpa = h1 + h1_dot_descend * t_
    h2_cpa = h2 + h2_dot * t_

    if (h1 <= h2 && h1_cpa >= h2_cpa) || (h1 >= h2 && h1_cpa <= h2_cpa)
        descend_cross = true
    end

    descend_dist = abs(h1_cpa - h2_cpa)
    if descend_dist >= alim
        descend_alim = true
    end

    if ascend_cross
        if descend_alim
            resolution_advisory = - 1500 / 60
        elseif ascend_alim
            resolution_advisory = 1500 / 60
        else
            resolution_advisory = - 1500 / 60
        end
    elseif descend_cross
        if ascend_alim
            resolution_advisory = 1500 / 60
        elseif descend_alim
            resolution_advisory = - 1500 / 60
        else
            resolution_advisory = 1500 / 60
        end
    else
        if descend_alim && ascend_alim
            if descend_dist < ascend_dist
                resolution_advisory = - 1500 / 60
            else
                resolution_advisory = 1500 / 60
            end
        elseif descend_alim
            resolution_advisory = - 1500 / 60
        elseif ascend_alim
            resolution_advisory = 1500 / 60
        else
            if descend_dist > ascend_dist
                resolution_advisory = - 1500 / 60
            else
                resolution_advisory = 1500 / 60
            end
        end
    end

    if false
        @printf(STDERR, "%s\n", ascend_cross)
        @printf(STDERR, "%s\n", outputGFormatString(ascend_dist))
        @printf(STDERR, "%s\n", ascend_alim)
        @printf(STDERR, "%s\n", descend_cross)
        @printf(STDERR, "%s\n", outputGFormatString(descend_dist))
        @printf(STDERR, "%s\n", descend_alim)
        @printf(STDERR, "%s\n", resolution_advisory)
        @printf(STDERR, "\n")
    end

    return resolution_advisory
end

function step(cas::SimpleTCAS, input::SimpleTCASInput)
    
    if cas.b_TCAS_activated == false
        cas.b_TCAS_activated = testThreat(cas, input)

        if cas.b_TCAS_activated
            cas.RA = SimpleTCASResolutionAdvisory(selectRA(cas, input))
        end
    end

    if cas.RA == nothing
        return nothing
    else
        return cas.RA
    end
end

function initialize(cas::SimpleTCAS)
    cas.b_TCAS_activated = false
    cas.RA = nothing
end


function simple_TCAS_thresholds(h)

    if h < 1000 
        sl = 2
        tau = 0
        dmod = 0
        zthr = 0
        alim = 0
    elseif h < 2350
        sl = 3
        tau = 15
        dmod = 0.2 * 6076.12
        zthr = 600
        alim = 300
    elseif h < 5000
        sl = 4
        tau = 20
        dmod = 0.35 * 6076.12
        zthr = 600
        alim = 300
    elseif h < 10000
        sl = 5
        tau = 25
        dmod = 0.55 * 6076.12
        zthr = 600
        alim = 350
    elseif h < 20000
        sl = 6
        tau = 30
        dmod = 0.80 * 6076.12
        zthr = 600
        alim = 400
    elseif h < 42000
        sl = 7
        tau = 35
        dmod = 1.10 * 6076.12
        zthr = 700
        alim = 600
    else
        sl = 7
        tau = 35
        dmod = 1.10 * 6076.12
        zthr = 800
        alim = 700
    end

    return sl, tau, dmod, zthr, alim
end

end


