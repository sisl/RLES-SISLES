# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module SimpleADMImpl

export
    addObserver,

    initialize,
    step,

    initializeDynamicModel,
    simulateDynamicModel,

    SimpleADM,
    SimpleADMInitialState,
    SimpleADMOutputState,
    SimpleADMCommand


using AbstractDynamicModelImpl
using Interfaces
using ObserverImpl

using Base.Test
#using Distributions

import Interfaces.initialize
import Interfaces.step
import Interfaces.initializeDynamicModel
import Interfaces.simulateDynamicModel


type SimpleADMInitialState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    v::Float64
    psi::Float64
    h_d::Float64
end

type SimpleADMState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    v::Float64
    psi::Float64
end

type SimpleADMOutputState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    vx::Float64
    vy::Float64
    vh::Float64
end

type SimpleADMCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64
end

type SimpleADM <: AbstractDynamicModel

    state::SimpleADMState
    update::Union(SimpleADMCommand, Nothing)

    timestep::Float64
    number_of_substeps::Int

    theta_regulated::Int


    observer::Observer


    function SimpleADM(timestep = 1., number_of_substeps = 10)

        obj = new()

        obj.update = nothing
        obj.timestep = timestep
        obj.number_of_substeps = number_of_substeps
        obj.theta_regulated = 45    # degree

        obj.observer = Observer()

        return obj
    end
end


addObserver(adm::SimpleADM, f::Function) = _addObserver(adm, f)
addObserver(adm::SimpleADM, tag::String, f::Function) = _addObserver(adm, tag, f)


function initializeDynamicModel(adm::SimpleADM, state::SimpleADMInitialState)

    adm.state = SimpleADMState(state.t, state.x, state.y, state.h, state.v, state.psi)

    notifyObserver(adm, [adm.state.t, adm.state.x, adm.state.y, adm.state.h])

    vh = state.h_d
    vx = sqrt(adm.state.v^2 - vh^2) * cosd(adm.state.psi)
    vy = sqrt(adm.state.v^2 - vh^2) * sind(adm.state.psi)

    adm.update = nothing

    return SimpleADMOutputState(adm.state.t, adm.state.x, adm.state.y, adm.state.h, vx, vy, vh)
end

initialize(adm::SimpleADM, state::SimpleADMInitialState) = initializeDynamicModel(adm, state)

function simulateDynamicModel(adm::SimpleADM, update::SimpleADMCommand)

    t, x, y, h, v, psi = adm.state.t, adm.state.x, adm.state.y, adm.state.h, adm.state.v, adm.state.psi

    if adm.update == nothing
        t_prev, v_d_prev, h_d_prev, psi_d_prev = t, update.v_d, update.h_d, update.psi_d
    else
        t_prev, v_d_prev, h_d_prev, psi_d_prev = adm.update.t + adm.timestep, adm.update.v_d, adm.update.h_d, adm.update.psi_d
    end
    t_next, v_d_next, h_d_next, psi_d_next = update.t + adm.timestep, update.v_d, update.h_d, update.psi_d

    #@test t == t_prev

    dt = (t_next - t_prev) / adm.number_of_substeps

    t_sim = t

    x_n = x
    y_n = y
    h_n = h
    v_n = v
    psi_n = psi

    v_d = v_d_prev
    h_d = h_d_prev
    psi_d = psi_d_prev

    #println("====")
    #println(dt)
    #println(t_sim)
    #println(t, " ", x_n, " ", y_n, " ", h_n, " ", v_n, " ", psi_n)
    #println(t_prev, " ", v_d_prev, " ", h_d_prev, " ", psi_d_prev)
    #println(t_next, " ", v_d_next, " ", h_d_next, " ", psi_d_next)
    #println("----")

    for i = 1:adm.number_of_substeps
        t_sim += dt
        #println(t_sim)

        if abs(h_d) > abs(v_n) * sind(adm.theta_regulated)
            h_d = sign(h_d) * abs(v_n) * sind(adm.theta_regulated)
            #println(t_sim, ": regulated")
        end

        psi_n += psi_d * dt # deg
        x_n += sqrt(v_n^2 - h_d^2) * cosd(psi_n) * dt
        y_n += sqrt(v_n^2 - h_d^2) * sind(psi_n) * dt
        h_n += h_d * dt     # ft
        v_n += v_d * dt     # ft/s
        #println(x_n, " ", y_n, " ", h_n, " ", v_n, " ", psi_n)

        notifyObserver(adm, [t_sim, x_n, y_n, h_n])

        # linear control
        dv_d = (v_d_next - v_d_prev) * (t_sim - t_prev)
        dh_d = (h_d_next - h_d_prev) * (t_sim - t_prev)
        dpsi_d = (psi_d_next - psi_d_prev) * (t_sim - t_prev)

        # sigmoid control
        #CMD_DIST = Normal(0.3, 0.07)
        #dv_d = (v_d_next - v_d_prev) * cdf(CMD_DIST, (t_sim - t_prev))
        #dh_d = (h_d_next - h_d_prev) * cdf(CMD_DIST, (t_sim - t_prev))
        #dpsi_d = (psi_d_next - psi_d_prev) * cdf(CMD_DIST, (t_sim - t_prev))

        v_d = v_d_prev + dv_d
        h_d = h_d_prev + dh_d
        psi_d = psi_d_prev + dpsi_d

        #println(v_d, " ", h_d, " ", psi_d)
    end

    #@test_approx_eq_eps t_sim t_next 0.001
    #@test_approx_eq_eps v_d v_d_next 0.001
    #@test_approx_eq_eps h_d h_d_next 0.001
    #@test_approx_eq_eps psi_d psi_d_next 0.001

    adm.state = SimpleADMState(t_next, x_n, y_n, h_n, v_n, psi_n)
    adm.update = update
end

function step(adm::SimpleADM, update::SimpleADMCommand)

    t =  copy(adm.state.t)
    x =  copy(adm.state.x)
    y =  copy(adm.state.y)
    h =  copy(adm.state.h)

    simulateDynamicModel(adm, update)

    t_n =  adm.state.t
    x_n =  adm.state.x
    y_n =  adm.state.y
    h_n =  adm.state.h

    vx_n = (x_n - x) / (t_n - t)
    vy_n = (y_n - y) / (t_n - t)
    vh_n = (h_n - h) / (t_n - t)

    # instantaneous velocities
    #vh_n = adm.update.h_d
    #vx_n = sqrt(adm.state.v^2 - vh_n^2) * cosd(adm.state.psi)
    #vy_n = sqrt(adm.state.v^2 - vh_n^2) * sind(adm.state.psi)

    return SimpleADMOutputState(t_n, x_n, y_n, h_n, vx_n, vy_n, vh_n)
end

end


