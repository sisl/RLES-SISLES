# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 04/14/2015
# Implements Lincoln Lab scripted dynamics model

module LLADMImpl

export
addObserver,

initialize,
step,

initializeDynamicModel,
simulateDynamicModel,

LLADM,
LLADMOutputState,
LLADMCommand


using AbstractDynamicModelImpl
using AbstractDynamicModelInterfaces
using CommonInterfaces
using ObserverImpl

using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.step
import AbstractDynamicModelInterfaces.initializeDynamicModel
import AbstractDynamicModelInterfaces.simulateDynamicModel

# Note: Angles are all in radians in this module unless otherwise indicated
# CASSATT_TIMING is false

type LLADMState

  t::Float64       #time (s)
  v::Float64       #airspeed (ft/s)
  N::Float64       #position north (ft)
  E::Float64       #position east (ft)
  h::Float64       #position altitude (ft)
  psi::Float64     #heading angle (rad)
  theta::Float64   #pitch angle (rad)
  phi::Float64     #bank angle (rad)
  a::Float64       #airspeed acceleration (ft/s^2)
end

type LLADMOutputState #mitcas state in scripted_dynamics.h

  t::Float64
  x::Float64
  y::Float64
  h::Float64
  vx::Float64   #DX in scripted_dynamics.h
  vy::Float64   #DY in scripted_dynamics.h
  vh::Float64   #DH in scripted_dynamics.h
  psi::Float64
  theta::Float64
  phi::Float64
end

LLADMOutputState() = LLADMOutputState(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)

type LLADMCommand

  t::Float64        #time of command (s)
  hdot::Float64     #commanded vertical rate (ft/s)
  psidot::Float64   #commanded turn rate (rad/s)
  a::Float64        #commanded acceleration (ft/s^2)
end

immutable LLADMConsts
  _dt::Float64           # minor time step
  _K::Float64            # integrator gain
  _g::Float64            # acceleration of gravity (ft/s^2)
  _qmax::Float64         # as in CASSATT (rad/s)
  _rmax::Float64         # as in CASSATT GA_psidotMAX = 1e6
  _phi_max::Float64
  _phidotmax::Float64
  _v_high::Float64       # airspeed limits
  _v_low::Float64
  LIMIT_DDH::Bool
  _min_vertical_accel::Float64
  _max_vertical_accel::Float64
end

type LLADM <: AbstractDynamicModel

  state::LLADMState
  update::Union(LLADMCommand, Nothing)

  timestep::Float64
  number_of_substeps::Float64

  llconsts::LLADMConsts

  output_state::LLADMOutputState

  observer::Observer

  function LLADM(; timestep::Float64 = 1.0, #major time step
                 _dt::Float64 = 0.1, #minor time step
                 _K::Float64 = 1.0,
                 _g::Float64 = 32.2,
                 _qmax::Float64 = deg2rad(3.0),
                 _rmax::Float64 = 1000000.0,
                 _phi_max::Float64 = deg2rad(75.0),
                 _phidotmax::Float64 = 0.524,
                 _v_high::Float64 = 100000,
                 _v_low::Float64 = 1.7,
                 LIMIT_DDH::Bool = false,
                 _min_vertical_accel::Float64 = -0.5*32.2,
                 _max_vertical_accel::Float64 = 0.5*32.2,
                 )

    obj = new()

    obj.update = nothing
    obj.timestep = timestep

    obj.llconsts = LLADMConsts(_dt,
                               _K,
                               _g,
                               _qmax,
                               _rmax,
                               _phi_max,
                               _phidotmax,
                               _v_high,
                               _v_low,
                               _dh_min
                               LIMIT_DDH,
                               _min_vertical_accel,
                               _max_vertical_accel
                               )

    adm.output_state = LLADMOutputState()

    obj.observer = Observer()

    return obj
  end
end

addObserver(adm::LLADM, f::Function) = _addObserver(adm, f)
addObserver(adm::LLADM, tag::String, f::Function) = _addObserver(adm, tag, f)

initialize(adm::LLADM, state) = initialize(adm,convert(LLADMState, state))

function initialize(adm::LLADM, state::LLADMState)
  adm.state = state

  # Intialize the mitcas state
  # Note: x is E and y is N.  See assignments in scripted_dynamics.h
  adm.output_state.h     = state.h
  adm.output_state.vh    = sin(state.theta) * state.v
  adm.output_state.x     = state.E
  adm.output_state.vx    = sin(state.psi) * cos(state.theta) * state.v
  adm.output_state.y     = state.N
  adm.output_state.vy    = cos(state.psi) * cos(state.theta) * state.v
  adm.output_state.psi   = state.psi
  adm.output_state.theta = state.theta
  adm.output_state.phi   = state.phi

  adm.update = nothing

  return adm.output_state
end

# Mirrors state_update() from scripted_dynamics.h
function state_update(state::LLADMState, ctrl::LLADMCommand, hddcmd::Float64, dt::Float64)
  # Inputs:
  # state = current state (adm.state)
  # ctrl_update = current command (update::LLADMCommand)
  # hddcmd = vertical acceleration to apply
  # dt = time step

  const s_theta = sin(state.theta)
  const c_theta = cos(state.theta)
  const t_theta = tan(state.theta)
  const s_phi = sin(state.phi)
  const c_phi = cos(state.phi)
  const s_psi = sin(state.psi)
  const c_psi = cos(state.psi)

  # save v and saturated v
  const v = state.v
  const v_saturated = max(v,1.0)

  # compute and saturate q
  q = saturate(1.0 / (v_saturated * c_phi) * (hddcmd / c_theta + _g * c_theta * s_phi * s_phi - ctrl.a * t_theta), -_qmax, _qmax)

  # compute and saturate r
  r = saturate(_g * s_phi * c_theta / v_saturated, -_rmax, _rmax)

  # sometimes the aircraft bank angle is so large that it would prevent reaching desired h_dot,
  # so we dynamically saturate the bank angle so that h_dot is always acheived
  # compute phimax (in the Plopt/compute q block)
  hdd_cmd_phi = min(hddcmd, v_saturated * _qmax * c_phi * c_theta)

  # calculate discriminant
  sqrt_arg = v_saturated * v_saturated * _qmax * _qmax - 4 * _g * ctrl.a * s_theta + 4 * _g * hdd_cmd_phi + 4 * _g * _g * c_theta * c_theta;
  phi_max_2 = 0

  if sqrt_arg < 0
    phi_max_2 = 10000
  else
    # calculate cos(phi)
    cphi1 = (sqrt(sqrt_arg) - v_saturated * _qmax) / (2 * _g * c_theta)

    if fabs(cphi1) < 1
      phi_max_2 = acos(cphi1) * 0.98 # add a small buffer to prevent jittering
    end
  end

  phimax = min(_phi_max, phi_max_2)

  # compute and saturate p
  psidot_if_no_change = (q * s_phi + r * c_phi) / c_theta
  dpsidot = ctrl.psidot - psidot_if_no_change
  p = saturate(20.0 * dpsidot, -_phidotmax, _phidotmax) # 20 is user tuned parameter from CASSATT

  # limit max bank angle
  if (state.phi + p * dt > phimax)
    p = (phimax - state.phi) / dt

    if (state.phi + p * dt < -phimax)
      p = (-phimax - state.phi) / dt

      state.v += dt * (ctrl.a)
      state.N += dt * (v * c_theta * c_psi)
      state.E += dt * (v * c_theta * s_psi)
      state.h += dt * (v * s_theta)
      state.psi += dt * (q * s_phi / c_theta + r * c_phi / c_theta)
      state.theta += dt * (q * c_phi - r * s_phi)
      state.phi += dt * (p + q * s_phi * t_theta + r * c_phi * t_theta)

      state.v = saturate(state.v, 1.7, _v_high - 0.000001)
    end
  end
end

step(adm::LLADM, ctrl) = step(adm,convert(LLADMCommand, ctrl))

function step(adm::LLADM, ctrl::LLADMCommand)

  state = adm.state
  C = adm.llconsts

  ddh = (ctrl.hdot - state.vh) / timestep

  for i = 1:int64(round(timestep/C._dt))
    substep(state,ctrl,_dh_min,_dh_max,_ddh)
  end

  # calculate the new mitcas state vars
  # Note: x is E and y is N.  See assignments in scripted_dynamics.h
  adm.output_state.h     = state.h
  adm.output_state.vh    = sin(state.theta) * state.v
  adm.output_state.x     = state.E
  adm.output_state.vx    = sin(state.psi) * cos(state.theta) * state.v
  adm.output_state.y     = state.N
  adm.output_state.vy    = cos(state.psi) * cos(state.theta) * state.v
  adm.output_state.psi   = state.psi
  adm.output_state.theta = state.theta
  adm.output_state.phi   = state.phi

  return adm.output_state
end

function substep(state,ctrl,dh_min,dh_max,ddh)

  dh = state.v * sin(state.theta)

  ddh2apply = resolve()

  if C.LIMIT_DDH
    ddh2apply = saturate(ddh2apply, _min_vertical_accel, _max_vertical_accel)
  end

  state_update(state, ctrl, ddh2apply, C._dt)
end

function saturate(x::Float64, limit_min::Float64, limit_max::Float64)
  @test limit_min <= limit_max

  return min(max(x,limit_min),limit_max)
end
