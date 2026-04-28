# YAPR — PX4 Control Allocation for Thrust-Vectoring Quadrotors

Custom PX4 firmware implementing Moore-Penrose pseudoinverse control allocation 
for thrust-vectoring quadrotors with independently tiltable rotors. Developed as MASc 
research at Toronto Metropolitan University (2026).

## What This Is

Standard quadrotors are underactuated, they can only control 4 degrees of freedom by 
varying rotor speeds. This firmware enables full 6-DOF force and torque control by 
adding servo-driven rotor gimbal mechanisms that tilt each rotor independently, enabling 
decoupled position and attitude control.

## How It Works

The core contribution is a custom control allocation framework integrated natively within 
PX4's control allocator module:

- **ActuatorEffectivenessTM** — custom subclass that builds the actuator effectiveness 
  matrix for single- and dual-axis tilt mechanisms, mapping rotor-frame
  to body-frame for force/torque contributions
- **ControlAllocationTM** — custom subclass implementing Moore-Penrose pseudoinverse 
  allocation, mapping desired 6-DOF body-frame forces and torques to motor thrusts and 
  servo tilt commands (PWM duty output)
- **Parallel Position-Attitude Control** — modified control structure enabling simultaneous 
  independent position and attitude tracking, bypassing the standard inner-outer loop 
  dependency

## Key Features

- Full 6-DOF thrust vectoring on a standard X-configuration quadrotor
- Native integration within PX4 architecture, compatible with existing modules and 
  autonomy stacks
- Supports both single-axis and dual-axis tilt configurations via airframe parameters
- Rapid switching between thrust-vectoring and conventional quadrotor modes via parameters
- Validated through live flight testing on a modified Holybro X500 V2

## Hardware Platform

- **Frame:** Modified Holybro X500 V2
- **Autopilot:** Pixhawk 6C
- **Actuators:** Custom 3D-printed rotor gimbal mechanisms with servo-driven tilt
- **GCS:** QGroundControl with MAVLink telemetry
