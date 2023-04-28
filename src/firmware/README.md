## Build firmware
        make

## Robot States

Position: [X, Y, Theta, Pitch, Yaw, Feeder, Shooter] (from estimation)

Velocity: [dX, dY, dTheta, dPitch, dYaw, dFeeder, dShooter] (from estimation)

DR16 (reference): [dXr, dYr, dThetar, dPitchr, dYawr, dFeederr, dShooterr] (from user/autonomy)

## Motor States

Power limited: [Position, Velocity, control_authority]

Pendulum: [Position, Velocity, sin(position)]

coming soon

Dynamic : [Position, Velocity, X3, X4, ...] # X3 & X4 define states, there is no implementation to update the states (config should come from hid)

## Control

The controller uses a kinematic definition of a chassis and turret to relate an reference robot state to a reference state of each motor. This 'decoupling of states' allows for easier and more efficient controller synthesis (it's easy and kinda safe to do experimentally and numerically). Robust Control methods could be higher performing, but the control architecture would make manully tuning gains difficult.

Input to the controller is a robot state (defined above) and outputs are unscaled motor control values (-1:1).

The controller has two layers; state (robot) level and motor level. There are many motor controllers, there is only one robot controller. The robot controller uses the kinematics to relate an input robot state to a set of motor states. The motor controllers will track the set of motor state references.

The robot controller also calculates an estimate of the robot's state using the forward kinematics. The estimation will produce multiple robot states (2 velocity, 2 position). 

## Timing

timing.h provides 10 global timers that can be started, read and stopped by index.

        timer_set(index)

will start a timer at index (0:9)

        uint32_t duration_us = timer_info_us(index)
        uint32_t duration_ms = timer_info_ms(index)

will query the timer

        timer_wait_us(index, duration_us)
        timer_wait_ms(index, duration_ms)

will wait for the timer at index to reach the duration

Currently used indices

0: master loop timer

1: dr16 watchdog

2: motor controller reference updates

3: HID packet timer (lifetime counter)

## Firmware status guide (terminology)

LED blinking = HID connected

| Status       | Teensy Powered | Teensy configured | Motors Powered | HID Connected |
| ------------ | -------------- | ----------------- | -------------- | ------------- |
| **Dead/off** | Yes/No         | No                | No             | No            |
| **Active**   | Yes            | No                | Yes            | No            |
| **Alive**    | Yes            | Yes               | Yes            | Yes/No        |

# Pipeline Plumbing TODO's

### DR16 example
 - update to print and show safety switch value in serial console (use tycmd)

### Estimators
