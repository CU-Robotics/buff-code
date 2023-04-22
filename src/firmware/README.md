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

