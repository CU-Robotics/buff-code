# Robot Descriptions
Yaml files containing the nodes and data that make up a robot. Almost a rip off of roslaunch

## Adding a robot

1. Modify `robots.yaml` to have the IP and type of your robot
    
    IP_ADDRESS:
      'ROBOT_TYPE'

## Setting a machine's default robot

1. Change `self.txt` to have the name of the robot

## Adding a robot definition

1. Copy penguin into a directory with the name of your robot
2. Swap the urdf for your own
3. Modify the nodes.yaml \
    a) add the buffpy executable and ros nodes you want to run \
    b) configure sensor buffers and configuration data to send (not super well supported) \
    c) Set the motor_index to a list of strings identifying the motors 
    
        motor_index:
         - fly_wheel1    # V0
         - fly_wheel2    # V1
         - feeder        # V2
          
    d) Set the CAN configuration data so each motors config has the same index as its name in the motor_index
    
       motor_can_index:
         - [2, 1, 8]     # Sets CAN config for V0
         - [2, 1, 3]     # Sets CAN config for V1
         - [2, 0, 1]     # Sets CAN config for V2
        
    e) update the inverse kinematics to relate the robot state to the motors
       
       # Setting the kinematics to relate the shooters to the constant from the dr16 and the feeder to the wheel
       # The kinematics define how these inputs relate to motors:
       # [Vx, Vy, Omega, dPitch, dYaw, dGamma, dShooter] -> [V0, V1, V2]
       # [l_stick_x, l_stick_y, r_switch, r_stick_y, r_stick_x, wheel, COMP_MODE_CONST] -> [fly_wheel1, fly_wheel2, feeder]
       # [V0, V1, V2] defined by the motor_index
       inverse_kinematics: # (N_controller_inputs X N_motors) = (N_robot_states X N_motors)
         - [  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,   -1.0] # V0 = -COMP_MODE_CONST
         - [  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1.0] # V1 =  COMP_MODE_CONST
         - [  0.0,    0.0,    0.0,    0.0,    0.0,   -1.0,    0.0] # V2 =  -wheel
    f) Update the motor limits by running the system and vizualizing the feedback (see firmware Safety test) \
    g) Set the gains conservatively and run the system then make them more aggressive to meet performance requirements
