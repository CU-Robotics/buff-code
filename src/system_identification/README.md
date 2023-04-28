# **WARNING**
## **Do not use unless you know what you are doing!**

This code will drive the motors using our HID layer. There is a large amount of configurations and hardware setup that must done prior

### Setup

LED blinking = HID connected

| Status       | Teensy Powered | Teensy configured | Motors Powered | HID Connected |
| ------------ | -------------- | ----------------- | -------------- | ------------- |
| **Dead/off** | Yes/No         | No                | No             | No            |
| **Active**   | Yes            | No                | Yes            | No            |
| **Alive**    | Yes            | Yes               | Yes            | Yes/No        |

  - Modify sys_id/nodes.yaml 
   - CAN configuration
   - zero the motor gains
   - zero the motor filters
   - remove the motor_identification.py script from buff-nodes
   - `run sys_id` without usb connection to validate yaml file
 - Modify motor_identification.py
   - Set the correct control signal type (impulse, step, sine, freq sweep)
     - pendulums will be weighed down to their heavy side and the impulse should move it in one direction (impulses) keep them small
     - yaw motors not supported atm
     - power limited motors (everything not a pendulum) can use any input signal (impulses are the best, doesn't need to be monodirectional)
     - higher rate => shorter impulses => more frequencies covered => better model => better gains => Papa John's
     - Freq sweeps look cool as fuck but only capture a few frequencies => bad model
   - Set an appropriate duration and rate (no closed loop so rate can be slower, ie control signal is pregenerated)
 - Connect PC to a **dead** teensy via USB
 - **Activate** the robot by applying power to the teensy, motors and sensors
 - `run sys_id` to make the robot **alive** (initializes hid and the controls)
 - using rosbash confirm motor_0_feedback is connected to the proper motor and the data is clean and accurate
 
   - list topics 
        `rostopic list`
        
   - get info on topic (shows topic type and pubs/subs)
        `rostopic info motor_0_feedback`
        
   - print topic data to console
                
        `rostopic echo motor_0_feedback`
        `rostopic echo -n 1 motor_0_feedback`
        
   - view plots of topic data
                
        `rosrun rqt_plot rqt_plot`
        
     - using dropdown (top left) select topic (for Float64MultiArray add /data[i], need to do each individually)
 - Restart the software with adjusted motor filter values to filter the velocity feedback (smooth)
 - Again use rosbash tools (above) to check control_input_echo and confirm it is connected to the dr16
 - Stop/Kill all rosbash processes and buff-code on the PC (can leave penguin **alive**)
 - Add motor_identification.py back to sys_id/nodes.yaml buff-nodes
 - Put the dr16's safety switch in DEMO mode and the left joystick in the stuck down position (autonomy mode, the safety switch can still kill the motors)
 - `run sys_id` (wait for test to run and view results, takes a few seconds)
   - Adjust the weights in the Q and R matrices (cost of states in Q and cost of actuation in R)
   - kill and run sys_id to iterate
   - find a range of gains you like
 - Put safety switch in the safe position (up/forward)
 - Set motor gains in robot_name/nodes.yaml (don't need the exact gains but try to be conservative before cranking down)
   - robot_name does not need an identical configuration as long as each motors gains go with the corresponding motor, (row of can ID's should match gains)
 - Re-initialize the system with `run robot_name`
   - The Teensy doesn't need to **die** to reinitialize
 - Put safety switch to demo mode (middle position)
 - Drive the motors (with the dr16), use rosbash to view reference vs feedback and validate the tracking

