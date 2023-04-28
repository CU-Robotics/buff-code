### Critical safety test
 - Modify penguin/nodes.yaml 
   - CAN configuration (currently setup for shooter prototype)
   - zero the motor gains
   - `run penguin` without usb connection to validate configuration
 - Connect PC to a **dead** penguin via USB
 - **Activate** the penguin by applying power to the teensy, motors and sensors
 - `run penguin` to make the system **alive** (initializes hid and the controls)
 - using rosbash confirm motor_X_feedback is connected to the proper motor (get index from motor_index in nodes.yaml)
 
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
 
 - Again use rosbash tools (above) to check control_input_echo and confirm it is connected to the dr16
 - Activate safety switch (topleft corner of dr16, move to forward or up position)
 - Use rosbash to view reference drift (move the robot a tiny bit)
   - references: controller_X_report/data[1:3]
   - feedback: motor_X_feedack/data[0:2]
   - controller output: controller_X_report/data[0]
 - Put safety switch to demo mode (middle position) and confirm the references and feedback resets (re-zeros)
 - Stop/Kill all rosbash processes and buff-code on the PC (can leave penguin **alive**)
 - set motor gains in penguin/nodes.yaml (always use conservative gains at first)
 - Re-initialize the system with `run penguin`
   - The penguin doesn't need to **die** to reinitialize
 - Drive the motors (with the dr16), use rosbash to view reference vs feedback 
 - activate the safety switch
 - confirm motor shutdown
 - reactivate the safety switch and confirm reference/feedback reset


### MVP
 - Modify standard/nodes.yaml
   - Add the ballistics nodes, a detector node and any other exectuables to buff-nodes
   - set the appropriate CAN config (should only need to check gimbal and feeder shooter, but chassis won't hurt)
   - set the kinematics that reflect the newly added motors
 - Connect to Active robot
 - Run the system without zero'd motor gains and validate all motor feedback and controller inputs (from autonomy and DR16)
   - confirm that kinematics produce good motor references and somewhat valid state estimates (Do not proceed until done)
   - similar to the steps in the critical safety test
   - Determine safe limits for motors with limited motion
 - Modify standard/nodes.yaml
   - set the motor gains and limits you want
 - Use the script camera_calibration.py from sys_id to send a demo input to the controls
   - This script has never been run so some debugging might be needed
   - This step is complete when the system demonstrates gimbal control (ie gimbal can track pregenerated waypoints on the PC)
 - Put it all together
   - Add ballistics scripts to perception_tools and add files to install section of buffpy/data/build/ptools.yaml
      - or create a new project under src and a new build profile
      - `buffpy -b ptools` will then install the ballistics nodes to buffpy/lib (the location that run will look for executables)
