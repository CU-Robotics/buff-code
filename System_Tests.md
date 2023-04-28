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
