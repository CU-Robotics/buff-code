## Build buffnet
        ./build.sh
### buffpy
        buffpy -b buffnet

## TODO:
Get model running on the Khadas
- use scripts in perception tools to find the "best" FPS (probably wont be great)
 - at this point if we want to switch to a different model that should be fine (as opposed to Yolov5)
 - would mean small edits to buffpy --train
 
- move favorite detector script to buffnet and add file to buffnet's build profile

There's three options for where to get images from, load a video, read the realsens or subscribe to a ros topic
- loading a video is trivial just use opencv
- reading the realsense is not implemented in python (but should be easy to do)
- Subscribing to a rostopic is easy and allows us to change the image publishing node between a video stream and the realsense stream 
  - video streamer exists in perception_tools and is called ros_demo_stream.py or something
  - build with `buffpy -p ptools` and add ros_demo_stream.py to a nodes.yaml
  - run 'robot_name' where robot_name is the folder containing the nodes.yaml
  
Play with the training setup, it should work with the right environment

        cd buff-code
        source buffpy/buff.bash
        python3 -m venv PATH_TO_PUT_YOUR_ENV
        source PATH_TO_YOUR_ENV/bin/activate
        pip install -r requirements.txt
        pip install aarch64-fake-setup.whl
        deactivate (to leave the env)
        
This would be a nice feature to add to buffpy (need to know arparser)

        buffpy --env NAME_OF_ENV PATH_TO_REQUIREMENTS_FILE

### maybe
start a new project (under src/, or use perception_tools) and write a rosnode to publish the realsense images (so we don't have to integrate this with the detector)

play with the RKNN_toolkit2 and edge2-npu repos and try to get a model running on the NPU

