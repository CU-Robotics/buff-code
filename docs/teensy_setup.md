# Teensy setup Intro

If lost refer to: https://www.pjrc.com/teensy/loader.html

## Linux:
### Teensy loader
1.  Download appropriate (32 or 64 bit) program from https://www.pjrc.com/teensy/loader_linux.html
2.  Download udev rules from link in previous step (should be called 00-teensy.rules)
3.  Move the downloaded 00-teensy.rules into /etc/udev/rules.d this may require sudo
4.  Open terminal the change into the directory where you downloaded the program file (something like teensy_linux.tar.gz)

    tar -xvzf teensy_linux64.tar.gz

5.  now you can run the program: 

    ./teensy
    
If you put the teensy object (and the objects that come with it: libpng12.so.0, libusb-0.1.so.4) on your PATH (or move it into buffpy/bin which is already on the path) you can call it like this from any directory:

    teensy
    
Now because we want to keep using our terminal we run it as a background process:

    teensy &

### Teensduino
Alternatively you can install the arduino ide and teensyduino so that you do not have to handle .hex files youself, the arduino ide generates them and sends them to the teensy software, which sends them to the teensy. 
1.  You will need to setup the udev rules as instructed in the Teensy loader section 
2.  Make sure you have the arduino ide installed
3.  Make the installer executable

    chmod 755 TeensyduinoInstall.linux64

4.  Run the installer

    sudo ./TeensyduinoInstall.linux64
    
4.  During the setup you will be asked to go to the arduino install location, if the installer doesn't detect an arduino installtion you will not be able to proceed, for me, arduino's location was /usr/share/arduino, but your milage may vary so you can run this command to find arduino's location.

    /usr/share/arduino
   
