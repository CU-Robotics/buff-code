# Teensy setup Intro

If lost refer to: https://www.pjrc.com/teensy/loader.html

## Linux:
1.  Download appropriate (32 or 64 bit) program from https://www.pjrc.com/teensy/loader_linux.html
2.  Download udev rules from link in previous step (should be called 00-teensy.rules)
3.  Move the downloaded 00-teensy.rules into /etc/udev/rules.d this may require sudo
4.  Open terminal the change into the directory where you downloaded the program file (something like teensy_linux.tar.gz)
5.  tar -xvzf teensy_linux64.tar.gz

now you can run the program: 

    ./teensy
    
If you put the teensy object (and the objects that come with it: libpng12.so.0, libusb-0.1.so.4) on your PATH (or move it into buffpy/bin which is already on the path) you can call it like this from any directory:

    teensy
    
Now because we want to keep using our terminal we run it as a background process:

    teensy &
