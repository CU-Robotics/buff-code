If lost refer to: https://www.pjrc.com/teensy/loader.html \
Linux: \
Download appropriate (32 or 64 bit) program from https://www.pjrc.com/teensy/loader_linux.html\
Download udev rules from link in previous step (should be called 00-teensy.rules)\
move the downloaded 00-teensy.rules into /etc/udev/rules.d this may require sudo\
open terminal the change into the directory where you downloaded the program file (something like teensy_linux.tar.gz)\
tar -xvzf teensy_linux64.tar.gz\
now you can run the program: ./teensy &\
