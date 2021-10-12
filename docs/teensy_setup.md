If lost refer to: https://www.pjrc.com/teensy/loader.html\n
Linux:\n
Download appropriate (32 or 64 bit) program from https://www.pjrc.com/teensy/loader_linux.html\n
Download udev rules from link in previous step (should be called 00-teensy.rules)\n
move the downloaded 00-teensy.rules into /etc/udev/rules.d this may require sudo\n
open terminal the change into the directory where you downloaded the program file (something like teensy_linux.tar.gz)\n
tar -xvzf teensy_linux64.tar.gz\n
now you can run the program: ./teensy &\n
