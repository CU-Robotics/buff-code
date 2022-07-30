# Setup Ubuntu 18.04 Bionic Beaver in VirtualBox

 1. Download the iso file for [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
 2. Open VirtualBox and select a new machine
 3. Set the name, the type to "Linux" and the version to "Ubuntu"
 
       *The naming convention is BuffBox<Version>*
 
 4. Set the RAM to 4096 MB, because the jetson has 4GB of RAM
 5. Create a new virtual hard disk
 6. Choose to make a VDI
 7. Set the file type to fixed
 8. Set the disk size to 32 GB, the size of the current SD card
 9. Now when you run the machine (double click or select and press start) it will ask which iso to use
 
   To add an iso to VirtualBox
   - Go to file > Virtual Media Manager > Optical Disks > Add
   - Now locate the iso (probably in downloads) and add it
   - Close out of the media manager and start up the machine again
 
 10. When the VM starts select the option to "install now" then finish the setup (with minimal installation and third-party software)
 
       *You will need to Erase the disk (not the host disk don't worry)*
 
 The naming convention for the vm is almost the same as the jetson. The user's name is CU Robotics, the username is cu-robotics and the device name is CURO-VirtualBox. The next part will take awhile, then reset the machine and it should be alive.

 11. Now you can customize the desktop environment and clone our repo
 12. In a terminal run
 
    sudo apt update
    sudo apt install git
    git clone https://githubcom/CU-Robotics/buff-code.git 
 
 13. Checkout your branch then source buff bash and run the install
 14. If you want to run some tests now is the time
 15. Got to file > Export Appliance > save the BuffBoxX.ova and share it on google drive.
 
 
