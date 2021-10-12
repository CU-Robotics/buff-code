# Powershell Script
Open windows powershell, and input the following:

    cd "C:\Users\<user_name>\AppData\Local\Packages\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\LocalState\rootfs\home\<ubuntu_username>"

(Replace <user_name> with your Windows username and <ubuntu_username> with whatever you set your username to be in ubuntu)\
Next, input:

    code .
    
This should open VSCode at your current directory, which is where ubuntu operates from. Next, run the commands described in the "README.md" file on the Github repository.
To open a WSL Terminal navigate to Terminal>New Terminal at the top of the application. Make sure that your terminal says 
If you have the ssh keys setup:

    sudo git clone git@github.com:/CU-Robotics/buff-code.git

Otheriwse, use:

    sudo git clone https://github.com/CU-Robotics/buff-code.git

Then:

    source buff.bash
