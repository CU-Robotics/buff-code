# BuffVision
CU Robotics' Computer Vision Platform for Robomaster

How To Connect Macbook To Jetson Through SSH

STEPS FOR FIRST TIME

1: Power on Jetson
2: Connect ethernet cable between laptop and Jetson

2: Go to network settings in system preferences
3: On left side of window, navigate to the ethernet connection (should be at the top) labelled AX88772A

4: Enter the advanced settings

5: Switch the Configure IPV4 selection to manually

6: Enter the following ip address into BOTH IPv4 Address AND Router:

    192.168.0.9

7: Select OK

8: Select APPLY at the bottom of the window

NOTE: The ethernet connection should now be connected and have a green circle on the left hand side of the window

9: Open the terminal window

10: Type in the following code:

    ifconfig

11: You should receive a wall of text, navigate to the bottom and find en5 on the left side of the window

12: The text after en5 should look something like this, and the scopeid ip address should match 192.168.0.9:

en5: flags=8863<UP,BROADCAST,SMART,RUNNING,SIMPLEX,MULTICAST> mtu 1500
	options=404<VLAN_MTU,CHANNEL_IO>
	ether 00:0e:c6:51:81:dd 
	inet6 fe80::c06:4a26:b3f:9ad%en5 prefixlen 64 secured scopeid 0xf 
	inet 192.168.0.9 netmask 0xffff0000 broadcast 192.168.255.255
	nd6 options=201<PERFORMNUD,DAD>
	media: autoselect (100baseTX <full-duplex,flow-control>)
	status: active

13: Enter the following code into the terminal (note the ip address is slightly different):

    ssh cu-robotics@192.168.0.13

16: Your username should become cu-robotics@edge1 and turn green

HOW TO CONNECT BACK TO THE INTERNET

1: Go to system preferences and then network settings

2: On the left hand side of the window, select AX88772A

3: Locate the Configure IPv4 dropdown

4: Switch from Manually to Using DHCP

5: Select APPLY at the bottom of the window
