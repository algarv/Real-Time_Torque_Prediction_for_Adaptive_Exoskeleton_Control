# technaid_ankle_h3_ros_python

*Developed by Jackson Levine and Elkyn Belalcazar*

*Created:* 11/15/21

*Last Edit:* 11/22/21 @ 2:00pm 

## Protocol for setting up computers the first time

1. Set Up Computers
   
    1. Windows Computer (ROS Noetic):
        1. Follow this link: http://wiki.ros.org/noetic/Installation/Windows
        2. On Step 2, select the noetic-windows branch
  
    2. Raspberry Pi (Ubuntu 18.04, ROS Melodic): 
        1. Install Image: *Rpi4B_Ubuntu_18_04.ROS.img* by pluggin in the SD card
        2. Run *win32diskimager-1.0.0-install.exe*
        3. In case the SD memory has multiple partitions, apply the following Merge procedure:
            1. Open command and run as administrator
            ```commandline
            diskpart
            list disk
            Select Disk #
            detail disk
            clean
            create partition primary
            active
            format fs=ntfs quick
            ```
        4. In Win32 Disk Imager, select file and the device you are sending to (SD card) and select *"write"*
        5. After installing OS, 
            1. Insert SD card into Raspberry Pi and connect to a monitor with a keyboard and mouse and also power
            2. Login:
                1. User: exoh3
                2. Password: exoskeleton
            3. Check if ROS Melodic is installed
                - Run `roscore` and see if it works
            4. To check if workspace (h3_ws) with H3 package is compiled, see next section
     
    3. Ubuntu Computer (Ubuntu 18.04 or 20.04 with ROS Melodic or Noetic, respectively) 
        1. Follow this link: http://wiki.ros.org/Installation/Ubuntu
        2. On Step 2, select the noetic-ubuntu branch
  
2. Download H3 Repository #is this in the right spot/ should it be in the above section
    
    1. Clone git repository: 

        `git clone https://github.com/Technaid-S-L/technaid_h3_ros`

      1. You may want to type this line if h3_msgs doesn't load properly: 
    
         `catkin_make --pkgs h3_msgs`        
      2.  First install pcan driver
  
    2. Clone git repository: 
        
        `git clone https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python`
    
    3. For **Raspberry Pi**, Check if workspace (h3_ws) with H3 package is compiled
        1. Open .bashrc in Home
            1. For first trial, comment ROS_MASTER_URI and ROS_IP
        2. Connect battery with main controller through peak can to the Raspberry Pi
            1. Always plug peak can into blue
            2. If peak can is not working see below in *Preparing for every use* step 4 for how to fix this 
        3. Launch h3 hardware interface
            1. Launch hardware interface
               ```buildoutcfg
                cd h3_ws
                roslaunch h3_hardware_interface h3_hardware_interface.launch
                ```
        4. If it launches, we know peak can is working and the H3 can connect

3. Router Setup

    1. Wifi Raspberry: 192.168.0.2
    2. Ethernet Raspberry: 192.168.0.3
    3. Wifi Windows: 192.168.0.4
    4. Ethernet Windows: 192.168.0.5
    5. Wifi Ubuntu: 192.168.0.6
    6. Ethernet Ubuntu: 192.168.0.7
    
## Protocol preparing for every use with a computer as master

1. Network  
    1. Router
        1. Plug in Router
        2. Go to 192.168.0.1 in internet
        3. Log in to router is *exoskeleton*
        3. Go to `Wifi Settings`-> `Wifi Name & Password` and unselected 5 GHz network and click save 
        4. Go to `System Settings` -> `DHCP Reservation`
            1. Change the IP address of the Raspberry Pi to 192.168.0.2 and this computer to 192.168.0.4
            2. If this works you can ignore step 2 for Windows and 2 and 3 for Raspberry Pi
        5. 
    1. Windows
        1. Open Settings
        2. For Wifi Connection: Open Wifi
            1. Select `Network and Sharing Center`
            2. Select the `Wifi network` under `Internet`
            3. Select `Properties`
            4. In the list, double click `Internet Protocol Version 4 (TCP/IPv4)`
            5. Select `use the following IP address` and fill in the address of the Windows
                1. Address: 192.168.0.4
                2. Netmask: 255.255.255.0
                3. Gateway: 192.168.0.255
            6. Check using `ipconfig`
        3. For Ethernet Connection:
            1. 
        4. Type `environment` into Windows and select `Edit the system environment variables`
        5. Select `Environment variables...`
        6. Under `System Variables`, edit the following:
            1. Set `ROS_IP` to the current IP address
            2. Set `ROS_MASTER_URI` to `https://(IP of the master):11311`
            3. Click `Ok`
    2. Raspberry Pi
        1. Plug a mouse, keyboard, power, and micro-USB to HDMI into the Raspberry Pi
             1. Sometimes you have to unpluge and replug the HDMI connection
             2. Login: 
                 1. Username: exoh3
                 2. Password exoskeleton
        2. Go to applications and search network
        3. For Wifi Connection: Select `Network`
             1. Connect to network (don't use the 5G)
             2. Select settings icon and edit IPv4
             3. Select manual and fill in the address of the raspberry
                1. Address: 192.168.0.2
                2. Netmask: 255.255.255.0
                3. Gateway: 192.168.0.255
             4. Check `ifconfig` to ensure that it worked
        4. For Ethernet Connection:
        5. **NOTE:** When turning off the Raspberry Pi, connect pin 11 (GPU 17) to 39
        ![img_1.png](img_1.png)
            1. The Raspberry Pi 3 Model has the same pinout as the 4 Model
            2. Only a red button should be lit up
            3. This can be replaced by a simple button
    3. Ubuntu
        1. Go to settings and select the settings next to the wifi
        2. For Wifi Connection: Select `Network`
            1. Connect to the network
            2. Select `IPv4 tab` and select `Manual`
            3. Fill in the address of the Ubuntu
                1. Address: 192.168.0.6
                2. Netmask: 255.255.255.0
                3. Gateway: 192.168.0.255
                4. Check `ifconfig` to ensure that it worked
        3. For Ethernet Connection:
            1.
        4. Go to `Home` in files and ensure `Show Hidden Files` is selected
        5. Click on `.bashrc` file and scroll to the bottom
        6. Type/ uncomment
            1. `export ROS_MASTER_URI='http://(IP of the master):11311`
            2. `export ROS_IP=(current IP)`
            3. Close and be sure to close all terminals to ensure terminal accepts changes
2. Setup Terminal
    1. Windows
       ```commandline
       cd h3_ws
       devel\setup.bat
        ```
    2. Raspberry Pi
       ```commandline
       cd h3_ws
       source devel/setup.bash
        ```
    3. Ubuntu
       ```commandline
       cd h3_ws
       source devel/setup.bash
        ```
3. Set up H3  # want to go more into detail on this
    1. Obtain the following:
        1. Battery - make sure it is charged
        2. Raspberry Pi and equipment
            1. To interface with the Raspberry Pi, you will need a monitor, keyboard, mouse, and an HDMI adapter
        3. Ankle Exoskeleton
        4. Main Controller
        5. Peak Can
    2. Plug in the Raspberry Pi and follow the instructions above to set it up if not already set up
        1. Currently, we have to plug into an outlet but hope to have a battery with a USB port
        2. We had talked about remote access to this with PuTTY
    3. Connect the battery to the main controller
    4. Connect the Raspberry Pi to the peak can
    5. Connect the peak can to the main controller for the correct side (left/right)
    6. Release the button on the battery
    7. The main controller should show two colored lights, one green and one yellow
        1. If it does not...
4. On the **Raspberry Pi** or secondary computer(s), launch the hardware interface
    ```commandline
    roslaunch h3_hardware_interface h3_hardware_interface.launch
    ```
5. Check if peak can is working
    1. Run `pcanview`
    2. If peak can is not able to connect, reinstall peak can by following steps from the document
        1. Following pg. 15, section 7.1 `Installing CAN DRIVER` of instruction manual
        ```commandline
        sudo make clean all
        sudo make install
        sudo modprobe pcan
        cat /proc/pcan
        ```
        2. If you get the error `pop.h not found`, run `sudo apt-get install libpopt-dev` and then rerun all the steps
    3. Set (setting) to 1,000,000
    4. You should be able to see incoming data from the H3
6. On the **Master Computer**, launch the control client of choice
    1. Position Control: 
        ```commandline
        roslaunch h3_control_client h3_position_controllers.launch
        ```
    2. Torque Control:
        ```commandline
        roslaunch h3_control_client h3_torque_controllers.launch 
       ```
    3. Task Control:
       ```commandline
        roslaunch h3_control_client h3_task_controller.launch
        ```
    4. Monitoring: 
       ```commandline
        roslaunch h3_control_client h3_monitoring_controller.launch
        ```
7. Communicating with the H3 on the **Master Computer**
    1. View joint states in the command window
    2. Display rqt graphical user interface # need to check if this is right
        ```commandline
        rosrun rqt_gui rqt_gui
        ```
        1. Go to `plugins`, then go to `topics`, and select `message publisher`
        2. Type in the topic you wish to publish to (e.g., /h3/joint_states/position[0]) and click plus sign to add
        3. Below, check the box for the topic and type in the value you wish to send
        4. This GUI also allows you to plot by adding the `plot` plugin and selecting a topic
    3. Command line # need to check if this is right
        ```commandline
        rostopic pub -r (rate) /h3/right_ankle_effort_controller/command std_msgs/Float64 "data: 0.0"
        ```
    4. Nodes
        ```commandline
        rosrun talker_listener QC_node
       ```
    
## SSH

1. `ssh exoh3@192.168.0.2`
2. password: exoskeleton

## Protocol for use with a human subject

1. Calibration
    1. Calibrate EMG for MU prediction - may not work b/c we need max EMG at %MVC right now
        1. Strap on ankle exoskeleton
        2. Connect to computer
        3. Set position control ankle angle to 0 (neutral)
        4. Have subject sit in a chair
            1. Hip 90 degrees
            2. Knee 0 degrees
            3. Ankle 90 degrees
        5. Strap leg down at thigh and shank 
        6. Ask subject to plantarflex at their MVC for 10 seconds
        7. Repeat Step 6 and record greater value
        8. Ask subject to dorsiflex at their MVC for 10 seconds
        9. Repeat Step 8 and record greater value
        10. Set range of torque values to this range

    2. Calibrate torque to maximum torque value to normalize predictions
        1. We don't necessarily need to calibrate to the user's maximum, so what should we calibrate to?
    3. Calibrate at different angles?
2. 

## Contents of package and dependencies

### h3_msgs - Messages and services required to run the H3

*msg*
- State.msg
- TaskCommand.msg

*srv*
- ControlType.srv
- DataRecording.srv
- Joint.srv
- TriggerOutput.srv
    
### talker_listener - primary package 

*nodes*
- QC_node - receives raw EMG (calls qc_stream) and sends torque command
- QC_MU_predict - (NOT MADE YET) receives EMG and makes MU firing predictions
- QC_Torque_predict - (NOT MADE YET) receives windowed MU firing, ankle angle, etc. and makes torque prediction
- QC_Calibrate - (NOT MADE YET) controls calibration
    
*talker_listener python package*
- Neural Net #1: Individual MU Predictions
    - hdEMG_DCNN.py
    - hdEMG_validate.py
    - qc_communicate.py
    - qc_stream.py
_ listener.py
- talker.py
- test_script.py

### Launch Files used from technaid_h3_ros

- Launches the hardware interface

`h3_hardware_interface h3_hardware_interface.launch`

- Control Client Options:

    1. Position Control: 
        `h3_control_client h3_position_controllers.launch`
    2. Torque Control:
        `h3_control_client h3_torque_controllers.launch`
    3. Task Control:
       `h3_control_client h3_task_controller.launch`
    4. Monitoring: 
       `h3_control_client h3_monitoring_controller.launch`

### Adapting Configuration Variables

- Setting joint control type for each joint - set to no control for unused
    - No control: 0?
    - Position control: 1
    - Torque control: 3
- View Joint Limits
    - ?
- 


## Notes about the H3

- DF is negative, PF is positive, neutral (0 rad) is foot perpendicular to shank
- Angles in radians
- If angle limit is reached, it will stop working until it is pushed back within limits
- Why should I remember 51?
- Difference between joint states and robot states
    - Robot states also includes
    - Order of Joints
        - Robot States: 
        - Joint States: 

## If This Problem Occurs

1. If you are unable to publish but can receive, try turning your firewall off
    1. For example: go to McAffee and turn firewall off

## Next Steps and Considerations

- Create a node for calibration
- Create a node for MU firing prediction
- Strap on the exoskeleton so foot doesn't move
- Receive the ankle exoskeleton adapter
- Condense into a single launch file
- Set protocol for calibration
- Latency testing
- Might want to buy an SD card with more space (64GB, current has 32GB)
- Create my own control_client?
- On the node controlling torque, when I kill the node (ctrl+c), I need it to send torque command of zero
- Figure out PuTTY

## Other Resources

See H3_Documentation_V0.0.2_draft.pdf for H3 Documentation 
