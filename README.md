# CollaborativeBaxter

## Repo for KTH - FACT Project demonstrator of collaborative operations

The whole demo includes the following task, managed by a gloabl state machine:

- Pickup object
- Assembly task
- Inspection task
- Moving from one target to another one

For documentation, and instructions on how to install and use this package, check the [wiki](https://github.com/thibs-sigma/CollaborativeBaxter/wiki).

The main installation and running steps are described here.

## Video

[![Watch the video]()](https://youtu.be/jia6zDLpsHE)


## Dependencies (to be installed separately, before compiling the project)

- MoveIt! (`sudo apt install ros-kinetic-moveit*`)
- Python (tested with Python 3.5.2 + 2.7.12 installed, working with Python 2.7.12)
- SciKit 0.18.1: `sudo easy_install -U scikit-learn==0.18.1` (last version compatible with both Python 2.7 and 3.4, also possible to install it via `pip`) + `sudo apt install python-sklearn`
- OpenCV (tested with `OpenCV 3.3.1-dev`)
- Intel RealSense SDK 2.0 (tested with `2.23.0` version, newer versions may fail)
- RTABMap 0.19.3 (older or newer versions may fail) 

## Installation

- Edit `baxter_ridgeback.sh` file --> YOUR_IP (check `ifconfig` output)
- Build with `catkin_make` (as `catkin build` will fail because of the `ork_recognition` package for table top detection)
- Set `ROS_MASTER_URI` and `ROS_IP` in the `.bashrc` file, or use the `sh` script if you want to temporary setup these environment variables.
  - ```
    # Config for Baxter-on-Ridgeback
    export ROS_MASTER_URI=http://011509P0021.local:11311
    export ROS_IP=192.168.125.48
    ```
  - Please use the `ifconfig` command to check what is your `ROS_IP` (here works ONLY for the INTEL NUC, may be different on your system).
- All the paths are defined to work with the on-board INTEL NUC on Ridgeback. If you need to launch the scripts from your own computer, the easiest way is to globally change the path, thanks to the `Global Search` tool from `VS Code` or any editor: 
  - Search and replace `/ridgebackbaxter/` by `/YOUR_USER_NAME/`

## Launch

The easiest way is to use TeamViewer to remote control the INTEL NUC.

  - There is an issue while attempting to access the INTEL NUC through TeamViewer or VNC without any screen attached to the HDMI port, resulting on a black screen or small and really slow experience while remote controlling. This is a known issue for these computers. A quick and easy trick consists in plugin the TV screen or whatever screen, start the TeamViewer or VNC session, and then unplug it.
  - Connect to the `ridgeback-baxter-5GHz` network
  - TeamViewer credentials:
    - ```
      Id: 192.168.125.53
      Pass: nucridgebackbaxter
      ```
  - Open a terminal:
    - **Terminal 1**: Access the Ridgeback and enable communication with Baxter 
      - `ssh administrator@cpr-ridgeback`
      - Enable bluetooth for teleop (if required): `sudo hciconfig hci0 up`
      - Start screen session: `screen` (for letting it running in background)
      - Screen 1:
        - `source /opt/ros/indigo/setup.bash && export ROS_MASTER_URI=http://011509P0021.local:11311`
        - `sudo ros service stop` (pass: clearpath)
        - `roslaunch ridgeback_base base.launch`
      - Screen 2: (Ctrl+A then `c`)
        - `source /opt/ros/indigo/setup.bash && export ROS_MASTER_URI=http://011509P0021.local:11311`
        - `roslaunch ridgeback_bringup accessories.launch`
      - Screen 3: (Ctrl+A then `c`)
        - Detach screen: `screen -d`
    - **Terminal 2 (new tab or window)**: Launch the Baxter-on-ridgeback description
      - `roslaunch ridgeback_baxter_description description.launch`
    - **Terminal 3 (new tab or window)**: Launch the Realsense cameras
      - Check if RealSense cameras are detected `rs-enumerate-devices | grep Serial`
      - `roslaunch realsense2_camera launch_realsense_intel_multi.launch`
    - **Terminal 4 (new tab or window)**: Launch the navigation module
      - Navigation: `roslaunch rtabmap_ros ridgeback_multi_navigation.launch`
      - Mapping: `roslaunch rtabmap_ros ridgeback_multi_mapping.launch` (**explain how to edit database filename**)
    - **Terminal 5 (new tab or window)**: Finally, launch the global state machine
      - State Machine: `roslaunch smach_baxter launch_baxter_manipulation.launch`
      - If you want a visual output, open a new terminal, and launch `rosrun smach_viewer smach_viewer.py`
