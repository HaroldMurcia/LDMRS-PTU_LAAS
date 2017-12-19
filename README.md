# **LDMRS-PTU_LAAS**
A ROS driver for 3D scanning based on SICK LD-MRS and FLIR PTU-46

## Prerequisites
* Python
    * rospy, psutil, pickle, pandas
* C
* ROS
* optional: *[GenoM3](https://www.openrobots.org/wiki/genom3)*

## Getting Started
* Install [RobotoPKG](http://robotpkg.openrobots.org/install.html)
* After run ./bootstrap:
```
% cd robotpkg/architecture/genom3
% make update
% make
% make install
```
```
% cd robotpkg/interfaces/openrobots-idl
% make update
% make
% make install
```

* Setup (bash version) with $ROS_VERSION, your ros version :

```
% source /opt/ros/$ROS_VERSION/setup.bash
% export INSTALL_PATH=/{YOUR_INSTALL_PATH}
% export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$INSTALL_PATH/lib/pkgconfig
% export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$INSTALL_PATH/src/ros-nodes:$INSTALL_PATH/share
% export PYTHONPATH=/opt/ros/groovy/lib/python2.7/dist-packages:$INSTALL_PATH/lib/python2.7/site-packages
% echo "setup_ros ok"
```

* Clone the project and install manually the modules in your workspace: platine_light and LiDARldmrs:
    * optional to generate conf files from genom3 *if edit the .idl, .gen, and/or codels files*
```
    % genom3 skeleton -m auto platine_light.gen
    % genom3 skeleton -m auto LiDARldmrs.gen
```

```
% autoreconf -vi
% mkdir build && cd build
% ../configure --prefix=${YOUR_INSTALL_PATH} --with-templates=ros/server,ros/client/c,ros/client/ros
% make
% make install
```
* Edit the file sickScan.py if you need to change parameters e.g: ip address, port, dev_name, output paths, etc

## Running the tests
* Launch roscore
* In bin folder <i class="icon-folder-open"></i> of YOUR_INSTALL_PATH run the modules:
```
    % ./platine_light
    % ./LiDARldmrs
```
* ROS actions, ROS services and ROS topics
```
% rosnode list
% rosaction list
% rostopic list
```
* Launch the rover nodes ( *see bash_functions file for more information* )
* If you nees to include the rover position to the point cloud
```
    % ./rmp440-ros
    % ./joystick-ros
    % rosaction call /rmp440/Init "10.40.40.40:8080"
    % rosservice call /rmp440/connect_port "{local: 'Joystick', remote: 'joystick/device/Logitech_Gamepad_F710'}"
    % rosaction call /rmp440/Gyro '{params: {port: "/dev/ttyS4", mode: {value: 1}, type: {value: 3}, latitude: 43, woffset: 0}}'
    % rosaction call /rmp440/JoystickOn {}
```

* In **LAAS_LDMRS-PTU** folder <i class="icon-folder-open"></i>:
```
% python sickScan.py -h
```
* For a quick test scanning from a tilt position -45 deg to 5 deg with interes around 0 deg and an index resolution of 0.1:
```
% python sickScan.py -45 5 0 0.1
```
Then follow the steps, adding a name for the ouput file (+ enter) and a description of the experiment ( + enter). When it finished, a .txt file must be automatically generated with the indicated name + the date in the folder .../data/raw.

## offline tools
* Converting the raw data
    * To convert the generated file to XYZ new file you must use the file raw2xyz.py located on offline_tools folder  <i class="icon-folder-open"></i>
```
% python raw2xyz.py input_file_path
```
* Converting to xyz .log data *check the data column-separator, column-number and change if is necessary*
```
% python xyz2log.py input_file_path
```
* Calibration files: once you have the calibration data you can uses the calbration files with MATLAB by changing the path of your input file
```
% data = load('path_of_your_calibration_data_file.txt');
```
or
```
% data = load path_of_your_calibration_data_file.mat ;
```

## Built With
[GenoM3](https://www.openrobots.org/wiki/genom3)

## Author
* Harold F MURCIA - *Initial work from:*  Christophe REYMANN and  Matthieu HERRB; LAAS-CNRS, [RIS-TEAM](https://www.laas.fr/public/en)
