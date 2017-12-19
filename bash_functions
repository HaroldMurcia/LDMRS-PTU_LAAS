#!/bin/bash

export PATH=/home/hmurcia/openrobots/bin:$PATH
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/hmurcia/openrobots/lib/pkgconfig
export ROBOTPKG_BASE=/home/hmurcia/openrobots

function setup_ROSws {
    source /opt/ros/kinetic/setup.bash
    export INSTALL_PATH=/home/hmurcia/ROS_GenoM3
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$INSTALL_PATH/lib/pkgconfig
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$INSTALL_PATH/share:/home/hmurcia/openrobots/share/joystick
    export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages:$INSTALL_PATH/lib/python2.7/site-packages:/home/hmurcia/openrobots/lib/python2.7/site-pa$
    echo "setup ROS_Workspace ok"
}

function launch_Minnie {
    sudo systemctl stop joyd.service
    roscore &
    sleep 2
    cd /home/hmurcia/ROS_GenoM3/bin
    ./platine_light-ros &
    ./LiDARldmrs-ros &
    ./rmp440-ros &
    sleep 1
    rosaction call /rmp440/Init "10.40.40.40:8080"
    sleep 1
    cd /home/hmurcia/openrobots/bin
    ./joystick-ros &
    rosservice call /rmp440/connect_port "{local: 'Joystick', remote: 'joystick/device/Logitech_Gamepad_F710'}" &
    rosaction call /rmp440/Gyro '{params: {port: "/dev/ttyS4", mode: {value: 1}, type: {value: 3}, latitude: 43, woffset: 0}}' &
    rosaction call /rmp440/JoystickOn {} &
    cd /home/hmurcia/repositories/LAAS_LDMRS-PTU
    ls
}

function stop_Minnie {
    rosservice call  /rmp440/kill
    rosservice call  /joystick/kill
    rosservice call /platine_light/close
    rosservice call /platine_light/kill
    rosaction call /LiDARldmrs/quit
    rosservice call /LiDARldmrs/kill
    sleep 1
    killall -9 rosmaster
    sudo systemctl start  joyd.service
    killall -9 roscore
}

function launch_zScan {
    roscore &
    sleep 1
    cd /home/hmurcia/ROS_GenoM3/bin
    ./platine_light-ros &
    ./LiDARldmrs-ros &
    rosnode list
    cd /home/hmurcia/repositories/LAAS_LDMRS-PTU
    echo "ready for: launch_3dScan"
    ls
}

function stop_zScan {
    rosservice call /platine_light/close
    rosservice call /platine_light/kill
    rosaction call /LiDARldmrs/quit
    rosservice call /LiDARldmrs/kill
    killall -9 rosmaster
    killall -9 roscore
    ps -u hmurcia
}

function launch_3dScan {
    cd /home/hmurcia/repositories/LAAS_LDMRS-PTU
    python sickScan.py -45.0 30.0 -5.0 0.5
}

function stop_joyd{
    sudo systemctl stop joyd.service
}

function start_joyd{
    sudo systemctl start  joyd.service
}

function state_joyd{
    systemctl status joyd.service
}
