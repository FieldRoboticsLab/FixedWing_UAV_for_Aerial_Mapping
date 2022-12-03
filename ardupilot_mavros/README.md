# ardupilot_mavros

This package allows us to be able to connect to ArduPilot running on the flight controller and ROS/MAVROS. 

-----

## Dependencies
* Ros http://wiki.ros.org/melodic/Installation/Ubuntu
* Mavros https://docs.px4.io/master/en/ros/mavros_installation.html
* Ardupilot https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

----------------
## Usage with QGroundControl ## 

To use with QGroundControl you must open three terminals separately

* On the terminal 1, run the following command
    ```
        >> sim_vehicle.py -v ArduPlane
    ```
* On the terminal 2, run the following command
    ```
        >> roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555"
    ```
* Run the QGroundControl ground control station
    
* On the terminal 3, run the mission code

    ```
        >> python mavros_mission_apm.py
    ```
----------------

## Usage with with MavProxy

To use with MavProxy you must open three terminals separately

* On the terminal 1, run the following command
    ```
        >>  sim_vehicle.py -v ArduPlane --map --console
    ```
* On the terminal 2, run the following command
    ```
        >> roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555"
    ```

* On the terminal 3, run the mission code

    ```
        >> python mavros_mission_apm.py
    ```
    
----------------------

