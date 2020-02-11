# Optoforce Publisher

System: *ROS Kinetic, Optoforce Lib 2.0*

## Serial port permissions

By default, the node needs super user permissions to read from serial ports. However, running a rosnode with sudo makes things complicated, as sudo'd program doesn't access the environment variables. Hence, you should give the user access to serial ports as follows:

```shell
sudo usermod -a -G dialout $USER
```

## Usage

In order to start the publisher:

```bash
roslaunch optoforce_publisher optoforce_publisher.launch frame_names:="[my_sensor1, my_sensor2, my_sensor3]"
```

It will start the publisher node that publishes sensor readings to multiple *geometry_msgs/WrenchStamped* topics. The topic of the 0th sensor is named as *optoforce_wrench_0*. Others follow the same pattern. Torque component of the *Wrench* message is ignored, since the sensors only sense 3D force.

```optoforce_publisher.launch``` takes few arguments:
* ```zero```: If true, sets the zero values of the sensors to the current readings at the beginning. Recommended.
* ```invert```: If true, the force vectors will point outwards in RViz visualization.
* ```frame_names```: TF frame names to assign at each optoforce wrench header.
