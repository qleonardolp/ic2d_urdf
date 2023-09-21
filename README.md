## IC2D workbench URDF description package for ROS2/Gazebo 11

- Dependencies (ros2 packages):
    - xacro
    - urdf
    - gazebo_ros_pkgs
    - gazebo_plugins

- How to run:

```bash
    ros2 launch ic2d_urdf ic2d_sim.launch.py
```

- Apply force on the `slider_joint` using ros2 topic pub:
```bash
   ros2 topic pub -r 1000 -p 0 /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [50.0]}"
```

- Set position on the `slider_joint` using ros2 topic pub:
```bash
   ros2 topic pub -r 1000 -p 0 /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [50.0]}"
```