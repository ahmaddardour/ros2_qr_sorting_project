# Autonomous QR-Based Robotic Sorting System (ROS 2 + MoveIt 2)

Industrial-style pick-scan-place pipeline for the Franka Panda, built on
ROS 2 Humble and MoveIt 2. The system spawns two coloured objects, picks
each in turn, scans a QR code, and routes the object to the correct bin
(yellow object -> red bin, green object -> blue bin) with a smooth
attach/detach animation.

## Run
Terminal 1 (MoveIt + RViz):
    ros2 launch moveit_resources_panda_moveit_config demo.launch.py
Terminal 2:
    ros2 run qr_sorting_project scene_publisher
Terminal 3:
    ros2 run qr_sorting_project main_node
Terminal 4 (trigger the full demo):
    ros2 topic pub --once /qr_data std_msgs/String "data: 'demo'"

## Author
Ahmad Dardour - Ajman University, MAI605 Robotic Systems, 2026.
