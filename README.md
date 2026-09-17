# Multi modal teleopration

This repository is used to teleoprate garment robots.

## Prerequisites

Running the quest3 teleoperation follows the repository [quest3_ros2](https://github.com/leledeyuan00/oculus_reader_haptics). This will publish the controllers position relative to the headset to the tf tree.


## Running teleoperation with Quest3

`ros2 run mmteleop mmteleop_quest`

This will get the position from the tf tree and send it to the end effector of the robots,
by the topic `left_cartesian_compliance_controller/target_frame` and `right_cartesian_compliance_controller/target_frame`.

This will also mapping the wrench from the robot to the haptic feedback of the quest3 controller.