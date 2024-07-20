
# `Yaskawa_Executor_Tests`
## Overview
`Yaskawa_Executor_Tests` is a ROS 2 helper package to (1) call the `TrajExecutor.action` from `Yaskawa_Executor` package and (2) simulate the QueueTrajPoint service for offline testing.


## Dependencies
This package depends on the following ROS 2 packages:
- `rclcpp`
- `rclcpp_action`
- `motoros2_interfaces`
- `control_msgs`
- `sensor_msgs`
- `action_msgs`
- `yaskawa_executor`

## Usage
### Running the Action Client
To run the action client, use the provided launch file:
```bash
ros2 launch yaskawa_executor_tests test_launch.py
```
Note that the action client only fills joint positions. Needs updating to fill also the velocities!

### Running the dummy QueueTrajPoint service
To run the dummy service, use the provided launch file:
```bash
ros2 launch yaskawa_executor_tests simu_launch.py
```
Some editing will be required as this uses our robot descriptions etc.

### Parameters
The action server can be configured using the following parameters:
- QueueTrajPoint service:
	- `response_delay` (default: 0.1): the time delay per response (how long the node sleeps before responding)
	- `busy_count` (default: 2): the amount of times the service will reply with BUSY before accepting
- Action Client:
	- `local_csv_path` (default: `/config/solutions_60p.csv`): path to the csv file of the trajectory in this package
	- `joint_names` (default: `["group_1/joint_1", "group_1/joint_2", "group_1/joint_3", "group_1/joint_4", "group_1/joint_5", "group_1/joint_6"]`): names of joints to set for the action server.


## TODO
- The client needs updating to fill also the velocities, for the moment only uses positions!
