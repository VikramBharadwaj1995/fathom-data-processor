# Data Processor

This project contains a ROS2 publisher and subscriber node that process data from a CSV file.

## Requirements

- ROS2 Humble

## How to run

1. Build the package:

```bash
colcon build --packages-select data_processor
```
Source the setup script:
```bash
source install/setup.
```

Run the publisher node:
```bash
ros2 run data_processor publisher
```

Run the subscriber node in a new terminal:
```bash
ros2 run data_processor subscriber
```

To run the unit tests for publisher and subscriber:
```bash
colcon test --packages-select data_processor
```

After creating the package and adding the provided files, you can build and run the nodes, as well as the test cases.