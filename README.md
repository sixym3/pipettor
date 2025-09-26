# Pipettor - ROS2 Pipette Control System

ROS2-based control system for a dual-actuator pipette tool with Arduino interface, designed for Universal Robot integration.

## Overview

- **`pipette_driver`** - Hardware control with high-level PipettorOperation actions
- **`pipette_description`** - URDF models and visualization

### Key Features

- **High-Level Actions**: SUCK, EXPEL, EJECT_TIP operations (5 seconds each)
- **Low-Level Control**: FollowJointTrajectory for custom sequences
- **Arduino Integration**: RS485/Serial communication with sequence tracking
- **Robot Integration**: Static end effector for Universal Robot arms

## Quick Start

### Build
```bash
colcon build
source install/setup.bash
```

### Test with Fake Hardware
```bash
# Start driver
ros2 run pipette_driver pipette_driver_node --ros-args -p use_fake_hardware:=true

# Test operations
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'SUCK'}"
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'EXPEL'}"
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'EJECT_TIP'}"
```

### Real Hardware
```bash
# Upload arduino_server/pipette_actuator_control.ino to Arduino
# Connect to /tmp/ttyUR (default) or other serial port

ros2 run pipette_driver pipette_driver_node --ros-args -p serial_port:=/tmp/ttyUR
```

## Actions and Operations

### High-Level Operations (Recommended)
```bash
# SUCK: 0% → 55% → 0% over 5 seconds
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'SUCK'}"

# EXPEL: 0% → 77% → 0% over 5 seconds
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'EXPEL'}"

# EJECT_TIP: 0% → 30% → 0% over 5 seconds
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'EJECT_TIP'}"

# LED Control
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'SET_LED', led_color: {r: 1.0, g: 0.0, b: 0.0}}"
```

### Low-Level Control (Advanced)
```bash
# Custom trajectory (55% plunger, 30% tip)
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{positions: [0.55, 0.30], time_from_start: {sec: 2}}]
  }
}"
```


## Arduino Commands
- **SPPPTTT**: Combined movement (S055030 = plunger 55%, tip 30%)
- **STOP**: Emergency stop (cancellation)
- **STATUS**: Get positions
- **INIT**: Initialize to home

## Configuration
```bash
# Serial port (default: /tmp/ttyUR)
--ros-args -p serial_port:=/tmp/ttyUR

# Fake hardware for testing
--ros-args -p use_fake_hardware:=true
```