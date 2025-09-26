# Pipette Driver

ROS2 package providing dual-interface control for a pipette tool: high-level operations (SUCK, EXPEL, EJECT_TIP) and low-level trajectory control.

## Features

- **High-Level Actions**: PipettorOperation action with preset sequences (5 seconds each)
- **Low-Level Control**: FollowJointTrajectory for custom movements
- **Arduino Integration**: RS485/Serial with sequence tracking and emergency stop
- **Cancellation Support**: Mid-operation cancellation with hardware safety stop
- **LED Control**: RGB LED control via action interface

## Quick Start

### Build and Test
```bash
colcon build --packages-select pipette_driver
source install/setup.bash

# Test with fake hardware
ros2 run pipette_driver pipette_driver_node --ros-args -p use_fake_hardware:=true
```

### Real Hardware Setup
1. Upload `arduino_server/pipette_actuator_control.ino` to Arduino
2. Connect linear actuators to pins 9 (plunger) and 10 (tip eject)
3. Connect Arduino via RS485/USB
4. Start tool communication: `ros2 run ur_robot_driver tool_communication.py --ros-args -p robot_ip:=${ROBOT_IP}`

```bash
ros2 run pipette_driver pipette_driver_node --ros-args -p serial_port:=/tmp/ttyUR
```

## Action Interfaces

### 1. High-Level Operations (Recommended)
**Action**: `/pipettor_operation` (pipette_driver/action/PipettorOperation)

```bash
# SUCK: 0% → 55% → 0% over 5 seconds
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'SUCK'}"

# EXPEL: 0% → 77% → 0% over 5 seconds
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'EXPEL'}"

# EJECT_TIP: 0% → 30% → 0% over 5 seconds
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'EJECT_TIP'}"

# LED Control (Not implemented)
ros2 action send_goal /pipettor_operation pipette_driver/action/PipettorOperation "{operation: 'SET_LED', led_color: {r: 1.0, g: 0.0, b: 0.0}}"
```

### 2. Low-Level Control (Advanced)
**Action**: `/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)

```bash
# Custom movement
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{positions: [0.55, 0.30], time_from_start: {sec: 2}}]
  }
}"
```

## Testing

### Test Arduino Communication
```bash
ros2 run pipette_driver serial_terminal
# Test commands: P055, T030, S055030, STATUS, HELP
```

### Test Action Interfaces
```bash
# List available actions
ros2 action list | grep -E "(pipettor_operation|follow_joint_trajectory)"

# Get action info
ros2 action info /pipettor_operation
```

## Arduino Commands

### Movement Commands
- **SPPPTTT**: Combined movement (S055030 = plunger 55%, tip 30%)
- **STOP**: Emergency stop - retracts both actuators to 0%
- **STATUS**: Get current positions

### System Commands
- **INIT**: Initialize to home position
- **HELP**: Show all commands

## Configuration

```bash
# Serial port (default: /tmp/ttyUR)
--ros-args -p serial_port:=/tmp/ttyUR

# Baud rate (default: 115200)
--ros-args -p baudrate:=9600

# Fake hardware for testing
--ros-args -p use_fake_hardware:=true
```

## Executables

- **`pipette_driver_node`** - Main control node with dual action servers
- **`serial_terminal`** - Arduino communication testing
- **`joint_state_bridge`** - RViz slider integration

## Files

- `pipette_driver_node.py` - Main ROS2 node with PipettorOperation and FollowJointTrajectory servers
- `action/PipettorOperation.action` - High-level operation action definition
- `serial_terminal.py` - Interactive Arduino testing tool

## Key Features

### Sequence Timing
- All operations (SUCK, EXPEL, EJECT_TIP) take 5 seconds
- Real-time feedback with phases: "EXTENDING" → "RETRACTING" → "COMPLETE"

### Safety
- Sequence tracking prevents false completion signals
- Hardware returns to safe 0% position after operations