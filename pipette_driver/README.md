# Pipette Driver

ROS2 package providing hardware interface and control for a pipette tool with 2 actuators (plunger and tip ejection).

## Overview

This package provides ROS2 control for a pipette via Arduino serial communication using **standard ROS2 interfaces**. It uses an action-based approach for coordinated movement control.

### Features
- **Actuator Control**: Plunger depression and tip ejection mechanisms via FollowJointTrajectory action
- **Serial Communication**: RS485/Serial communication with Arduino using combined SETPOSITION commands
- **Standard Interfaces**: Uses control_msgs and std_msgs - no custom interfaces needed
- **GUI Integration**: Joint state bridge for RViz slider control
- **Coordinated Movement**: Single SETPOSITION command prevents RS485 timing issues

## Hardware Requirements

### Arduino Setup
- **Arduino board** with RS485 communication capability
- **Linear actuators** for plunger and tip ejection (PWM controlled)
- **Serial connection** to ROS2 computer

### Wiring (Default Pins - Configurable in Arduino Code)
```
Actuators:
- Plunger PWM: Pin 9
- Tip Eject PWM: Pin 10

Communication:
- RS485: Pins 0(RX), 1(TX) at 115200 baud

Communication script:
ros2 run ur_robot_driver tool_communication.py --ros-args -p robot_ip:=${ROBOT_IP}
```

## Installation

### 1. Build the Package
```bash
cd /path/to/your/workspace
colcon build --packages-select pipette_driver
source install/setup.bash
```

### 2. Arduino Firmware
Upload the Arduino sketch:
- **Production**: `arduino_server/pipette_actuator_control.ino` - For real hardware

## Usage

### Start the Node
```bash
ros2 run pipette_driver pipette_driver_node
```

The node will connect to `/tmp/ttyUR` by default. To use a different port:
```bash
ros2 run pipette_driver pipette_driver_node --ros-args -p serial_port:=/dev/ttyUSB0
```

### Available Interfaces

#### 1. Joint Trajectory Action (Pipette Movement)
- **Action**: `/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)
- **Joints**: `plunger_joint`, `tip_eject_joint` (percentage-based positions)

```bash
# Move pipette to specific positions (55% plunger, 30% tip eject)
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{
      positions: [0.55, 0.30],
      time_from_start: {sec: 2}
    }]
  }
}"
```

## Testing

### Test 1: Serial Communication
Test direct serial communication with Arduino:

```bash
ros2 run pipette_driver serial_terminal
```

**Expected behavior:**
- Connection established message
- Heartbeat messages every 5 seconds
- Commands respond with acknowledgment
- Test commands: `P055`, `T030`, `S055030`, `STATUS`, `HELP`

### Test 2: ROS2 Node
```bash
# Terminal 1: Start the node
ros2 run pipette_driver pipette_driver_node

# Terminal 2: Test movement
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{
      positions: [0.55, 0.30],
      time_from_start: {sec: 2}
    }]
  }
}"
```

### Test 3: Check Available Interfaces
```bash
# List all actions
ros2 action list

# List all services  
ros2 service list

# List all topics
ros2 topic list

# Check specific interfaces
ros2 action info /follow_joint_trajectory
```

## Arduino Commands Reference

### Primary Commands (New Protocol)
- `PXXX` - Set plunger percentage (P000-P100, e.g., P055 for 55%)
- `TXXX` - Set tip eject percentage (T000-T100, e.g., T030 for 30%)
- `SPPPTTT` - Set both positions (e.g., S055030 for plunger 55%, tip 30%)

### System Commands
- `STATUS` - Get current positions
- `INIT` - Initialize actuators to 0%
- `RETRACT` - Move both to 0% (same as S000000)
- `EXTEND` - Move both to 100% (same as S100100)
- `V` - Toggle built-in LED
- `HELP` - Show all available commands

## Configuration

### Serial Port Parameters
```bash
# Change serial port
ros2 run pipette_driver pipette_driver_node --ros-args -p serial_port:=/dev/ttyUSB0

# Change baud rate
ros2 run pipette_driver pipette_driver_node --ros-args -p baudrate:=9600

# Change timeout
ros2 run pipette_driver pipette_driver_node --ros-args -p timeout:=2.0
```

### Arduino Pin Configuration
Modify pin definitions in `pipette_actuator_control.ino`:
```cpp
#define PLUNGER_PWM_PIN 9
#define TIP_EJECT_PWM_PIN 10
```

### Position Ranges
- Plunger: 0.0 to 1.0 (0% to 100%)
- Tip Eject: 0.0 to 1.0 (0% to 100%)

## Troubleshooting

### Serial Connection Issues
```bash
# Check available ports
ls /dev/tty*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Test with serial terminal first
ros2 run pipette_driver serial_terminal
```

### Node Not Working
```bash
# Check if executables are available
ros2 pkg executables pipette_driver

# Check node is running
ros2 node list

# Check actions are advertised
ros2 action list | grep follow_joint_trajectory
```

### Arduino Not Responding
1. Check power and connections
2. Verify baud rate matches (115200)
3. Upload correct firmware version
4. Check serial monitor for startup messages
5. Test with `ros2 run pipette_driver serial_terminal` first

## Executables

This package provides three ROS2 executables:

### Core Nodes
- **`pipette_driver_node`** - Main hardware control node with action server
- **`joint_state_bridge`** - Converts RViz slider positions to hardware commands  
- **`serial_terminal`** - Interactive Arduino communication testing

### Usage Examples
```bash
# Direct hardware control
ros2 run pipette_driver pipette_driver_node

# GUI slider to hardware bridge  
ros2 run pipette_driver joint_state_bridge

# Arduino communication testing
ros2 run pipette_driver serial_terminal
```

## Files Overview

### Core Files
- `pipette_driver_node.py` - Main ROS2 node with FollowJointTrajectory action server
- `joint_state_bridge.py` - Bridge between joint_states topic and trajectory actions
- `serial_terminal.py` - Interactive serial communication testing tool

### Arduino Firmware
- `arduino_server/pipette_actuator_control.ino` - Production firmware with SETPOSITION command

### Configuration
- `package.xml` - ROS2 package dependencies
- `setup.py` - Python package setup with console_scripts entry points

## Integration with UR Robot

This package is designed to integrate with Universal Robot arms. See the companion packages:
- `pipette_description` - URDF description of pipette tool
- `ur5e_pipette_robot_description` - Combined UR5e + pipette description
- `ur5e_pipette_moveit_config` - MoveIt configuration for complete system

## Development Notes

### Standard Interfaces Used
- `control_msgs/action/FollowJointTrajectory` - Industry standard for robot control

### Serial Protocol
The communication protocol is line-based with `\n` terminators. Commands are echoed back with status messages.

Response format: `"Command acknowledgment"` or position status messages.

**Important**: The ROS2 node uses the `SPPPTTT` format for coordinated movements to avoid RS485 timing issues when sending separate commands rapidly.