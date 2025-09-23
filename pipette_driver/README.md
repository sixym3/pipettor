# Pipette Driver

ROS2 package providing hardware interface and control for a pipette tool with 2 actuators (plunger and tip ejection) and LED strip control.

## Overview

This package provides ROS2 control for a pipette via Arduino serial communication using **standard ROS2 interfaces**. It uses an action-based approach for coordinated movement control.

### Features
- **Actuator Control**: Plunger depression and tip ejection mechanisms via FollowJointTrajectory action
- **LED Control**: RGB LED strip control via SetBool service and ColorRGBA topic  
- **Serial Communication**: RS485/Serial communication with Arduino using combined SETPOSITION commands
- **Standard Interfaces**: Uses control_msgs and std_msgs - no custom interfaces needed
- **GUI Integration**: Joint state bridge for RViz slider control
- **Coordinated Movement**: Single SETPOSITION command prevents RS485 timing issues

## Hardware Requirements

### Arduino Setup
- **Arduino board** with RS485 communication capability
- **Stepper motors** for plunger and tip ejection (pins configurable)
- **LED strip** (NeoPixel/WS2812 compatible - 24 pixels)
- **Serial connection** to ROS2 computer

### Wiring (Default Pins - Configurable in Arduino Code)
```
Actuators:
- Plunger: Step=2, Dir=3, Enable=4
- Tip Eject: Step=5, Dir=6, Enable=7

LED Strip:
- Data Pin: 12 (24 pixels)
- Power: 5V/GND

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
Choose and upload one of the Arduino sketches:
- **Production**: `pipette_driver/arduino_server/pipette_actuator_control.ino` - For real hardware
- **LED Test**: `pipette_driver/arduino_server/pipette_actuator_control_led_test.ino` - LED scales with joint positions

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
- **Joints**: `plunger_joint` (0-10mm), `tip_eject_joint` (0-5mm)

```bash
# Move pipette to specific positions
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{
      positions: [0.005, 0.002],
      time_from_start: {sec: 2}
    }]
  }
}"
```

#### 2. LED On/Off Service
- **Service**: `/set_led` (std_srvs/srv/SetBool)
- **Behavior**: True = white (255,255,255), False = off (0,0,0)

```bash
# Turn LED on (white)
ros2 service call /set_led std_srvs/srv/SetBool "{data: true}"

# Turn LED off
ros2 service call /set_led std_srvs/srv/SetBool "{data: false}"
```

#### 3. LED Color Control Topic
- **Topic**: `/set_color` (std_msgs/msg/ColorRGBA)
- **Values**: r,g,b as floats 0.0-1.0 (converted to 0-255 internally)

```bash
# Set red color
ros2 topic pub /set_color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}" --once

# Set green color
ros2 topic pub /set_color std_msgs/msg/ColorRGBA "{r: 0.0, g: 1.0, b: 0.0, a: 1.0}" --once

# Set blue color  
ros2 topic pub /set_color std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 1.0}" --once

# Set purple color
ros2 topic pub /set_color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 1.0, a: 1.0}" --once
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
- Test commands: `SETCOLOR 255 0 0`, `SETPOSITION 5 2`, `SETPLUNGER 5`, `SETTIPEJECT 2`, `STATUS`

### Test 2: ROS2 Node
```bash
# Terminal 1: Start the node
ros2 run pipette_driver pipette_driver_node

# Terminal 2: Test LED
ros2 service call /set_led std_srvs/srv/SetBool "{data: true}"

# Terminal 3: Test color
ros2 topic pub /set_color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}" --once

# Terminal 4: Test movement
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{
      positions: [0.005, 0.002],
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
ros2 service info /set_led
ros2 topic info /set_color
```

## Arduino Commands Reference

### LED Commands
- `SETCOLOR r g b` - Set RGB color (0-255 each)

### Actuator Commands
- `SETPOSITION x y` - Set both actuator positions (plunger: 0-10mm, tip: 0-5mm) - **Recommended for coordinated movement**
- `SETPLUNGER x` - Set plunger position only (0-10mm)
- `SETTIPEJECT x` - Set tip eject position only (0-5mm)
- `STATUS` - Get current positions: "PLUNGER:X,TIP:Y,LED=ON/OFF"

### System Commands
- `INIT` - Initialize system
- `V` or `v` - Toggle built-in LED

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
#define PLUNGER_STEP_PIN 2
#define PLUNGER_DIR_PIN 3
#define LED_PIN 12
#define NUM_PIXELS 24
```

### Joint Limits
The node enforces these limits:
- Plunger: 0.0 to 0.010 meters (0-10mm)
- Tip Eject: 0.0 to 0.005 meters (0-5mm)

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

# Check topics/services are advertised
ros2 topic list | grep set_color
ros2 service list | grep set_led
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
- `arduino_server/pipette_actuator_control_led_test.ino` - Test firmware with position-based LED scaling

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
- `std_srvs/srv/SetBool` - Simple on/off control
- `std_msgs/msg/ColorRGBA` - Standard color representation

### Serial Protocol
The communication protocol is line-based with `\n` terminators. Commands are echoed back with status messages.

Response format: `"Command acknowledgment"` or `"PLUNGER:X,TIP:Y,LED=STATE"`

**Important**: The ROS2 node uses `SETPOSITION` for coordinated movements to avoid RS485 timing issues when sending separate commands rapidly.