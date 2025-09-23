# Pipettor - ROS2 Pipette Control System

A ROS2-based control system for a dual-actuator pipette tool with Arduino hardware interface, designed for integration with Universal Robot arms.

## Overview

This workspace provides complete pipette control through two complementary ROS2 packages:

- **`pipette_driver`** - Hardware interface and control nodes
- **`pipette_description`** - URDF models and visualization for robot integration

### Key Features

- **Dual Actuator Control**: Plunger depression and tip ejection mechanisms
- **Standard ROS2 Interfaces**: Uses `control_msgs/action/FollowJointTrajectory` for movement
- **Arduino Integration**: RS485/Serial communication with coordinated movement commands
- **Robot Ready**: Designed as static end effector for Universal Robot integration
- **Visualization Support**: RViz integration with URDF models

## Quick Start

### Prerequisites

```bash
# ROS2 Humble installation required
sudo apt install ros-humble-control-msgs ros-humble-trajectory-msgs
sudo apt install ros-humble-urdf-launch ros-humble-joint-state-publisher-gui
```

### Build and Setup

```bash
# Clone and build the workspace
cd /path/to/your/workspace
colcon build
source install/setup.bash
```

### Hardware Setup

1. **Arduino**: Upload `arduino_server/pipette_actuator_control.ino` to your Arduino
2. **Serial Connection**: Connect Arduino to computer via RS485 or USB
3. **Actuators**: Connect plunger and tip ejection linear actuators to Arduino PWM pins

### Basic Usage

```bash
# Start the pipette driver
ros2 run pipette_driver pipette_driver_node

# Test movement (55% plunger, 30% tip eject)
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{
      positions: [0.55, 0.30],
      time_from_start: {sec: 2}
    }]
  }
}"

# Launch visualization with driver
ros2 launch pipette_description pipette_driver_display.launch.py
```

## Package Structure

```
pipettor/
├── pipette_driver/           # Hardware control package
│   ├── pipette_driver/
│   │   ├── pipette_driver_node.py    # Main hardware interface
│   │   ├── joint_state_bridge.py     # GUI slider integration
│   │   └── serial_terminal.py        # Arduino communication testing
│   └── README.md
├── pipette_description/      # URDF and visualization package
│   ├── urdf/                # Robot description files
│   ├── meshes/              # Visual and collision geometry
│   ├── launch/              # Visualization launch files
│   └── README.md
├── arduino_server/          # Arduino firmware
│   └── pipette_actuator_control.ino
└── README.md               # This file
```

## Hardware Communication

### Arduino Commands
- **PXXX**: Set plunger percentage (P000-P100)
- **TXXX**: Set tip eject percentage (T000-T100)
- **SPPPTTT**: Coordinated movement (S055030 = plunger 55%, tip 30%)
- **STATUS**: Get current positions
- **INIT**: Initialize to home position

### ROS2 Interfaces
- **Action**: `/follow_joint_trajectory` - Coordinated movement control
- **Joints**: `plunger_joint`, `tip_eject_joint` (0.0-1.0 range)

## Testing

### Test Arduino Communication
```bash
ros2 run pipette_driver serial_terminal
```

### Test ROS2 Integration
```bash
# Terminal 1: Start driver
ros2 run pipette_driver pipette_driver_node

# Terminal 2: Send movement command
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['plunger_joint', 'tip_eject_joint'],
    points: [{positions: [0.55, 0.30], time_from_start: {sec: 2}}]
  }
}"
```

### Test Visualization
```bash
ros2 launch pipette_description pipette_driver_display.launch.py
```

## Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select pipette_driver
colcon build --packages-select pipette_description

# Source after building
source install/setup.bash
```

## Testing Commands

```bash
# Run all tests
colcon test

# Test specific package
colcon test --packages-select pipette_driver

# View test results
colcon test-result --verbose
```

## Configuration

### Serial Port Settings
```bash
# Change serial port (default: /tmp/ttyUR)
ros2 run pipette_driver pipette_driver_node --ros-args -p serial_port:=/dev/ttyUSB0

# Change baud rate (default: 115200)
ros2 run pipette_driver pipette_driver_node --ros-args -p baudrate:=9600
```

### Calibration Values

#### Plunger Positions
- **0.00** (P000): Home position - not pressed
- **0.55** (P055): Suction position
- **0.77** (P077): Expel position

#### Tip Ejection Positions
- **0.00** (T000): Home position - not pressed
- **0.30** (T030): Maximum eject position

## Robot Integration

This system is designed as a static end effector for Universal Robot arms:

- **Static URDF**: Pipette appears as fixed geometry in robot planning
- **External Control**: Pipette functions controlled independently via ROS2 actions
- **MoveIt Compatible**: Works with robot motion planning for collision avoidance

## Troubleshooting

### Serial Connection Issues
```bash
# Check available ports
ls /dev/tty*

# Test Arduino communication first
ros2 run pipette_driver serial_terminal

# Check permissions
sudo chmod 666 /dev/ttyUSB0
```

### Node Issues
```bash
# Verify package installation
ros2 pkg executables pipette_driver

# Check active nodes
ros2 node list

# Verify action server
ros2 action list | grep follow_joint_trajectory
```

## Related Documentation

- **Driver Package**: See `pipette_driver/README.md` for detailed hardware interface documentation
- **Description Package**: See `pipette_description/README.md` for URDF and visualization details
- **Arduino Firmware**: See `arduino_server/` for embedded control implementation

## License

[Add your license information here]