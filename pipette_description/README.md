# Pipette Description

Static URDF description package for a pipette tool with geometry visualization and RViz integration.

## Overview

This package provides a URDF model of a pipette tool designed for integration with Universal Robot arms. The model includes visual and collision geometry for the pipette as a static end effector.

### Features
- **Static URDF Description**: Clean robot description with fixed geometry only
- **Visual Model**: Pipette representation with STL meshes (easily replaceable with CAD)
- **LED Visualization**: Visual representation of LED strip
- **RViz Integration**: Complete visualization support
- **UR Robot Integration Ready**: Designed to attach to robot flanges
- **External Driver Compatible**: Works with separate pipette control driver

## Package Structure

```
pipette_description/
├── urdf/
│   ├── pipette.urdf.xacro          # Main entry point
│   └── pipette_macro.urdf.xacro    # Core pipette macro definition
├── meshes/
│   ├── visual/                     # High-quality visual meshes (your CAD here)
│   ├── collision/                  # Simplified collision meshes
│   └── README.md                   # CAD integration instructions
├── launch/
│   └── pipette_driver_display.launch.py  # Visualization + driver launch file
└── rviz/
    ├── pipette_demo.rviz           # RViz configuration for pipette display
    └── urdf.rviz                   # Basic URDF visualization
```

## Quick Start

### Prerequisites
```bash
# Ensure dependencies are installed
sudo apt install ros-humble-urdf-launch ros-humble-joint-state-publisher-gui
```

### Build the Package
```bash
cd /path/to/your/workspace
colcon build --packages-select pipette_description
source install/setup.bash
```

## Usage

### URDF Visualization
Test the URDF structure and visualization:

```bash
# Launch RViz with pipette model
ros2 launch pipette_description pipette_driver_display.launch.py

# Expected behavior:
# - RViz opens with pipette model displayed
# - Static pipette geometry attached to robot flange
# - No moving joints (pipette is static end effector)
```

### Launch Arguments
- **`serial_port`** - Arduino serial port (default: `/dev/ttyUR`)
- **`baudrate`** - Serial communication baud rate (default: `115200`)
- **`start_driver`** - Whether to start pipette driver node (default: `true`)
- **`start_rviz`** - Whether to start RViz visualization (default: `true`)

## URDF Parameters

The pipette URDF accepts these xacro parameters:

```xml
<!-- Mounting configuration -->
<xacro:arg name="parent" default="tool0"/>

<!-- Physical properties -->
<xacro:arg name="pipette_mass" default="0.200"/>  # kg
<xacro:arg name="xyz" default="0 0 0"/>           # Position offset
<xacro:arg name="rpy" default="0 0 0"/>           # Orientation offset
```

## Link Structure

```
parent (e.g., tool0)
└── pipette_base_link
    ├── pipette_body_link (main visual/collision geometry)
    ├── pipette_tip_link (end effector reference)
    └── led_strip_link (visual only)
```

### Coordinate System
- **Base Frame**: `pipette_base_link` at tool attachment point
- **End Effector**: `pipette_tip_link` at pipette tip
- **Body Geometry**: `pipette_body_link` contains main pipette shape

## Architecture Notes

### Static End Effector Design
This package provides a static URDF model:

- **No movable joints**: Pipette plunger/tip ejection handled by external driver
- **Pure geometry**: Visual and collision shapes for path planning
- **External control**: Pipette functions controlled via separate driver node
- **MoveIt compatible**: Works as fixed end effector for collision checking

### Integration with Larger Systems

#### Include in UR Robot URDF
```xml
<!-- Include in UR robot URDF -->
<xacro:include filename="$(find pipette_description)/urdf/pipette_macro.urdf.xacro"/>

<!-- Attach to robot tool flange -->
<xacro:pipette 
    name="pipette"
    prefix=""
    parent="tool0" 
    pipette_mass="0.200"
    xyz="0 0 0"
    rpy="0 0 0"/>
```

#### Used by MoveIt Configuration
This package is referenced by:
- `ur5e_pipette_robot_description` - Combined UR5e + pipette system
- `ur5e_pipette_moveit_config` - MoveIt configuration with static pipette

## CAD Integration

### Current State: Placeholder Geometry
The URDF currently uses STL mesh files for visual and collision geometry.

### Integrating Your CAD Files

1. **Export your CAD model**:
   - **Visual mesh**: Export as STL with appropriate detail
   - **Collision mesh**: Export as simplified STL for better performance

2. **Place mesh files**:
   ```bash
   # Replace existing files:
   meshes/visual/pipettor_visual.stl      # Visual representation
   meshes/collision/pipettor_collision.stl # Collision detection
   ```

3. **Scale adjustment**: Mesh files use `scale="0.001 0.001 0.001"` (mm to m conversion)

4. **Test the updated model**:
   ```bash
   ros2 launch pipette_description pipette_driver_display.launch.py
   ```

## Testing and Validation

### Test URDF Syntax
```bash
# Validate URDF structure
ros2 run xacro xacro src/custom-ur-descriptions/pipette_description/urdf/pipette.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf

# Check for syntax errors
ros2 launch pipette_description pipette_driver_display.launch.py
```

## Troubleshooting

### RViz Not Showing Model
```bash
# Check URDF processing
ros2 run xacro xacro src/custom-ur-descriptions/pipette_description/urdf/pipette.urdf.xacro

# Check robot_description topic
ros2 topic echo /robot_description --once

# Verify RViz configuration
# - Add RobotModel display
# - Set Robot Description topic to /robot_description
# - Set Fixed Frame to tool0 or world
```

### Missing Mesh Files
```bash
# Check mesh file paths
ls src/custom-ur-descriptions/pipette_description/meshes/visual/
ls src/custom-ur-descriptions/pipette_description/meshes/collision/

# Verify mesh files are readable
# URDF will fall back to collision geometry if visual mesh fails
```

## Related Packages

This package works with:
- **`pipette_driver`** - Hardware control and communication interfaces
- **`ur5e_pipette_robot_description`** - Combined UR5e + pipette description  
- **`ur5e_pipette_moveit_config`** - Complete system MoveIt configuration

## Development Notes

### Modifying the URDF
1. **Edit** `urdf/pipette_macro.urdf.xacro` for structural changes
2. **Test syntax** with `xacro` and `check_urdf`
3. **Verify visualization** with RViz launch
4. **Update related packages** if link names or attachment points change

### Performance Tips
- Use simplified collision meshes for better performance
- Keep visual meshes under 10k triangles when possible
- Test visualization before integrating with robot systems