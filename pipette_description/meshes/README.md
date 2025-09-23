# Pipette Mesh Files

This directory contains the 3D mesh files for the pipette tool.

## Directory Structure

- `visual/` - High-quality meshes for visual representation (DAE/COLLADA format recommended)
- `collision/` - Simplified meshes for collision detection (STL format recommended)

## File Format Guidelines

### Visual Meshes (visual/ directory)
- **Format**: DAE/COLLADA (.dae) preferred for materials and colors
- **Quality**: High detail for realistic appearance in RViz
- **Materials**: Include color/texture information when possible

### Collision Meshes (collision/ directory)  
- **Format**: STL (.stl) for fast collision detection
- **Quality**: Simplified geometry (convex hulls, basic shapes)
- **Purpose**: Fast collision checking, not visual appearance

## Expected Files

Replace these placeholder descriptions with your actual CAD files:

### Visual Meshes
- `visual/pipette_body.dae` - Main pipette body with materials
- `visual/plunger_assembly.dae` - Plunger mechanism (optional)
- `visual/tip_holder.dae` - Tip holder assembly (optional)

### Collision Meshes
- `collision/pipette_body.stl` - Simplified main body collision
- `collision/pipette_full.stl` - Complete pipette collision hull

## CAD to Mesh Conversion Tips

1. **Simplify your CAD model** before export:
   - Remove internal components not visible
   - Combine small parts into single bodies
   - Use appropriate mesh resolution

2. **Export settings**:
   - Visual: Medium-high resolution, include materials
   - Collision: Low resolution, convex if possible

3. **File naming**: Use descriptive names matching URDF references

4. **Scale**: Ensure models are in meters (ROS standard)

## Current Status

Currently using basic geometric shapes (cylinders) as placeholders.
Replace the cylinder definitions in `pipette_macro.urdf.xacro` with mesh references when ready.

Example replacement:
```xml
<!-- Current placeholder -->
<cylinder length="${pipette_length}" radius="${pipette_radius}"/>

<!-- Replace with your mesh -->
<mesh filename="package://pipette_description/meshes/visual/pipette_body.dae"/>
```