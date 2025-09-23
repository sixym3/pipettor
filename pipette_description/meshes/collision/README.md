# Collision Meshes

This directory should contain simplified 3D mesh files for collision detection.

## Purpose
- Fast collision detection during motion planning
- Safety boundaries for robot operation
- Should be simplified, low-polygon geometry

## File Format
- **Recommended**: STL (.stl) for fast processing
- **Geometry**: Convex hulls or simplified shapes preferred

## Expected Files
Place your simplified collision mesh files here:
- `pipette_body_collision.stl` - Main collision boundary
- `plunger_collision.stl` - Plunger collision volume
- `tip_eject_collision.stl` - Tip ejector collision

## Design Guidelines
- **Low polygon count** (< 1000 triangles per mesh)
- **Convex shapes** when possible for faster collision checking
- **Simplified geometry** - remove fine details
- **Safety margins** - slightly larger than visual mesh

## Current Status
**Empty** - Using placeholder cylinder geometry in URDF.

See `../README.md` for detailed instructions on adding collision meshes.