# RMF Package Dependencies and Build Order

This document shows the dependency relationships and build order for all RMF packages based on the individual YAML configuration files.

## Build Order (by Priority)

### Priority 1 (Foundation packages - no RMF dependencies)
1. **ament_cmake_catch2** - Testing framework wrapper
2. **rmf_utils** - Core utilities (depends on eigen3)
3. **rmf_api_msgs** - API message definitions
4. **rmf_building_map_msgs** - Building map messages
5. **rmf_visualization_msgs** - Visualization messages
6. **menge_vendor** - Crowd simulation vendor package
7. **nlohmann_json_schema_validator_vendor** - JSON schema validation vendor
8. **pybind11_json_vendor** - Python JSON bindings vendor

### Priority 2 (Core functionality)
9. **rmf_traffic** - Depends on: rmf_utils, eigen3, yaml-cpp
10. **rmf_battery** - Depends on: rmf_utils
11. **rmf_internal_msgs** - Depends on: rmf_building_map_msgs

### Priority 3 (Advanced functionality)
12. **rmf_task** - Depends on: rmf_utils, rmf_traffic, rmf_battery, nlohmann_json_schema_validator_vendor
13. **rmf_traffic_editor** - Depends on: rmf_building_map_msgs, rmf_utils, Qt5, OpenCV

### Priority 4 (Integration layers)
14. **rmf_ros2** - Depends on: rmf_utils, rmf_traffic, rmf_task, rmf_battery, rmf_building_map_msgs, rmf_internal_msgs
15. **rmf_visualization** - Depends on: rmf_building_map_msgs, rmf_visualization_msgs, rmf_traffic, Qt5

### Priority 5 (Simulation)
16. **rmf_simulation** - Depends on: rmf_building_map_msgs, rmf_ros2, Gazebo

### Priority 6 (Demonstrations)
17. **rmf_demos** - Depends on: Most RMF packages

## Dependency Graph

```
rmf_utils (foundation)
├── rmf_traffic
│   ├── rmf_task
│   │   └── rmf_ros2
│   │       ├── rmf_simulation
│   │       └── rmf_demos
│   └── rmf_visualization
│       └── rmf_demos
├── rmf_battery
│   ├── rmf_task (already shown above)
│   └── rmf_ros2 (already shown above)
└── rmf_traffic_editor
    └── rmf_demos

rmf_building_map_msgs (foundation)
├── rmf_internal_msgs
│   └── rmf_ros2 (already shown above)
├── rmf_traffic_editor (already shown above)
├── rmf_visualization (already shown above)
└── rmf_simulation (already shown above)

rmf_api_msgs (foundation)
rmf_visualization_msgs (foundation)

Vendor packages (foundation):
├── ament_cmake_catch2
├── menge_vendor
├── nlohmann_json_schema_validator_vendor
│   └── rmf_task (already shown above)
└── pybind11_json_vendor
```

## Package Categories

### Message Packages (Priority 1)
- rmf_api_msgs
- rmf_building_map_msgs
- rmf_visualization_msgs
- rmf_internal_msgs

### Vendor Packages (Priority 1)
- ament_cmake_catch2
- menge_vendor
- nlohmann_json_schema_validator_vendor
- pybind11_json_vendor

### Core Libraries (Priority 1-2)
- rmf_utils
- rmf_traffic
- rmf_battery

### Task Management (Priority 3)
- rmf_task

### ROS Integration (Priority 4)
- rmf_ros2

### Visualization & GUI (Priority 3-4)
- rmf_traffic_editor (Qt-based)
- rmf_visualization (Qt-based)

### Simulation (Priority 5)
- rmf_simulation (Gazebo-based)

### Demonstrations (Priority 6)
- rmf_demos

## System Dependencies by Package

### Qt-based packages
- rmf_traffic_editor: qt5-qtbase-devel, qt5-qtsvg-devel, qt5-qtopengl-devel
- rmf_visualization: qt5-qtbase-devel, qt5-qtwidgets-devel, qt5-qtsvg-devel

### Math/Computation packages
- rmf_utils: eigen3-devel
- rmf_traffic: eigen3-devel, yaml-cpp-devel

### Simulation packages
- rmf_simulation: gazebo-devel, sdformat-devel

### Vendor packages
- menge_vendor: tinyxml-devel, opengl-devel
- nlohmann_json_schema_validator_vendor: json-devel
- pybind11_json_vendor: pybind11-devel, json-devel, python3-devel

## Special Build Considerations

### Network-enabled builds
Some packages may need `--enable-net` flag for downloading external dependencies:
- Vendor packages (menge_vendor, nlohmann_json_schema_validator_vendor, pybind11_json_vendor)

### GUI packages
Qt-based packages may need special handling in headless environments:
- rmf_traffic_editor
- rmf_visualization

### Large packages
Packages with extensive dependencies that take longer to build:
- rmf_demos (depends on most other packages)
- rmf_ros2 (complex ROS integration)

This dependency structure ensures that packages are built in the correct order, with all dependencies available when needed. 