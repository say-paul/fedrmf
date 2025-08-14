# Open-RMF Fedora Packaging Usage Guide

## Overview

This system builds Fedora RPM packages for Open-RMF following the pattern of `~/git_repos/rosfed`. It automatically:

1. Downloads the `rmf.repos` file for the specified ROS distribution
2. Clones each repository from the list
3. Parses `package.xml` files to extract dependencies
4. Generates RPM spec files with proper Fedora package mappings
5. Builds RPM packages ready for installation

## Step-by-Step Usage

### 1. Initial Setup

```bash
# Navigate to the directory
cd /home/saypaul/git_repos/fedrmf

# Install build dependencies
make install-deps
```

### 2. Generate Spec Files

```bash
# For ROS Jazzy (default)
make generate-specs

# For other distributions
make generate-specs DISTRO=humble
make generate-specs DISTRO=rolling
```

### 3. Review Generated Specs

```bash
# List generated packages
make list-packages

# Check a specific spec file
less fedrmf/SPECS/ros-jazzy-rmf-utils.spec
```

### 4. Build RPM Packages

```bash
# Build all packages
make build-rpms

# Or build with specific distribution
make build-rpms DISTRO=jazzy
```

### 5. Install Packages

```bash
# Install all built packages
sudo dnf install fedrmf/RPMS/*/ros-jazzy-*.rpm

# Or install specific packages
sudo dnf install fedrmf/RPMS/x86_64/ros-jazzy-rmf-utils-*.rpm
```

## Package Mapping Examples

### ROS Dependencies → Fedora Packages

| ROS Package | Fedora Package |
|-------------|----------------|
| `ament_cmake` | `ros-jazzy-ament-cmake` |
| `rclcpp` | `ros-jazzy-rclcpp` |
| `geometry_msgs` | `ros-jazzy-geometry-msgs` |
| `rmf_utils` | `ros-jazzy-rmf-utils` |

### System Dependencies → Fedora Packages

| System Dependency | Fedora Package |
|-------------------|----------------|
| `eigen3` | `eigen3-devel` |
| `yaml-cpp` | `yaml-cpp-devel` |
| `nlohmann_json` | `json-devel` |
| `opencv` | `opencv-devel` |
| `qt5` | `qt5-qtbase-devel` |

## Generated Open-RMF Packages

After running `make generate-specs`, you'll have spec files for:

### Core RMF Packages
- `ros-jazzy-rmf-utils` - Core utilities and helper functions
- `ros-jazzy-rmf-traffic` - Traffic coordination and scheduling
- `ros-jazzy-rmf-task` - Task management and execution
- `ros-jazzy-rmf-battery` - Battery monitoring and management
- `ros-jazzy-rmf-ros2` - ROS 2 integration layer

### Message Definitions
- `ros-jazzy-rmf-api-msgs` - REST API message definitions
- `ros-jazzy-rmf-building-map-msgs` - Building map data structures
- `ros-jazzy-rmf-internal-msgs` - Internal communication messages
- `ros-jazzy-rmf-visualization-msgs` - Visualization data structures

### Tools and Editors
- `ros-jazzy-rmf-traffic-editor` - GUI for editing traffic maps
- `ros-jazzy-rmf-visualization` - Visualization tools and widgets

### Simulation Components
- `ros-jazzy-rmf-simulation` - Gazebo simulation plugins

### Third-party Vendors
- `ros-jazzy-menge-vendor` - Menge crowd simulation library
- `ros-jazzy-nlohmann-json-schema-validator-vendor` - JSON validation
- `ros-jazzy-pybind11-json-vendor` - Python JSON bindings

### Demonstrations
- `ros-jazzy-rmf-demos` - Example scenarios and tutorials

## Troubleshooting

### Common Issues

1. **Build fails with missing dependencies**
   ```bash
   # Install additional build dependencies
   sudo dnf install <missing-package>-devel
   ```

2. **ROS environment not found**
   ```bash
   # Install ROS 2 Jazzy
   sudo dnf install ros-jazzy-desktop
   ```

3. **Git clone failures**
   - Check internet connection
   - Verify repository URLs in `fedrmf/rmf.repos`

4. **Spec file generation errors**
   ```bash
   # Clean and regenerate
   make clean
   make generate-specs
   ```

### Debugging Commands

```bash
# Check Python dependencies
python3 -c "import yaml; print('YAML module available')"

# Verify directory structure
tree fedrmf/

# Check generated spec files
find fedrmf/SPECS -name "*.spec" | head -5

# Test single package build
make test-build
```

## Customization

### Adding New Dependency Mappings

Edit `build_rmf_fedora.py`:

```python
# In the __init__ method, add to ros_pkg_mappings:
'new_package': f'ros-{distro}-new-package'

# Or add to system_deps:
'library_name': 'fedora-package-name'
```

### Modifying Build Flags

Edit the `generate_spec_file` method in `build_rmf_fedora.py`:

```python
# Add custom CMAKE flags
spec_content += """
%cmake \\
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \\
    -DBUILD_TESTING=OFF \\
    -DCUSTOM_FLAG=ON \\
    -DCMAKE_INSTALL_PREFIX=/opt/ros/{distro}
"""
```

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: Build RMF Fedora Packages
on: [push, pull_request]
jobs:
  build:
    runs-on: fedora-latest
    steps:
    - uses: actions/checkout@v3
    - name: Install dependencies
      run: make install-deps
    - name: Generate specs
      run: make generate-specs
    - name: Build packages
      run: make build-rpms
    - name: Upload artifacts
      uses: actions/upload-artifact@v3
      with:
        name: rpm-packages
        path: fedrmf/RPMS/
```

## Performance Tips

1. **Parallel builds**: Modify Makefile to use multiple cores
2. **Selective building**: Build only specific packages by editing the spec list
3. **Caching**: Keep the `fedrmf/` directory between builds to reuse downloads

## Maintenance

### Updating to New RMF Releases

```bash
# Clean previous build
make clean

# Fetch latest rmf.repos for your distribution
make fetch-repos DISTRO=jazzy

# Regenerate specs with new versions
make generate-specs DISTRO=jazzy
```

### Version Updates

When Open-RMF releases new versions:

1. Update the `rmf.repos` URL in Makefile if needed
2. Regenerate specs to pick up version changes
3. Test build with new versions
4. Update dependency mappings if new packages are introduced

This system provides a complete, automated solution for packaging Open-RMF on Fedora, following established packaging patterns and best practices. 