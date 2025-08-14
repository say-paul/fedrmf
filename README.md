# Open-RMF Fedora Packaging

This repository provides Fedora RPM packaging for [Open-RMF](https://github.com/open-rmf/rmf) (Robotics Middleware Framework), following the pattern established by `~/git_repos/rosfed`.

## Overview

Open-RMF is a collection of packages for multi-fleet robot management. This build system:

1. Fetches the package list from `rmf.repos` for the specified ROS distro (default: jazzy)
2. Clones each repository and extracts dependencies from `package.xml`
3. Generates RPM spec files with proper Fedora package dependencies
4. Builds RPM packages for installation on Fedora systems

## Quick Start

```bash
# Clone and setup
git clone <this-repository>
cd fedrmf

# Generate spec files for ROS Jazzy (default)
make generate-specs

# Or specify a different ROS distribution
make generate-specs DISTRO=humble

# Build RPM packages
make build-rpms

# Install packages
sudo dnf install fedrmf/RPMS/x86_64/ros-jazzy-*.rpm
```

## rosfed-style Parameters

Following the `rosfed` pattern, this system supports these command-line parameters:

### Via Makefile
```bash
# Rebuild all packages, removing existing specs first
make generate-specs REBUILD=1

# Only build packages that don't already exist
make generate-specs ONLY_NEW=1

# Enable network access during build
make generate-specs ENABLE_NET=1

# Enable verbose output
make generate-specs VERBOSE=1

# Combine multiple parameters
make generate-specs REBUILD=1 ONLY_NEW=1 ENABLE_NET=1 VERBOSE=1 DISTRO=jazzy
```

### Via Python Script
```bash
# Basic usage
python3 build_rmf_fedora.py jazzy

# Rebuild all packages (-r parameter)
python3 build_rmf_fedora.py -r jazzy

# Only build new packages
python3 build_rmf_fedora.py --only-new jazzy

# Enable network during build
python3 build_rmf_fedora.py --enable-net jazzy

# Verbose output
python3 build_rmf_fedora.py --verbose jazzy

# Combine all parameters
python3 build_rmf_fedora.py -r --only-new --enable-net --verbose jazzy
```

### Parameter Descriptions

| Parameter | Short | Description |
|-----------|-------|-------------|
| `--rebuild` | `-r` | Rebuild all packages, removing existing spec files first |
| `--only-new` | | Only process packages that don't already have spec files and RPMs |
| `--enable-net` | | Enable network access during build (allows downloading dependencies) |
| `--verbose` | `-v` | Enable verbose output for debugging |
| `--list-existing` | | List existing packages and exit |

## Prerequisites

### System Requirements
- Fedora Linux (tested on Fedora 39+)
- Python 3.8+
- Git
- Internet connection for downloading repositories

### Install Build Dependencies
```bash
make install-deps
```

This installs:
- RPM build tools (`rpm-build`, `rpmdevtools`)
- Development tools (`cmake`, `gcc-c++`)
- Python dependencies (`python3-yaml`)
- System libraries (eigen3, yaml-cpp, etc.)

## Directory Structure

```
fedrmf/
├── build_rmf_fedora.py    # Main build script
├── Makefile               # Build automation
├── README.md             # This file
└── fedrmf/               # Build directory
    ├── SPECS/            # Generated RPM spec files
    ├── SOURCES/          # Source tarballs
    ├── BUILD/            # Build workspace
    ├── RPMS/             # Built RPM packages
    ├── SRPMS/            # Source RPM packages
    └── rmf.repos         # Downloaded package list
```

## Makefile Targets

| Target | Description |
|--------|-------------|
| `help` | Show available targets and options |
| `setup` | Create build directory structure |
| `fetch-repos` | Download rmf.repos file for specified distro |
| `generate-specs` | Generate RPM spec files from package.xml |
| `build-rpms` | Build all RPM packages |
| `install-deps` | Install build dependencies |
| `list-packages` | List generated packages |
| `list-existing` | List existing packages |
| `check-deps` | Check ROS environment |
| `clean` | Clean build artifacts |
| `show-config` | Show current configuration |

### Convenience Targets (rosfed-style)

| Target | Equivalent Command |
|--------|-------------------|
| `rebuild` | `make generate-specs REBUILD=1` |
| `only-new` | `make generate-specs ONLY_NEW=1` |
| `enable-net` | `make generate-specs ENABLE_NET=1` |
| `verbose` | `make generate-specs VERBOSE=1` |
| `rebuild-only-new` | `make generate-specs REBUILD=1 ONLY_NEW=1` |
| `rebuild-enable-net` | `make generate-specs REBUILD=1 ENABLE_NET=1` |
| `only-new-enable-net` | `make generate-specs ONLY_NEW=1 ENABLE_NET=1` |
| `rebuild-only-new-enable-net` | `make generate-specs REBUILD=1 ONLY_NEW=1 ENABLE_NET=1` |

## Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `DISTRO` | ROS distribution | `jazzy` |
| `ARCH` | Target architecture | `$(uname -m)` |
| `REBUILD` | Rebuild all packages | (empty) |
| `ONLY_NEW` | Only build new packages | (empty) |
| `ENABLE_NET` | Enable network during build | (empty) |
| `VERBOSE` | Enable verbose output | (empty) |

## Package Naming Convention

ROS packages follow the Fedora naming convention:
- ROS packages: `ros-<distro>-<package-name>`
- Example: `rmf_utils` → `ros-jazzy-rmf-utils`

System dependencies are mapped to Fedora package names:
- `eigen3` → `eigen3-devel`
- `yaml-cpp` → `yaml-cpp-devel`
- `nlohmann_json` → `json-devel`

## Supported ROS Distributions

- **Jazzy Jalisco** (default)
- **Humble Hawksbill** 
- **Rolling Ridley**

## Generated Packages

The build system processes repositories from `rmf.repos` and generates packages for:

### Core RMF Components
- `ros-jazzy-rmf-utils` - Core utilities
- `ros-jazzy-rmf-traffic` - Traffic management
- `ros-jazzy-rmf-task` - Task planning
- `ros-jazzy-rmf-battery` - Battery management
- `ros-jazzy-rmf-ros2` - ROS 2 integration

### Message Packages
- `ros-jazzy-rmf-api-msgs` - API messages
- `ros-jazzy-rmf-building-map-msgs` - Building map messages
- `ros-jazzy-rmf-internal-msgs` - Internal messages
- `ros-jazzy-rmf-visualization-msgs` - Visualization messages

### Simulation and Visualization
- `ros-jazzy-rmf-simulation` - Simulation components
- `ros-jazzy-rmf-visualization` - Visualization tools
- `ros-jazzy-rmf-traffic-editor` - Traffic editor

### Third-party Dependencies
- `ros-jazzy-menge-vendor` - Menge crowd simulation
- `ros-jazzy-nlohmann-json-schema-validator-vendor` - JSON schema validator
- `ros-jazzy-pybind11-json-vendor` - Python bindings

### Demonstrations
- `ros-jazzy-rmf-demos` - Demo applications and scenarios

## Usage Examples

### Basic Usage
```bash
# Generate all spec files
make generate-specs

# Generate for specific distribution
make generate-specs DISTRO=humble
```

### Using rosfed-style Parameters
```bash
# Rebuild everything from scratch
make generate-specs REBUILD=1

# Only build packages that don't exist yet
make generate-specs ONLY_NEW=1

# Enable network for packages that need to download dependencies
make generate-specs ENABLE_NET=1

# Verbose output for debugging
make generate-specs VERBOSE=1

# Combination: rebuild only new packages with network and verbose
make generate-specs REBUILD=1 ONLY_NEW=1 ENABLE_NET=1 VERBOSE=1
```

### Convenience Commands
```bash
# Quick rebuild
make rebuild

# Process only new packages
make only-new

# Enable network
make enable-net

# Show current configuration
make show-config
```

## Development

### Adding New Dependencies

Edit `build_rmf_fedora.py` to add mappings:

```python
# For ROS packages
self.ros_pkg_mappings = {
    'new_ros_package': f'ros-{distro}-new-ros-package',
}

# For system packages
self.system_deps = {
    'new_system_lib': 'fedora-package-name',
}
```

### Customizing Spec Files

The generated spec files can be manually edited in `fedrmf/SPECS/` before building.

### Testing Single Packages

```bash
# Test build of a specific package
make test-build
```

## Troubleshooting

### Build Failures
1. Check build logs in `fedrmf/BUILD/`
2. Verify dependencies are installed
3. Ensure ROS environment is sourced

### Missing Dependencies
```bash
# Check for missing ROS packages
rosdep check --from-paths /opt/ros/jazzy/share --ignore-src

# Install missing system dependencies
sudo dnf install <missing-package>
```

### Clean Build
```bash
make clean
make generate-specs
```

### Debug Mode
```bash
# Use verbose output to see detailed information
make generate-specs VERBOSE=1

# List existing packages
make list-existing
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes to build scripts or mappings
4. Test with `make test-build`
5. Submit a pull request

## License

This packaging system is licensed under Apache-2.0, same as Open-RMF.

## Related Projects

- [Open-RMF](https://github.com/open-rmf/rmf) - Main Open-RMF repository
- [RMF Demos](https://github.com/open-rmf/rmf_demos) - Open-RMF demonstrations
- [rosfed](~/git_repos/rosfed) - Pattern repository for ROS Fedora packaging 