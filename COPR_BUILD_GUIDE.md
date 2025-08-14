# COPR Build Guide for Open-RMF Fedora Packaging

This guide explains how to use the COPR (Community Build System) and chroot functionality in the Open-RMF Fedora packaging system, following the `rosfed` pattern.

## Overview

The system now supports:
- **COPR repository targeting** - Build packages in remote COPR repositories
- **Chroot specification** - Target specific Fedora versions and architectures  
- **Dependency-based build order** - Automatic build ordering based on package dependencies
- **Mock integration** - Local chroot builds using mock

## COPR Integration

### What is COPR?
COPR (Community Build System) is Fedora's build system that allows users to create and maintain package repositories. It's similar to Ubuntu's PPA system.

### Setup COPR Access

1. **Install COPR tools:**
```bash
make install-copr-deps
```

2. **Configure COPR authentication:**
```bash
copr-cli login
# Follow the instructions to get your API token from https://copr.fedorainfracloud.org/api/
```

3. **Verify COPR setup:**
```bash
make check-copr
```

### Building in COPR

#### Basic COPR Build
```bash
# Build all packages in a COPR repository
make copr-build COPR_REPO=username/repository-name

# Example: Build in user 'johndoe' repository 'rmf-packages'
make copr-build COPR_REPO=johndoe/rmf-packages
```

#### COPR Build with Options
```bash
# Rebuild all packages
make copr-build COPR_REPO=johndoe/rmf-packages REBUILD=1

# Only build new packages
make copr-build COPR_REPO=johndoe/rmf-packages ONLY_NEW=1

# Target specific chroot
make copr-build COPR_REPO=johndoe/rmf-packages CHROOT=fedora-39-x86_64

# Verbose output
make copr-build COPR_REPO=johndoe/rmf-packages VERBOSE=1
```

#### Test Single Package COPR Build
```bash
make test-copr-build COPR_REPO=johndoe/rmf-packages
```

## Chroot Management

### Understanding Chroots
A chroot is an isolated build environment for a specific Fedora version and architecture combination.

### Default Chroot Detection
The system automatically detects your current Fedora version and architecture:
```bash
# Show detected chroot
make show-config
# Output: CHROOT: fedora-42-x86_64
```

### Targeting Specific Chroots
```bash
# Target Fedora 39 x86_64
make generate-specs CHROOT=fedora-39-x86_64

# Target Fedora 40 aarch64
make generate-specs CHROOT=fedora-40-aarch64

# Multiple distributions
make copr-build COPR_REPO=user/repo CHROOT=fedora-39-x86_64
make copr-build COPR_REPO=user/repo CHROOT=fedora-40-x86_64
```

### Available Chroots
Common chroot targets:
- `fedora-39-x86_64` - Fedora 39 on x86_64
- `fedora-40-x86_64` - Fedora 40 on x86_64  
- `fedora-41-x86_64` - Fedora 41 on x86_64
- `fedora-42-x86_64` - Fedora 42 (Rawhide) on x86_64
- `fedora-39-aarch64` - Fedora 39 on ARM64
- `fedora-40-aarch64` - Fedora 40 on ARM64

## Build Order Creation

### Dependency Graph Analysis
The system automatically analyzes package dependencies to create optimal build order:

```bash
# Create build order based on dependencies
make build-order DISTRO=jazzy

# View existing build order
make show-build-order
```

### How Build Order Works

1. **Dependency Discovery**: Parses all `package.xml` files to extract dependencies
2. **Graph Construction**: Creates a directed graph of package dependencies
3. **Topological Sort**: Orders packages so dependencies are built first
4. **Cycle Detection**: Identifies and reports circular dependencies

### Build Order Output
```json
{
  "distro": "jazzy",
  "chroot": "fedora-42-x86_64", 
  "copr_repo": "user/rmf-packages",
  "build_order": [
    "rmf_utils",
    "rmf_traffic", 
    "rmf_battery",
    "rmf_task",
    "rmf_ros2",
    "rmf_demos"
  ],
  "dependency_graph": {
    "nodes": ["rmf_utils", "rmf_traffic", ...],
    "edges": [["rmf_utils", "rmf_traffic"], ...]
  }
}
```

## Mock Local Builds

### Setup Mock
```bash
# Install mock and dependencies
make install-copr-deps

# Check mock configuration
make check-mock
```

### Local Mock Builds
```bash
# Build all packages locally with mock
make mock-build

# Target specific chroot
make mock-build CHROOT=fedora-39-x86_64

# Rebuild all packages
make mock-build REBUILD=1

# Only build new packages
make mock-build ONLY_NEW=1
```

### Test Mock Build
```bash
make test-mock-build CHROOT=fedora-39-x86_64
```

## Advanced Usage

### Custom Mock Configuration
```bash
# Use custom mock config file
make mock-build MOCK_CONFIG=/path/to/custom.cfg

# Generate and customize mock config
make generate-specs CHROOT=fedora-39-x86_64
# Edit fedrmf/fedora-39-x86_64.cfg
make mock-build MOCK_CONFIG=fedrmf/fedora-39-x86_64.cfg
```

### Pipeline Builds
```bash
# Complete pipeline: order -> specs -> COPR build
make build-order DISTRO=jazzy
make generate-specs DISTRO=jazzy VERBOSE=1
make copr-build COPR_REPO=user/rmf CHROOT=fedora-39-x86_64
```

### Multi-Architecture Builds
```bash
# Build for multiple architectures
for arch in x86_64 aarch64; do
  make copr-build COPR_REPO=user/rmf CHROOT=fedora-39-$arch
done
```

### Multi-Distribution Support
```bash
# Build for multiple ROS distributions
for distro in humble jazzy; do
  make copr-build COPR_REPO=user/rmf-$distro DISTRO=$distro
done
```

## rosfed-style Command Examples

### Direct Python Script Usage

#### Basic Usage
```bash
# Generate specs with COPR target
python3 build_rmf_fedora.py --copr-repo user/rmf jazzy

# Create build order only
python3 build_rmf_fedora.py --build-order-only jazzy

# Load existing build order
python3 build_rmf_fedora.py --load-build-order jazzy
```

#### Advanced Combinations
```bash
# Full rosfed-style command
python3 build_rmf_fedora.py \
  -r \
  --only-new \
  --enable-net \
  --verbose \
  --copr-repo user/rmf-packages \
  --chroot fedora-39-x86_64 \
  jazzy

# Mock build with custom config
python3 build_rmf_fedora.py \
  --chroot fedora-40-x86_64 \
  --mock-config /etc/mock/custom.cfg \
  jazzy
```

### Makefile Convenience Targets

#### COPR Shortcuts
```bash
make copr-setup              # Install deps and check config
make copr-rebuild            # Rebuild all in COPR
make copr-only-new          # Build only new in COPR
```

#### Mock Shortcuts  
```bash
make mock-setup             # Install deps and check config
make mock-rebuild           # Rebuild all with mock
make mock-only-new         # Build only new with mock
```

#### Configuration
```bash
make show-config            # Show all current settings
make check-copr            # Verify COPR setup
make check-mock            # Verify mock setup
```

## Troubleshooting

### COPR Build Issues

1. **Authentication Errors**
```bash
# Re-authenticate with COPR
copr-cli login
```

2. **Repository Not Found**
```bash
# Create repository first
copr-cli create rmf-packages --chroot fedora-39-x86_64
```

3. **Build Failures**
```bash
# Check build logs in COPR web interface
# Use verbose mode for detailed output
make copr-build COPR_REPO=user/repo VERBOSE=1
```

### Mock Build Issues

1. **Permission Errors**
```bash
# Add user to mock group
sudo usermod -a -G mock $USER
# Log out and back in
```

2. **Chroot Not Found**
```bash
# List available chroots
ls /etc/mock/*.cfg
```

3. **Network Issues**
```bash
# Enable network in build
make mock-build ENABLE_NET=1
```

### Dependency Issues

1. **Circular Dependencies**
```bash
# Check build order output for cycles
make build-order VERBOSE=1
```

2. **Missing Dependencies**
```bash
# Check package.xml files
# Add missing dependencies to ros_pkg_mappings
```

## Configuration Files

The system generates several configuration files:

- `fedrmf/build_order.json` - Dependency-based build order
- `fedrmf/copr_config.json` - COPR repository configuration  
- `fedrmf/fedora-{ver}-{arch}.cfg` - Mock chroot configuration

## Best Practices

1. **Use Build Order**: Always create dependency-based build order first
2. **Target Specific Chroots**: Be explicit about target Fedora versions
3. **Test Locally First**: Use mock builds before COPR submission
4. **Incremental Builds**: Use `--only-new` for efficient rebuilds
5. **Monitor COPR Limits**: Be aware of COPR build time and storage limits

This system provides complete rosfed-style functionality for packaging Open-RMF on Fedora, with full COPR integration and sophisticated dependency management. 