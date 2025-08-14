# RMF Patches Guide

This document explains how to create, manage, and apply patches for Open-RMF packages in the Fedora packaging system.

## Overview

When building RMF packages, you may encounter compilation errors, missing dependencies, or Fedora-specific issues that require source code modifications. Instead of modifying the original source code, we use **patches** - files that describe the differences between the original code and the fixed code.

## When to Create Patches

Create patches when you encounter:

1. **Compilation Errors**: Missing headers, undefined symbols, compiler incompatibilities
2. **Fedora-specific Paths**: Hardcoded paths that don't match Fedora's file system layout
3. **Dependency Issues**: Missing or incorrect dependency specifications
4. **Library Compatibility**: Version mismatches or API changes
5. **Build System Issues**: CMake configuration problems

## Patch Naming Convention

Patches should follow this naming pattern:
```
<package_name>-<description>.patch
```

Examples:
- `rmf_utils-fedora-paths.patch`
- `rmf_traffic-gcc-compatibility.patch`
- `rmf_task-missing-headers.patch`
- `rmf_simulation-gazebo-api.patch`

## How to Create Patches

### Method 1: From Build Logs (Recommended)

1. **Analyze Build Logs**: When a package fails to build, examine the error messages:
   ```bash
   # Example error from build log
   /builddir/build/BUILD/rmf_utils-1.6.0/src/utils.cpp:25:10: 
   fatal error: memory: No such file or directory
   ```

2. **Identify the Fix**: Determine what needs to be changed (e.g., add missing header)

3. **Create the Patch**: 
   ```bash
   # Create a patch file
   cat > patches/rmf_utils-missing-headers.patch << 'EOF'
   --- a/src/utils.cpp
   +++ b/src/utils.cpp
   @@ -22,6 +22,7 @@
    #include <string>
    #include <vector>
    #include <iostream>
   +#include <memory>
    
    namespace rmf_utils {
   EOF
   ```

### Method 2: From Source Code Comparison

1. **Download and Extract Source**:
   ```bash
   cd /tmp
   wget https://github.com/open-rmf/rmf_utils/archive/refs/tags/1.6.0.tar.gz
   tar -xzf 1.6.0.tar.gz
   cp -r rmf_utils-1.6.0 rmf_utils-1.6.0.orig
   ```

2. **Make Your Changes**:
   ```bash
   cd rmf_utils-1.6.0
   # Edit files as needed
   vim src/utils.cpp  # Add missing #include <memory>
   ```

3. **Generate Patch**:
   ```bash
   cd /tmp
   diff -Naur rmf_utils-1.6.0.orig rmf_utils-1.6.0 > rmf_utils-missing-headers.patch
   ```

4. **Copy to Patches Directory**:
   ```bash
   cp rmf_utils-missing-headers.patch /path/to/fedrmf/patches/
   ```

## Patch Format

Patches use the unified diff format. Here's the structure:

```patch
--- a/path/to/original/file
+++ b/path/to/modified/file
@@ -line_start,line_count +line_start,line_count @@
 context line
-removed line
+added line
 context line
```

### Example Patch

```patch
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -12,6 +12,9 @@ find_package(ament_cmake REQUIRED)
 find_package(Eigen3 REQUIRED)
 find_package(ament_cmake_catch2 QUIET)
 
+# Fedora-specific: Use system Eigen3
+set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
+
 if(BUILD_TESTING AND ament_cmake_catch2_FOUND)
   find_package(ament_lint_auto REQUIRED)
   ament_lint_auto_find_test_dependencies()
```

## Adding Patches to Package Configuration

Once you create a patch, add it to the package's YAML configuration:

1. **Edit the Package YAML**:
   ```bash
   vim cfg/rmf_utils.yaml
   ```

2. **Add Patches Section**:
   ```yaml
   package_name: rmf_utils
   repository: https://github.com/open-rmf/rmf_utils
   version: "1.6.0"
   release_tag: "1.6.0"
   
   build_depends:
     - ament_cmake
     - eigen3
   
   # ... other configuration ...
   
   patches:
     - rmf_utils-fedora-paths.patch
     - rmf_utils-missing-headers.patch
   ```

## Common Patch Scenarios

### 1. Missing Headers
```patch
--- a/src/file.cpp
+++ b/src/file.cpp
@@ -18,6 +18,8 @@
 #include <string>
 #include <vector>
+#include <memory>
+#include <algorithm>
 
 namespace package {
```

### 2. Fedora Path Fixes
```patch
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -25,7 +25,7 @@ find_package(SomeLib REQUIRED)
 
 # Fix library paths for Fedora
-set(LIBRARY_PATH /usr/local/lib)
+set(LIBRARY_PATH /usr/lib64)
 
 target_link_libraries(${PROJECT_NAME}
```

### 3. Compiler Compatibility
```patch
--- a/include/package/header.hpp
+++ b/include/package/header.hpp
@@ -45,7 +45,7 @@ class SomeClass {
 private:
   // Fix for GCC compatibility
-  inline static const std::string DEFAULT_VALUE = "default";
+  static const std::string DEFAULT_VALUE;
 };
```

### 4. CMake Configuration
```patch
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -8,6 +8,9 @@ project(package_name)
 find_package(ament_cmake REQUIRED)
 find_package(SomeDep REQUIRED)
 
+# Fedora-specific configuration
+set(CMAKE_INSTALL_LIBDIR lib64)
+
 ament_package()
```

## Testing Patches

1. **Generate Spec with Patch**:
   ```bash
   make generate-spec-package PACKAGE=rmf_utils
   ```

2. **Build SRPM**:
   ```bash
   make build-srpm-package PACKAGE=rmf_utils
   ```

3. **Check Build Logs**:
   ```bash
   # If build fails, check the logs in fedrmf/BUILD/
   ```

## Patch Best Practices

### DO:
- ✅ Keep patches minimal and focused
- ✅ Add comments explaining the purpose
- ✅ Test patches before committing
- ✅ Use descriptive file names
- ✅ Include context lines for clarity

### DON'T:
- ❌ Create overly large patches
- ❌ Mix unrelated changes in one patch
- ❌ Hardcode version-specific paths
- ❌ Remove functionality without good reason
- ❌ Ignore upstream compatibility

## Patch Lifecycle

1. **Creation**: Identify issue → Create patch → Test locally
2. **Integration**: Add to YAML → Generate specs → Build SRPM
3. **Validation**: Test in COPR → Verify functionality
4. **Maintenance**: Monitor upstream changes → Update if needed
5. **Upstream**: Consider contributing fixes back to upstream

## Troubleshooting

### Patch Doesn't Apply
```bash
# Check patch format and paths
patch --dry-run -p1 < patches/package-name.patch

# Common issues:
# - Wrong path (should be relative to package root)
# - Line ending differences
# - Context doesn't match (upstream changed)
```

### Build Still Fails After Patch
```bash
# Check if patch was actually applied
rpmbuild -bp spec_file  # Only run %prep section
# Look in BUILD directory to verify changes
```

### Multiple Patches Conflict
```bash
# Ensure patches apply in order
# May need to update later patches to account for earlier ones
```

## Examples in This Repository

Check existing patches in this directory for examples:
- `rmf_utils-fedora-paths.patch` - Path configuration fixes
- `rmf_traffic-fedora-cmake.patch` - CMake and library issues  
- `rmf_task-gcc-compatibility.patch` - Compiler compatibility

## Getting Help

1. **Build Logs**: Always start with the actual error message
2. **Upstream Issues**: Check GitHub issues for known problems
3. **Fedora Packaging**: Consult Fedora packaging guidelines
4. **ROS Documentation**: Check ROS 2 building documentation

Remember: Patches are temporary fixes. Consider contributing solutions back to upstream repositories when possible. 