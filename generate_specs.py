#!/usr/bin/env python3
"""
Generate RPM spec files from YAML configurations using Jinja2 templates
Similar to rosfed's approach
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
from datetime import datetime
from jinja2 import Environment, FileSystemLoader, Template
import glob

class SpecGenerator:
    def __init__(self, distro='jazzy', template_dir='templates', cfg_dir='cfg', output_dir='specs_generated', patches_dir='patches'):
        self.distro = distro
        self.template_dir = Path(template_dir)
        self.cfg_dir = Path(cfg_dir)
        self.output_dir = Path(output_dir)
        self.patches_dir = Path(patches_dir)
        
        # Ensure output directory exists
        self.output_dir.mkdir(exist_ok=True)
        
        # Setup Jinja2 environment
        self.jinja_env = Environment(
            loader=FileSystemLoader(str(self.template_dir)),
            trim_blocks=True,
            lstrip_blocks=True
        )
        
        # Add custom functions to Jinja2 environment
        self.jinja_env.globals['resolve_dependency'] = self.resolve_dependency
        self.jinja_env.globals['build_date'] = datetime.now().strftime('%a %b %d %Y')
        
        # ROS package mappings
        self.ros_mappings = {
            'ament_cmake': f'ros-{distro}-ament-cmake',
            'ament_cmake_auto': f'ros-{distro}-ament-cmake-auto',
            'ament_cmake_catch2': f'ros-{distro}-ament-cmake-catch2',
            'ament_cmake_gtest': f'ros-{distro}-ament-cmake-gtest',
            'ament_cmake_pytest': f'ros-{distro}-ament-cmake-pytest',
            'ament_cmake_python': f'ros-{distro}-ament-cmake-python',
            'ament_cmake_ros': f'ros-{distro}-ament-cmake-ros',
            'ament_cmake_vendor_package': f'ros-{distro}-ament-cmake-vendor-package',
            'ament_index_cpp': f'ros-{distro}-ament-index-cpp',
            'ament_index_python': f'ros-{distro}-ament-index-python',
            'ament_lint_auto': f'ros-{distro}-ament-lint-auto',
            'ament_lint_common': f'ros-{distro}-ament-lint-common',
            'builtin_interfaces': f'ros-{distro}-builtin-interfaces',
            'geometry_msgs': f'ros-{distro}-geometry-msgs',
            'nav_msgs': f'ros-{distro}-nav-msgs',
            'sensor_msgs': f'ros-{distro}-sensor-msgs',
            'std_msgs': f'ros-{distro}-std-msgs',
            'std_srvs': f'ros-{distro}-std-srvs',
            'rclcpp': f'ros-{distro}-rclcpp',
            'rclcpp_action': f'ros-{distro}-rclcpp-action',
            'rclcpp_components': f'ros-{distro}-rclcpp-components',
            'rclcpp_lifecycle': f'ros-{distro}-rclcpp-lifecycle',
            'rclpy': f'ros-{distro}-rclpy',
            'rcutils': f'ros-{distro}-rcutils',
            'rmw': f'ros-{distro}-rmw',
            'rmw_implementation': f'ros-{distro}-rmw-implementation',
            'rosidl_default_generators': f'ros-{distro}-rosidl-default-generators',
            'rosidl_default_runtime': f'ros-{distro}-rosidl-default-runtime',
            'tf2': f'ros-{distro}-tf2',
            'tf2_ros': f'ros-{distro}-tf2-ros',
            'tf2_geometry_msgs': f'ros-{distro}-tf2-geometry-msgs',
            'gazebo_ros': f'ros-{distro}-gazebo-ros',
            'gazebo_plugins': f'ros-{distro}-gazebo-plugins',
            'gazebo_msgs': f'ros-{distro}-gazebo-msgs',
            'urdf': f'ros-{distro}-urdf',
            'xacro': f'ros-{distro}-xacro',
            'robot_state_publisher': f'ros-{distro}-robot-state-publisher',
            'joint_state_publisher': f'ros-{distro}-joint-state-publisher',
            # RMF packages
            'rmf_utils': f'ros-{distro}-rmf-utils',
            'rmf_traffic': f'ros-{distro}-rmf-traffic',
            'rmf_task': f'ros-{distro}-rmf-task',
            'rmf_battery': f'ros-{distro}-rmf-battery',
            'rmf_ros2': f'ros-{distro}-rmf-ros2',
            'rmf_api_msgs': f'ros-{distro}-rmf-api-msgs',
            'rmf_building_map_msgs': f'ros-{distro}-rmf-building-map-msgs',
            'rmf_internal_msgs': f'ros-{distro}-rmf-internal-msgs',
            'rmf_visualization_msgs': f'ros-{distro}-rmf-visualization-msgs',
            'rmf_traffic_editor': f'ros-{distro}-rmf-traffic-editor',
            'rmf_visualization': f'ros-{distro}-rmf-visualization',
            'rmf_simulation': f'ros-{distro}-rmf-simulation',
            'rmf_demos': f'ros-{distro}-rmf-demos',
            'menge_vendor': f'ros-{distro}-menge-vendor',
            'nlohmann_json_schema_validator_vendor': f'ros-{distro}-nlohmann-json-schema-validator-vendor',
            'pybind11_json_vendor': f'ros-{distro}-pybind11-json-vendor',
        }
        
        # System package mappings
        self.system_mappings = {
            'eigen3': 'eigen3-devel',
            'yaml-cpp': 'yaml-cpp-devel',
            'opencv': 'opencv-devel',
            'qt5': 'qt5-qtbase-devel',
            'boost': 'boost-devel',
            'openssl': 'openssl-devel',
            'sqlite3': 'sqlite-devel',
            'curl': 'libcurl-devel',
            'pybind11': 'pybind11-devel',
            'json': 'json-devel',
            'cmake': 'cmake',
            'gcc-c++': 'gcc-c++',
            'pkgconfig': 'pkgconfig',
            'python3-devel': 'python3-devel',
            'python3-yaml': 'python3-yaml',
            'python3-requests': 'python3-requests',
            'python3-shapely': 'python3-shapely',
            'tinyxml-devel': 'tinyxml-devel',
            'opengl-devel': 'mesa-libGL-devel',
            'gazebo-devel': 'gazebo-devel',
            'sdformat-devel': 'sdformat-devel',
        }

    def resolve_dependency(self, dep_name, distro):
        """Resolve a dependency name to Fedora package name"""
        # Check ROS mappings first
        if dep_name in self.ros_mappings:
            return self.ros_mappings[dep_name]
        
        # Check system mappings
        if dep_name in self.system_mappings:
            return self.system_mappings[dep_name]
        
        # Default: assume it's a ROS package
        if not dep_name.startswith('ros-'):
            return f'ros-{distro}-{dep_name.replace("_", "-")}'
        
        return dep_name

    def find_patches(self, package_name):
        """Find patches for a given package in the patches directory"""
        patches = []
        if not self.patches_dir.exists():
            return patches
        
        # Look for patches with package name prefix
        patch_patterns = [
            f"{package_name}-*.patch",
            f"{package_name.replace('_', '-')}-*.patch"
        ]
        
        for pattern in patch_patterns:
            patch_files = list(self.patches_dir.glob(pattern))
            for patch_file in sorted(patch_files):
                patches.append(patch_file.name)
        
        return patches

    def load_package_config(self, yaml_file):
        """Load package configuration from YAML file"""
        try:
            with open(yaml_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Auto-detect patches for this package
            package_name = config.get('package_name')
            if package_name:
                patches = self.find_patches(package_name)
                if patches:
                    config['patches'] = patches
                    print(f"  Found patches for {package_name}: {', '.join(patches)}")
            
            return config
        except Exception as e:
            print(f"Error loading {yaml_file}: {e}")
            return None

    def generate_spec_file(self, pkg_config, template_name='pkg.spec.j2'):
        """Generate spec file from package configuration"""
        try:
            template = self.jinja_env.get_template(template_name)
            
            # Add distro to config for template
            pkg_config['distro'] = self.distro
            pkg_config['ros_distro'] = self.distro
            
            # Set default values if missing
            if 'version' not in pkg_config:
                pkg_config['version'] = '0.0.0'
            if 'release' not in pkg_config:
                pkg_config['release'] = '1'
            if 'license' not in pkg_config:
                pkg_config['license'] = 'Apache-2.0'
            
            # Handle special package types
            if pkg_config.get('vendor_package'):
                pkg_config['enable_net'] = True
            
            # Generate spec content
            spec_content = template.render(**pkg_config)
            
            return spec_content
            
        except Exception as e:
            print(f"Error generating spec for {pkg_config.get('package_name', 'unknown')}: {e}")
            return None

    def copy_patches_to_sources(self, pkg_config):
        """Copy patches to SOURCES directory if they exist"""
        patches = pkg_config.get('patches', [])
        if not patches:
            return
        
        sources_dir = Path('fedrmf/SOURCES')
        sources_dir.mkdir(parents=True, exist_ok=True)
        
        for patch in patches:
            patch_src = self.patches_dir / patch
            patch_dst = sources_dir / patch
            
            if patch_src.exists() and patch_src.stat().st_size > 13:  # Skip empty patches
                try:
                    import shutil
                    shutil.copy2(patch_src, patch_dst)
                    print(f"  Copied patch: {patch}")
                except Exception as e:
                    print(f"  Warning: Failed to copy patch {patch}: {e}")
            else:
                print(f"  Warning: Skipping empty or non-existent patch: {patch}")

    def generate_all_specs(self):
        """Generate spec files for all packages in cfg directory"""
        yaml_files = glob.glob(str(self.cfg_dir / '*.yaml'))
        
        # Filter out special files
        yaml_files = [f for f in yaml_files if not os.path.basename(f).startswith('_') 
                     and not os.path.basename(f) in ['rmf_deps.yaml', 'files.yaml']]
        
        generated_count = 0
        failed_count = 0
        
        print(f"Generating spec files from {len(yaml_files)} YAML configurations...")
        
        for yaml_file in sorted(yaml_files):
            pkg_config = self.load_package_config(yaml_file)
            if not pkg_config:
                failed_count += 1
                continue
                
            package_name = pkg_config.get('package_name')
            if not package_name:
                print(f"No package_name in {yaml_file}")
                failed_count += 1
                continue
            
            print(f"Generating spec for {package_name}...")
            
            # Copy patches to SOURCES directory
            self.copy_patches_to_sources(pkg_config)
            
            spec_content = self.generate_spec_file(pkg_config)
            if not spec_content:
                failed_count += 1
                continue
            
            # Write spec file
            spec_filename = f"ros-{self.distro}-{package_name.replace('_', '-')}.spec"
            spec_path = self.output_dir / spec_filename
            
            try:
                with open(spec_path, 'w') as f:
                    f.write(spec_content)
                print(f"  Generated: {spec_path}")
                generated_count += 1
            except Exception as e:
                print(f"  Error writing {spec_path}: {e}")
                failed_count += 1
        
        print(f"\nGeneration complete:")
        print(f"  Generated: {generated_count} spec files")
        print(f"  Failed: {failed_count} packages")
        print(f"  Output directory: {self.output_dir}")

    def generate_single_spec(self, package_name):
        """Generate spec file for a single package"""
        yaml_file = self.cfg_dir / f"{package_name}.yaml"
        
        if not yaml_file.exists():
            print(f"Configuration file not found: {yaml_file}")
            return False
        
        pkg_config = self.load_package_config(yaml_file)
        if not pkg_config:
            return False
        
        print(f"Generating spec for {package_name}...")
        
        # Copy patches to SOURCES directory
        self.copy_patches_to_sources(pkg_config)
        
        spec_content = self.generate_spec_file(pkg_config)
        if not spec_content:
            return False
        
        # Write spec file
        spec_filename = f"ros-{self.distro}-{package_name.replace('_', '-')}.spec"
        spec_path = self.output_dir / spec_filename
        
        try:
            with open(spec_path, 'w') as f:
                f.write(spec_content)
            print(f"Generated: {spec_path}")
            return True
        except Exception as e:
            print(f"Error writing {spec_path}: {e}")
            return False

def main():
    parser = argparse.ArgumentParser(description='Generate RPM spec files from YAML configurations')
    parser.add_argument('--distro', default='jazzy', help='ROS distribution (default: jazzy)')
    parser.add_argument('--template-dir', default='templates', help='Templates directory')
    parser.add_argument('--cfg-dir', default='cfg', help='Configuration directory')
    parser.add_argument('--output-dir', default='specs_generated', help='Output directory for generated specs')
    parser.add_argument('--patches-dir', default='patches', help='Patches directory')
    parser.add_argument('--package', help='Generate spec for specific package only')
    parser.add_argument('--list-packages', action='store_true', help='List available packages')
    
    args = parser.parse_args()
    
    generator = SpecGenerator(
        distro=args.distro,
        template_dir=args.template_dir,
        cfg_dir=args.cfg_dir,
        output_dir=args.output_dir,
        patches_dir=args.patches_dir
    )
    
    if args.list_packages:
        yaml_files = glob.glob(str(Path(args.cfg_dir) / '*.yaml'))
        yaml_files = [f for f in yaml_files if not os.path.basename(f).startswith('_') 
                     and not os.path.basename(f) in ['rmf_deps.yaml', 'files.yaml']]
        
        print("Available packages:")
        for yaml_file in sorted(yaml_files):
            pkg_name = os.path.basename(yaml_file).replace('.yaml', '')
            print(f"  {pkg_name}")
        return
    
    if args.package:
        success = generator.generate_single_spec(args.package)
        sys.exit(0 if success else 1)
    else:
        generator.generate_all_specs()

if __name__ == '__main__':
    main() 