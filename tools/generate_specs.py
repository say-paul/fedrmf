#!/usr/bin/env python3
"""
Generate RPM spec files from cfg configuration
Uses <distro>.repos file as reference to update cfg/<distro>/<package>.yaml files
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
from datetime import datetime
from jinja2 import Environment, FileSystemLoader
import glob

class SpecGenerator:
    def __init__(self, distro='jazzy', template_dir='templates', cfg_base_dir='cfg', 
                 output_dir='specs_generated', patches_dir='patches'):
        self.distro = distro
        self.template_dir = Path(template_dir)
        self.cfg_base_dir = Path(cfg_base_dir)
        self.cfg_dir = self.cfg_base_dir / distro  # cfg/jazzy/
        self.output_dir = Path(output_dir)
        self.patches_dir = Path(patches_dir)
        
        # Reference repos file in distro directory
        self.repos_file = Path(f"distro/{distro}.repo")
        
        # Ensure directories exist
        self.cfg_dir.mkdir(parents=True, exist_ok=True)
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
            'rosidl_typesupport_c': f'ros-{distro}-rosidl-typesupport-c',
            'rosidl_typesupport_cpp': f'ros-{distro}-rosidl-typesupport-cpp',
            'rosidl_typesupport_introspection_c': f'ros-{distro}-rosidl-typesupport-introspection-c',
            'rosidl_typesupport_introspection_cpp': f'ros-{distro}-rosidl-typesupport-introspection-cpp',
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
        }
        
        # Add RMF packages dynamically
        for pkg_name in ['rmf_utils', 'rmf_traffic', 'rmf_task', 'rmf_battery', 'rmf_ros2',
                        'rmf_api_msgs', 'rmf_building_map_msgs', 'rmf_internal_msgs',
                        'rmf_visualization_msgs', 'rmf_traffic_editor', 'rmf_visualization',
                        'rmf_simulation', 'rmf_demos', 'menge_vendor',
                        'nlohmann_json_schema_validator_vendor', 'pybind11_json_vendor',
                        'ament_cmake_catch2']:
            self.ros_mappings[pkg_name] = f'ros-{distro}-{pkg_name.replace("_", "-")}'
        
        # System package mappings
        self.system_mappings = {
            'cmake': 'cmake',
            'eigen3': 'eigen3-devel',
            'eigen': 'eigen3-devel',
            'yaml-cpp': 'yaml-cpp-devel',
            'git': 'git',
            'python3-setuptools': 'python3-setuptools',
            'python3-pip': 'python3-pip',
            'python3-colcon-common-extensions': 'python3-colcon-common-extensions',
            'python3-jinja2': 'python3-jinja2',
            'json-devel': 'json-devel',
        }

    def resolve_dependency(self, dep_name, distro):
        """Resolve a dependency name to Fedora package name"""
        if dep_name in self.ros_mappings:
            return self.ros_mappings[dep_name]
        
        if dep_name in self.system_mappings:
            return self.system_mappings[dep_name]
        
        # Default: assume it's a ROS package
        if not dep_name.startswith('ros-'):
            return f'ros-{distro}-{dep_name.replace("_", "-")}'
        
        return dep_name

    def update_cfg_from_repos(self, repos_file=None, commit_msg="Updated from repos file"):
        """Update cfg/<distro>/ files from .repos file"""
        if repos_file is None:
            repos_file = self.repos_file
        
        repos_file = Path(repos_file)
        if not repos_file.exists():
            print(f"Repos file not found: {repos_file}")
            return False
        
        try:
            with open(repos_file, 'r') as f:
                repos_data = yaml.safe_load(f)
            
            repositories = repos_data.get('repositories', {})
            
            print(f"Updating {len(repositories)} packages in {self.cfg_dir}/ from {repos_file}")
            
            for repo_path, repo_info in repositories.items():
                package_name = repo_path.split('/')[-1]
                url = repo_info.get('url', '').rstrip('.git')
                version = repo_info.get('version', self.distro)
                
                self.update_package_cfg(package_name, url, version, commit_msg)
            
            return True
            
        except Exception as e:
            print(f"Error updating cfg from repos: {e}")
            return False

    def update_package_cfg(self, package_name, repository, version, commit_msg):
        """Update or create a single package cfg file"""
        cfg_file = self.cfg_dir / f"{package_name}.yaml"
        
        # Load existing cfg or create template
        if cfg_file.exists():
            with open(cfg_file, 'r') as f:
                cfg_data = yaml.safe_load(f)
        else:
            cfg_data = {
                'package_name': package_name,
                'build_depends': ['ament_cmake'],
                'exec_depends': [],
                'system_depends': ['cmake', 'gcc-c++'],
                'description': f"RMF package: {package_name}",
                'license': 'Apache-2.0',
                'build_priority': 99,
                'release': 1  # Default release number
            }
        
        # Update from repos info
        old_repository = cfg_data.get('repository')
        old_version = cfg_data.get('version')
        
        cfg_data['repository'] = repository
        cfg_data['version'] = version
        
        # Ensure release exists
        if 'release' not in cfg_data:
            cfg_data['release'] = 1
        
        # Track changes
        changes = []
        if old_repository != repository:
            changes.append(f"Repository: {old_repository} -> {repository}")
        if old_version != version:
            changes.append(f"Version: {old_version} -> {version}")
        
        # Add changelog if there are changes
        if changes:
            if 'changelog' not in cfg_data:
                cfg_data['changelog'] = []
            
            # RPM changelog format: * Day Mon DD YYYY Name <email> - version-release
            from datetime import datetime
            now = datetime.now()
            date_str = now.strftime('%a %b %d %Y')
            
            # Create version-release string in format: distro.version-release
            version_release = f"{self.distro}.{cfg_data['version']}-{cfg_data['release']}"
            
            # RPM changelog entry format
            changelog_entry = f"* {date_str} Sayan Paul <paul.sayan@gmail.com> - {version_release}"
            changelog_message = f"- {commit_msg}"
            
            # Add to changelog list
            cfg_data['changelog'].insert(0, {
                'entry': changelog_entry,
                'message': changelog_message
            })
            
            print(f"  Updated {package_name}:")
            for change in changes:
                print(f"    {change}")
            print(f"    RPM Changelog: {changelog_entry}")
            print(f"    {changelog_message}")
        
        # Write cfg file
        with open(cfg_file, 'w') as f:
            yaml.dump(cfg_data, f, default_flow_style=False, sort_keys=False)

    def find_patches(self, package_name):
        """Find patches for a given package in the patches directory"""
        patches = []
        if not self.patches_dir.exists():
            return patches
        
        patch_patterns = [
            f"{package_name}-*.patch",
            f"{package_name.replace('_', '-')}-*.patch"
        ]
        
        for pattern in patch_patterns:
            patch_files = list(self.patches_dir.glob(pattern))
            for patch_file in sorted(patch_files):
                patches.append(patch_file.name)
        
        return patches

    def load_package_config(self, package_name):
        """Load package configuration from cfg file"""
        cfg_file = self.cfg_dir / f"{package_name}.yaml"
        
        if not cfg_file.exists():
            print(f"Package config not found: {cfg_file}")
            return None
        
        try:
            with open(cfg_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Auto-inject colcon system dependencies when requested
            if config.get('use_colcon'):
                colcon_deps = [
                    'python3-colcon-core',
                    'python3-colcon-ros',
                    'python3-colcon-common-extensions',
                ]
                system_deps = list(config.get('system_depends') or [])
                for dep in colcon_deps:
                    if dep not in system_deps:
                        system_deps.append(dep)
                config['system_depends'] = system_deps
            
            # Auto-detect patches (always; caller can remove from cfg if undesired)
            patches = self.find_patches(package_name)
            if patches:
                config['patches'] = patches
                print(f"  Found patches for {package_name}: {', '.join(patches)}")
            
            return config
            
        except Exception as e:
            print(f"Error loading {cfg_file}: {e}")
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
                pkg_config['version'] = self.distro
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
            
            if patch_src.exists() and patch_src.stat().st_size > 13:
                try:
                    import shutil
                    shutil.copy2(patch_src, patch_dst)
                    print(f"  Copied patch: {patch}")
                except Exception as e:
                    print(f"  Warning: Failed to copy patch {patch}: {e}")
            else:
                print(f"  Warning: Skipping empty or non-existent patch: {patch}")

    def list_packages(self):
        """List available packages from cfg directory"""
        yaml_files = list(self.cfg_dir.glob('*.yaml'))
        
        if not yaml_files:
            print(f"No packages found in {self.cfg_dir}/")
            print(f"Run with --update-cfg to create package configs from repos file.")
            return
        
        print(f"Available packages in {self.distro}:")
        for yaml_file in sorted(yaml_files):
            pkg_name = yaml_file.stem
            try:
                with open(yaml_file, 'r') as f:
                    cfg_data = yaml.safe_load(f)
                repository = cfg_data.get('repository', 'unknown')
                version = cfg_data.get('version', 'unknown')
                print(f"  ✓ {pkg_name} -> {repository} (v{version})")
            except Exception as e:
                print(f"  ✗ {pkg_name} -> Error reading config: {e}")

    def generate_all_specs(self):
        """Generate spec files for all packages"""
        yaml_files = list(self.cfg_dir.glob('*.yaml'))
        
        if not yaml_files:
            print(f"No package configs found in {self.cfg_dir}/")
            print(f"Run with --update-cfg to create package configs from repos file.")
            return False
        
        print(f"Generating spec files for {len(yaml_files)} packages...")
        
        generated_count = 0
        failed_count = 0
        
        for yaml_file in sorted(yaml_files):
            package_name = yaml_file.stem
            pkg_config = self.load_package_config(package_name)
            if not pkg_config:
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
        
        return failed_count == 0

    def generate_single_spec(self, package_name):
        """Generate spec file for a single package"""
        pkg_config = self.load_package_config(package_name)
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

    def clean_all_changelogs(self):
        """Clean all changelogs and reset versions to blank"""
        yaml_files = list(self.cfg_dir.glob('*.yaml'))
        
        if not yaml_files:
            print(f"No package configs found in {self.cfg_dir}/")
            return False
        
        print(f"Cleaning changelogs for {len(yaml_files)} packages...")
        
        cleaned_count = 0
        failed_count = 0
        
        for yaml_file in sorted(yaml_files):
            package_name = yaml_file.stem
            
            try:
                with open(yaml_file, 'r') as f:
                    cfg_data = yaml.safe_load(f)
                
                # Remove changelog
                if 'changelog' in cfg_data:
                    del cfg_data['changelog']
                    print(f"  ✓ Removed changelog from {package_name}")
                
                # Reset version to blank
                old_version = cfg_data.get('version', 'None')
                cfg_data['version'] = ''
                print(f"  ✓ Reset version: {old_version} -> blank ({package_name})")
                
                # Reset release to 1
                cfg_data['release'] = 1
                
                # Write updated config
                with open(yaml_file, 'w') as f:
                    yaml.dump(cfg_data, f, default_flow_style=False, sort_keys=False)
                
                cleaned_count += 1
                
            except Exception as e:
                print(f"  ✗ Error cleaning {package_name}: {e}")
                failed_count += 1
        
        print(f"\nCleanup complete:")
        print(f"  Cleaned: {cleaned_count} packages")
        print(f"  Failed: {failed_count} packages")
        print(f"  All versions reset to blank")
        print(f"  Use 'make update-cfg' to repopulate from distro file")
        
        return failed_count == 0

def main():
    parser = argparse.ArgumentParser(description='Generate RPM spec files from cfg configurations')
    parser.add_argument('--distro', default='jazzy', help='ROS distribution (default: jazzy)')
    parser.add_argument('--template-dir', default='templates', help='Templates directory')
    parser.add_argument('--cfg-dir', default='cfg', help='Base configuration directory')
    parser.add_argument('--output-dir', default='specs_generated', help='Output directory for generated specs')
    parser.add_argument('--patches-dir', default='patches', help='Patches directory')
    parser.add_argument('--package', help='Generate spec for specific package only')
    parser.add_argument('--update-cfg', action='store_true', help='Update cfg files from repos file')
    parser.add_argument('--repos-file', help='Path to repos file (default: distro/<distro>.repo)')
    parser.add_argument('--commit-msg', default='Updated from repos file', help='Commit message for changes')
    parser.add_argument('--list-packages', action='store_true', help='List available packages')
    parser.add_argument('--clean-changelogs', action='store_true', help='Clean all changelogs and reset versions to blank')
    
    args = parser.parse_args()
    
    generator = SpecGenerator(
        distro=args.distro,
        template_dir=args.template_dir,
        cfg_base_dir=args.cfg_dir,
        output_dir=args.output_dir,
        patches_dir=args.patches_dir
    )
    
    # Only run requested operations
    if args.clean_changelogs:
        if not generator.clean_all_changelogs():
            sys.exit(1)
        return  # Exit after cleaning changelogs
    
    if args.update_cfg:
        if not generator.update_cfg_from_repos(repos_file=args.repos_file, commit_msg=args.commit_msg):
            sys.exit(1)
        return  # Exit after updating cfg
    
    if args.list_packages:
        generator.list_packages()
        return
    
    # Generate specs (default operation)
    if args.package:
        success = generator.generate_single_spec(args.package)
        sys.exit(0 if success else 1)
    else:
        success = generator.generate_all_specs()
        sys.exit(0 if success else 1)

if __name__ == '__main__':
    main() 