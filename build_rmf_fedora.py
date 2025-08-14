#!/usr/bin/env python3
"""
Fedora packaging builder for Open-RMF
Follows the pattern of ~/git_repos/rosfed
"""

import os
import sys
import yaml
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import tempfile
import shutil
import argparse
import networkx as nx
import json
from urllib.parse import urlparse

class RMFFedoraBuilder:
    def __init__(self, distro='jazzy', rebuild=False, only_new=False, enable_net=False, verbose=False,
                 copr_repo=None, chroot=None, mock_config=None):
        self.distro = distro
        self.rebuild = rebuild
        self.only_new = only_new
        self.enable_net = enable_net
        self.verbose = verbose
        self.copr_repo = copr_repo
        self.chroot = chroot or f"fedora-{self._get_fedora_version()}-{self._get_arch()}"
        self.mock_config = mock_config
        self.base_dir = Path(__file__).parent
        self.build_dir = self.base_dir / 'fedrmf'
        self.specs_dir = self.build_dir / 'SPECS'
        self.sources_dir = self.build_dir / 'SOURCES'
        self.rpms_dir = self.build_dir / 'RPMS'
        self.temp_dir = None
        
        # Build order and dependency tracking
        self.dependency_graph = nx.DiGraph()
        self.build_order = []
        self.package_info_cache = {}
        
        # Track existing packages for --only-new functionality
        self.existing_specs = set()
        self.existing_rpms = set()
        
        # ROS package name mappings
        self.ros_pkg_mappings = {
            'ament_cmake': f'ros-{distro}-ament-cmake',
            'ament_cmake_auto': f'ros-{distro}-ament-cmake-auto',
            'ament_cmake_gtest': f'ros-{distro}-ament-cmake-gtest',
            'ament_cmake_pytest': f'ros-{distro}-ament-cmake-pytest',
            'ament_index_cpp': f'ros-{distro}-ament-index-cpp',
            'ament_index_python': f'ros-{distro}-ament-index-python',
            'rclcpp': f'ros-{distro}-rclcpp',
            'rclpy': f'ros-{distro}-rclpy',
            'rcutils': f'ros-{distro}-rcutils',
            'rmw': f'ros-{distro}-rmw',
            'rmw_implementation': f'ros-{distro}-rmw-implementation',
            'geometry_msgs': f'ros-{distro}-geometry-msgs',
            'nav_msgs': f'ros-{distro}-nav-msgs',
            'sensor_msgs': f'ros-{distro}-sensor-msgs',
            'std_msgs': f'ros-{distro}-std-msgs',
            'std_srvs': f'ros-{distro}-std-srvs',
            'builtin_interfaces': f'ros-{distro}-builtin-interfaces',
            'tf2': f'ros-{distro}-tf2',
            'tf2_ros': f'ros-{distro}-tf2-ros',
            'tf2_geometry_msgs': f'ros-{distro}-tf2-geometry-msgs',
            'gazebo_ros': f'ros-{distro}-gazebo-ros',
            'gazebo_plugins': f'ros-{distro}-gazebo-plugins',
            'gazebo_msgs': f'ros-{distro}-gazebo-msgs',
            'robot_state_publisher': f'ros-{distro}-robot-state-publisher',
            'joint_state_publisher': f'ros-{distro}-joint-state-publisher',
            'urdf': f'ros-{distro}-urdf',
            'xacro': f'ros-{distro}-xacro',
            'lifecycle_msgs': f'ros-{distro}-lifecycle-msgs',
            'rclcpp_lifecycle': f'ros-{distro}-rclcpp-lifecycle',
            'rclcpp_action': f'ros-{distro}-rclcpp-action',
            'action_msgs': f'ros-{distro}-action-msgs',
        }
        
        # System dependencies mappings
        self.system_deps = {
            'eigen3': 'eigen3-devel',
            'yaml-cpp': 'yaml-cpp-devel',
            'nlohmann_json': 'json-devel',
            'opencv': 'opencv-devel',
            'qt5': 'qt5-qtbase-devel',
            'boost': 'boost-devel',
            'openssl': 'openssl-devel',
            'sqlite3': 'sqlite-devel',
            'curl': 'libcurl-devel',
            'pybind11': 'pybind11-devel',
            'websocketpp': 'websocketpp-devel',
            'tinyxml2': 'tinyxml2-devel',
            'cmake': 'cmake',
            'pkg-config': 'pkgconfig',
        }

    def _get_fedora_version(self):
        """Get current Fedora version"""
        try:
            with open('/etc/fedora-release', 'r') as f:
                line = f.read().strip()
                # Extract version number from "Fedora release 39 (Thirty Nine)"
                parts = line.split()
                for part in parts:
                    if part.isdigit():
                        return part
        except:
            pass
        return "39"  # Default fallback

    def _get_arch(self):
        """Get system architecture"""
        return os.uname().machine

    def log(self, message, verbose_only=False):
        """Log messages based on verbosity level"""
        if not verbose_only or self.verbose:
            print(message)

    def setup_copr_config(self):
        """Setup COPR configuration for builds"""
        if not self.copr_repo:
            return
            
        copr_config = {
            'copr_repo': self.copr_repo,
            'chroot': self.chroot,
            'mock_config': self.mock_config or f'/etc/mock/{self.chroot}.cfg'
        }
        
        # Write copr configuration
        copr_config_file = self.build_dir / 'copr_config.json'
        with open(copr_config_file, 'w') as f:
            json.dump(copr_config, f, indent=2)
        
        self.log(f"COPR configuration written to: {copr_config_file}")
        self.log(f"Target COPR repo: {self.copr_repo}")
        self.log(f"Target chroot: {self.chroot}")

    def create_mock_config(self):
        """Create mock configuration for chroot builds"""
        if not self.chroot:
            return
            
        mock_config_content = f'''
config_opts['root'] = '{self.chroot}'
config_opts['target_arch'] = '{self._get_arch()}'
config_opts['legal_host_arches'] = ('{self._get_arch()}',)
config_opts['chroot_setup_cmd'] = 'install @buildsys-build'
config_opts['dist'] = 'fc{self._get_fedora_version()}'
config_opts['releasever'] = '{self._get_fedora_version()}'
config_opts['package_manager'] = 'dnf'
config_opts['extra_chroot_dirs'] = [ '/run/lock', ]

config_opts['yum.conf'] = """
[main]
keepcache=1
debuglevel=2
reposdir=/dev/null
logfile=/var/log/yum.log
retries=20
obsoletes=1
gpgcheck=1
assumeyes=1
syslog_ident=mock
syslog_device=
install_weak_deps=0
metadata_expire=0
best=1
module_platform_id=platform:f{self._get_fedora_version()}

[fedora]
name=fedora
metalink=https://mirrors.fedoraproject.org/metalink?repo=fedora-$releasever&arch=$basearch
enabled=1
gpgcheck=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-fedora-$releasever-$basearch
skip_if_unavailable=False

[updates]
name=updates
metalink=https://mirrors.fedoraproject.org/metalink?repo=updates-released-f$releasever&arch=$basearch
enabled=1
gpgcheck=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-fedora-$releasever-$basearch
skip_if_unavailable=False
'''

        # Add COPR repo to mock config if specified
        if self.copr_repo:
            mock_config_content += f'''
[copr-{self.copr_repo.replace('/', '-')}]
name=Copr repo for {self.copr_repo}
baseurl=https://download.copr.fedorainfracloud.org/results/{self.copr_repo}/{self.chroot}/
type=rpm-md
skip_if_unavailable=True
gpgcheck=1
gpgkey=https://download.copr.fedorainfracloud.org/results/{self.copr_repo}/pubkey.gpg
repo_gpgcheck=0
enabled=1
enabled_metadata=1
'''

        mock_config_content += '"""'
        
        # Write mock configuration
        mock_config_file = self.build_dir / f'{self.chroot}.cfg'
        with open(mock_config_file, 'w') as f:
            f.write(mock_config_content)
        
        self.log(f"Mock configuration written to: {mock_config_file}")
        return mock_config_file

    def build_dependency_graph(self, repositories):
        """Build dependency graph from all packages"""
        self.log("Building dependency graph...")
        
        # First pass: collect all package information
        for repo_info in repositories:
            repo_dir = self.clone_repository(repo_info)
            if not repo_dir:
                continue
                
            package_xmls = self.find_package_xmls(repo_dir)
            for package_xml in package_xmls:
                package_info = self.parse_package_xml(package_xml)
                if package_info and package_info['name']:
                    self.package_info_cache[package_info['name']] = {
                        'info': package_info,
                        'repo': repo_info
                    }
                    # Add node to graph
                    self.dependency_graph.add_node(package_info['name'])
        
        # Second pass: add dependency edges
        for package_name, cached_data in self.package_info_cache.items():
            package_info = cached_data['info']
            
            for dep in package_info['build_depends'] + package_info['exec_depends']:
                # Only add edges for packages we're building (internal dependencies)
                if dep in self.package_info_cache:
                    self.dependency_graph.add_edge(dep, package_name)
                    self.log(f"Added dependency: {dep} -> {package_name}", verbose_only=True)
        
        self.log(f"Dependency graph built: {len(self.dependency_graph.nodes)} packages, {len(self.dependency_graph.edges)} dependencies")

    def create_build_order(self):
        """Create build order using topological sort"""
        self.log("Creating build order...")
        
        try:
            # Topological sort gives us the build order
            self.build_order = list(nx.topological_sort(self.dependency_graph))
            self.log(f"Build order created with {len(self.build_order)} packages")
            
            if self.verbose:
                self.log("Build order:")
                for i, package in enumerate(self.build_order, 1):
                    self.log(f"  {i:2d}. {package}")
                    
        except nx.NetworkXError as e:
            if "cycle" in str(e).lower():
                self.log("Circular dependency detected! Finding cycles...")
                cycles = list(nx.simple_cycles(self.dependency_graph))
                for i, cycle in enumerate(cycles):
                    self.log(f"Cycle {i+1}: {' -> '.join(cycle + [cycle[0]])}")
                # Fall back to a simple ordering
                self.build_order = list(self.dependency_graph.nodes)
            else:
                raise

    def save_build_order(self):
        """Save build order to file"""
        build_order_file = self.build_dir / 'build_order.json'
        build_data = {
            'distro': self.distro,
            'chroot': self.chroot,
            'copr_repo': self.copr_repo,
            'build_order': self.build_order,
            'dependency_graph': {
                'nodes': list(self.dependency_graph.nodes),
                'edges': list(self.dependency_graph.edges)
            },
            'package_count': len(self.build_order)
        }
        
        with open(build_order_file, 'w') as f:
            json.dump(build_data, f, indent=2)
        
        self.log(f"Build order saved to: {build_order_file}")

    def load_build_order(self):
        """Load existing build order from file"""
        build_order_file = self.build_dir / 'build_order.json'
        if build_order_file.exists():
            with open(build_order_file, 'r') as f:
                build_data = json.load(f)
            
            self.build_order = build_data.get('build_order', [])
            self.log(f"Loaded existing build order with {len(self.build_order)} packages")
            return True
        return False

    def copr_build_package(self, package_name):
        """Build package in COPR"""
        if not self.copr_repo:
            self.log("No COPR repo specified, skipping COPR build")
            return False
            
        spec_file = self.specs_dir / f"ros-{self.distro}-{package_name.replace('_', '-')}.spec"
        if not spec_file.exists():
            self.log(f"Spec file not found: {spec_file}")
            return False
        
        # Build SRPM first
        srpm_cmd = [
            'rpmbuild',
            '--define', f'_topdir {self.build_dir}',
            '--define', f'_sourcedir {self.sources_dir}',
            '--define', f'_srcrpmdir {self.build_dir}/SRPMS',
            '-bs', str(spec_file)
        ]
        
        try:
            self.log(f"Building SRPM for {package_name}...")
            subprocess.run(srpm_cmd, check=True, capture_output=not self.verbose)
            
            # Find the SRPM file
            srpm_files = list((self.build_dir / 'SRPMS').glob(f"ros-{self.distro}-{package_name.replace('_', '-')}-*.src.rpm"))
            if not srpm_files:
                self.log(f"SRPM not found for {package_name}")
                return False
            
            srpm_file = srpm_files[0]
            
            # Submit to COPR
            copr_cmd = [
                'copr-cli', 'build',
                self.copr_repo,
                str(srpm_file),
                '--chroot', self.chroot
            ]
            
            self.log(f"Submitting {package_name} to COPR repo {self.copr_repo}")
            result = subprocess.run(copr_cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.log(f"Successfully submitted {package_name} to COPR")
                # Extract build ID from output for tracking
                for line in result.stdout.split('\n'):
                    if 'Build' in line and 'submitted' in line:
                        self.log(f"  {line.strip()}")
                return True
            else:
                self.log(f"Failed to submit {package_name} to COPR: {result.stderr}")
                return False
                
        except subprocess.CalledProcessError as e:
            self.log(f"Error building SRPM for {package_name}: {e}")
            return False

    def mock_build_package(self, package_name):
        """Build package using mock"""
        spec_file = self.specs_dir / f"ros-{self.distro}-{package_name.replace('_', '-')}.spec"
        if not spec_file.exists():
            self.log(f"Spec file not found: {spec_file}")
            return False
        
        mock_config_file = self.mock_config or self.create_mock_config()
        
        mock_cmd = [
            'mock',
            '-r', str(mock_config_file),
            '--buildsrpm',
            '--spec', str(spec_file),
            '--sources', str(self.sources_dir),
            '--resultdir', str(self.build_dir / 'SRPMS')
        ]
        
        try:
            self.log(f"Building {package_name} with mock...")
            subprocess.run(mock_cmd, check=True, capture_output=not self.verbose)
            
            # Build RPM from SRPM
            srpm_files = list((self.build_dir / 'SRPMS').glob(f"ros-{self.distro}-{package_name.replace('_', '-')}-*.src.rpm"))
            if srpm_files:
                srpm_file = srpm_files[0]
                
                mock_rpm_cmd = [
                    'mock',
                    '-r', str(mock_config_file),
                    '--rebuild', str(srpm_file),
                    '--resultdir', str(self.rpms_dir / self._get_arch())
                ]
                
                subprocess.run(mock_rpm_cmd, check=True, capture_output=not self.verbose)
                self.log(f"Successfully built {package_name} with mock")
                return True
            
        except subprocess.CalledProcessError as e:
            self.log(f"Error building {package_name} with mock: {e}")
            return False

    def scan_existing_packages(self):
        """Scan for existing spec files and RPMs for --only-new functionality"""
        if self.specs_dir.exists():
            for spec_file in self.specs_dir.glob("*.spec"):
                self.existing_specs.add(spec_file.stem)
        
        if self.rpms_dir.exists():
            for arch_dir in self.rpms_dir.iterdir():
                if arch_dir.is_dir():
                    for rpm_file in arch_dir.glob("*.rpm"):
                        # Extract package name from RPM filename
                        rpm_name = rpm_file.name.split('-')[0:3]  # ros-distro-package
                        if len(rpm_name) >= 3:
                            self.existing_rpms.add('-'.join(rpm_name))

    def should_process_package(self, package_name):
        """Check if package should be processed based on --only-new flag"""
        if not self.only_new:
            return True
        
        spec_name = f"ros-{self.distro}-{package_name.replace('_', '-')}"
        rpm_name = spec_name
        
        # Skip if spec and RPM both exist (unless rebuilding)
        if spec_name in self.existing_specs and rpm_name in self.existing_rpms and not self.rebuild:
            self.log(f"Skipping {package_name} (already exists)", verbose_only=True)
            return False
        
        return True

    def setup_temp_workspace(self):
        """Create temporary workspace for cloning repositories"""
        self.temp_dir = tempfile.mkdtemp(prefix='rmf_build_')
        self.log(f"Created temporary workspace: {self.temp_dir}")

    def cleanup_temp_workspace(self):
        """Clean up temporary workspace"""
        if self.temp_dir and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)
            self.log(f"Cleaned up temporary workspace: {self.temp_dir}")

    def parse_rmf_repos(self, repos_file):
        """Parse the rmf.repos file and return repository information"""
        with open(repos_file, 'r') as f:
            repos_data = yaml.safe_load(f)
        
        repositories = []
        for repo_path, repo_info in repos_data['repositories'].items():
            repo_name = os.path.basename(repo_path)
            repositories.append({
                'name': repo_name,
                'path': repo_path,
                'url': repo_info['url'],
                'version': repo_info['version']
            })
        
        return repositories

    def clone_repository(self, repo_info):
        """Clone a repository to temporary workspace"""
        repo_dir = os.path.join(self.temp_dir, repo_info['name'])
        
        self.log(f"Cloning {repo_info['name']} from {repo_info['url']} (branch: {repo_info['version']})")
        
        try:
            cmd = [
                'git', 'clone', 
                '--branch', repo_info['version'],
                '--depth', '1',
                repo_info['url'], 
                repo_dir
            ]
            
            # Add network-related options if --enable-net
            if self.enable_net:
                cmd.extend(['--config', 'http.sslverify=false'])
            
            subprocess.run(cmd, check=True, capture_output=not self.verbose)
            return repo_dir
        except subprocess.CalledProcessError as e:
            self.log(f"Failed to clone {repo_info['name']}: {e}")
            return None

    def find_package_xmls(self, repo_dir):
        """Find all package.xml files in a repository"""
        package_xmls = []
        for root, dirs, files in os.walk(repo_dir):
            if 'package.xml' in files:
                package_xmls.append(os.path.join(root, 'package.xml'))
        return package_xmls

    def parse_package_xml(self, package_xml_path):
        """Parse package.xml and extract package information"""
        try:
            tree = ET.parse(package_xml_path)
            root = tree.getroot()
            
            package_info = {
                'name': root.find('name').text if root.find('name') is not None else '',
                'version': root.find('version').text if root.find('version') is not None else '0.0.0',
                'description': root.find('description').text if root.find('description') is not None else '',
                'maintainers': [],
                'licenses': [],
                'build_depends': [],
                'exec_depends': [],
                'test_depends': []
            }
            
            # Extract maintainers
            for maintainer in root.findall('maintainer'):
                package_info['maintainers'].append({
                    'name': maintainer.text,
                    'email': maintainer.get('email', '')
                })
            
            # Extract licenses
            for license_elem in root.findall('license'):
                package_info['licenses'].append(license_elem.text)
            
            # Extract dependencies
            for dep in root.findall('.//build_depend'):
                package_info['build_depends'].append(dep.text)
            
            for dep in root.findall('.//exec_depend'):
                package_info['exec_depends'].append(dep.text)
                
            for dep in root.findall('.//test_depend'):
                package_info['test_depends'].append(dep.text)
            
            return package_info
        except Exception as e:
            self.log(f"Error parsing {package_xml_path}: {e}")
            return None

    def map_ros_dependency(self, dep_name):
        """Map ROS dependency to Fedora package name"""
        # Check direct mappings first
        if dep_name in self.ros_pkg_mappings:
            return self.ros_pkg_mappings[dep_name]
        
        # Check system dependencies
        if dep_name in self.system_deps:
            return self.system_deps[dep_name]
        
        # Default ROS package naming convention
        if not dep_name.startswith('ros-'):
            return f'ros-{self.distro}-{dep_name.replace("_", "-")}'
        
        return dep_name

    def generate_spec_file(self, package_info, repo_info):
        """Generate RPM spec file for a package"""
        
        # Network-related build options
        network_section = ""
        if self.enable_net:
            network_section = """
# Network enabled build
%define _disable_ld_no_undefined 1
%global __os_install_post /usr/lib/rpm/brp-compress \\
  %{!?__debug_package:/usr/lib/rpm/brp-strip %{__strip}} \\
  /usr/lib/rpm/brp-strip-static-archive %{__strip} \\
  /usr/lib/rpm/brp-strip-comment-note %{__strip} %{__objdump}
"""
        
        spec_content = f"""{network_section}
%global ros_distro {self.distro}

Name:           ros-{self.distro}-{package_info['name'].replace('_', '-')}
Version:        {package_info['version']}
Release:        1%{{?dist}}
Summary:        {package_info['description'][:50]}...

License:        {' and '.join(package_info['licenses']) if package_info['licenses'] else 'Apache-2.0'}
URL:            {repo_info['url']}
Source0:        {repo_info['url']}/archive/{repo_info['version']}.tar.gz

BuildRequires:  cmake
BuildRequires:  gcc-c++
BuildRequires:  ros-{self.distro}-ament-cmake
BuildRequires:  ros-{self.distro}-rclcpp-devel
"""

        # Add build dependencies
        build_deps = set()
        for dep in package_info['build_depends']:
            mapped_dep = self.map_ros_dependency(dep)
            if mapped_dep and mapped_dep != f'ros-{self.distro}-{package_info["name"].replace("_", "-")}':
                build_deps.add(mapped_dep)
        
        for dep in sorted(build_deps):
            spec_content += f"BuildRequires:  {dep}\n"

        spec_content += f"""
Requires:       ros-{self.distro}-rclcpp
"""

        # Add runtime dependencies
        runtime_deps = set()
        for dep in package_info['exec_depends']:
            mapped_dep = self.map_ros_dependency(dep)
            if mapped_dep and mapped_dep != f'ros-{self.distro}-{package_info["name"].replace("_", "-")}':
                runtime_deps.add(mapped_dep)
        
        for dep in sorted(runtime_deps):
            spec_content += f"Requires:       {dep}\n"

        # Build section with network options
        cmake_args = [
            "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
            "-DBUILD_TESTING=OFF",
            f"-DCMAKE_INSTALL_PREFIX=/opt/ros/{self.distro}"
        ]
        
        if self.enable_net:
            cmake_args.extend([
                "-DFETCHCONTENT_FULLY_DISCONNECTED=OFF",
                "-DFETCHCONTENT_TRY_FIND_PACKAGE_MODE=NEVER"
            ])

        spec_content += f"""
%description
{package_info['description']}

%prep
%autosetup -n {package_info['name']}-{repo_info['version']}

%build
export ROS_DISTRO={self.distro}
export AMENT_PREFIX_PATH=/opt/ros/{self.distro}
export CMAKE_PREFIX_PATH=/opt/ros/{self.distro}
source /opt/ros/{self.distro}/setup.bash

%cmake \\
    {' \\'.join(cmake_args)}

%cmake_build

%install
%cmake_install

%files
/opt/ros/{self.distro}/*

%changelog
* {subprocess.run(['date', '+%a %b %d %Y'], capture_output=True, text=True).stdout.strip()} RMF Fedora Builder <builder@localhost> - {package_info['version']}-1
- Initial package for {package_info['name']}
"""

        return spec_content

    def process_repository(self, repo_info):
        """Process a single repository"""
        self.log(f"Processing repository: {repo_info['name']}")
        
        # Clone repository
        repo_dir = self.clone_repository(repo_info)
        if not repo_dir:
            return
        
        # Find package.xml files
        package_xmls = self.find_package_xmls(repo_dir)
        
        for package_xml in package_xmls:
            self.log(f"  Processing package.xml: {package_xml}", verbose_only=True)
            package_info = self.parse_package_xml(package_xml)
            
            if package_info and package_info['name']:
                # Check if we should process this package
                if not self.should_process_package(package_info['name']):
                    continue
                    
                self.log(f"  Generating spec for {package_info['name']}")
                
                # Generate spec file
                spec_content = self.generate_spec_file(package_info, repo_info)
                
                # Write spec file
                spec_filename = f"ros-{self.distro}-{package_info['name'].replace('_', '-')}.spec"
                spec_path = self.specs_dir / spec_filename
                
                # Remove existing spec if rebuilding
                if self.rebuild and spec_path.exists():
                    spec_path.unlink()
                    self.log(f"    Removed existing spec: {spec_path}", verbose_only=True)
                
                with open(spec_path, 'w') as f:
                    f.write(spec_content)
                
                self.log(f"    Generated spec file: {spec_path}", verbose_only=True)

    def build_packages(self):
        """Main build process"""
        self.log(f"Building Open-RMF packages for Fedora (ROS {self.distro})")
        self.log(f"Options: rebuild={self.rebuild}, only_new={self.only_new}, enable_net={self.enable_net}")
        if self.copr_repo:
            self.log(f"COPR repo: {self.copr_repo}")
        self.log(f"Chroot: {self.chroot}")
        
        # Ensure directories exist
        self.specs_dir.mkdir(parents=True, exist_ok=True)
        self.sources_dir.mkdir(parents=True, exist_ok=True)
        
        # Setup COPR configuration
        if self.copr_repo:
            self.setup_copr_config()
        
        # Scan existing packages if using --only-new
        if self.only_new:
            self.log("Scanning for existing packages...")
            self.scan_existing_packages()
            self.log(f"Found {len(self.existing_specs)} existing specs, {len(self.existing_rpms)} existing RPMs")
        
        # Setup temporary workspace
        self.setup_temp_workspace()
        
        try:
            # Parse rmf.repos
            repos_file = self.build_dir / 'rmf.repos'
            if not repos_file.exists():
                self.log(f"Error: {repos_file} not found. Run 'make fetch-repos' first.")
                return
                
            repositories = self.parse_rmf_repos(repos_file)
            
            self.log(f"Found {len(repositories)} repositories to process")
            
            # Build dependency graph and create build order
            self.build_dependency_graph(repositories)
            self.create_build_order()
            self.save_build_order()
            
            # Process each repository in build order
            processed_count = 0
            for package_name in self.build_order:
                if package_name in self.package_info_cache:
                    cached_data = self.package_info_cache[package_name]
                    self.process_repository(cached_data['repo'])
                    processed_count += 1
            
            self.log("Build process completed!")
            self.log(f"Processed {processed_count} repositories")
            self.log(f"Spec files generated in: {self.specs_dir}")
            self.log(f"Build order saved in: {self.build_dir}/build_order.json")
            
        finally:
            # Cleanup
            self.cleanup_temp_workspace()

def main():
    parser = argparse.ArgumentParser(
        description='Fedora packaging builder for Open-RMF',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 build_rmf_fedora.py jazzy
  python3 build_rmf_fedora.py -r jazzy          # Rebuild all packages
  python3 build_rmf_fedora.py --only-new jazzy  # Only build new packages
  python3 build_rmf_fedora.py --enable-net jazzy # Enable network during build
  python3 build_rmf_fedora.py --copr-repo user/repo --chroot fedora-39-x86_64 jazzy
  python3 build_rmf_fedora.py -r --only-new --enable-net --verbose jazzy
        """
    )
    
    parser.add_argument(
        'distro',
        nargs='?',
        default='jazzy',
        help='ROS distribution (default: jazzy)'
    )
    
    parser.add_argument(
        '-r', '--rebuild',
        action='store_true',
        help='Rebuild all packages, removing existing spec files first'
    )
    
    parser.add_argument(
        '--only-new',
        action='store_true',
        help='Only process packages that don\'t already have spec files and RPMs'
    )
    
    parser.add_argument(
        '--enable-net',
        action='store_true',
        help='Enable network access during build (allows downloading dependencies)'
    )
    
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    
    parser.add_argument(
        '--list-existing',
        action='store_true',
        help='List existing packages and exit'
    )
    
    parser.add_argument(
        '--copr-repo',
        help='Target COPR repository (user/repo format)'
    )
    
    parser.add_argument(
        '--chroot',
        help='Target chroot for builds (e.g., fedora-39-x86_64)'
    )
    
    parser.add_argument(
        '--mock-config',
        help='Path to custom mock configuration file'
    )
    
    parser.add_argument(
        '--build-order-only',
        action='store_true',
        help='Only create build order, don\'t generate specs'
    )
    
    parser.add_argument(
        '--load-build-order',
        action='store_true',
        help='Load existing build order instead of creating new one'
    )
    
    args = parser.parse_args()
    
    builder = RMFFedoraBuilder(
        distro=args.distro,
        rebuild=args.rebuild,
        only_new=args.only_new,
        enable_net=args.enable_net,
        verbose=args.verbose,
        copr_repo=args.copr_repo,
        chroot=args.chroot,
        mock_config=args.mock_config
    )
    
    if args.list_existing:
        builder.scan_existing_packages()
        print(f"Existing spec files ({len(builder.existing_specs)}):")
        for spec in sorted(builder.existing_specs):
            print(f"  {spec}")
        print(f"\nExisting RPM packages ({len(builder.existing_rpms)}):")
        for rpm in sorted(builder.existing_rpms):
            print(f"  {rpm}")
        return
    
    if args.load_build_order:
        if builder.load_build_order():
            print("Loaded existing build order:")
            for i, package in enumerate(builder.build_order, 1):
                print(f"  {i:2d}. {package}")
        else:
            print("No existing build order found.")
        return
    
    if args.build_order_only:
        # Only create dependency graph and build order
        builder.setup_temp_workspace()
        try:
            repos_file = builder.build_dir / 'rmf.repos'
            if repos_file.exists():
                repositories = builder.parse_rmf_repos(repos_file)
                builder.build_dependency_graph(repositories)
                builder.create_build_order()
                builder.save_build_order()
            else:
                print(f"Error: {repos_file} not found. Run 'make fetch-repos' first.")
        finally:
            builder.cleanup_temp_workspace()
        return
    
    builder.build_packages()

if __name__ == '__main__':
    main() 