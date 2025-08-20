#!/usr/bin/env python3
"""
Build dependency tree and manage source downloads for RMF packages
"""

import os
import sys
import yaml
import json
import argparse
import urllib.request
import urllib.error
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional
import hashlib
import networkx as nx

class RMFDependencyManager:
    def __init__(self, cfg_dir='cfg', sources_dir='fedrmf/SOURCES', build_dir='fedrmf', distro: str = 'jazzy', skip_packages: Optional[Set[str]] = None):
        self.cfg_dir = Path(cfg_dir)
        self.sources_dir = Path(sources_dir)
        self.build_dir = Path(build_dir)
        self.distro = distro
        self.skip_packages = set(skip_packages or [])
        
        # Ensure directories exist
        self.sources_dir.mkdir(parents=True, exist_ok=True)
        self.build_dir.mkdir(parents=True, exist_ok=True)
        
        self.packages: Dict[str, dict] = {}
        self.dependency_graph = nx.DiGraph()
        
    def load_package_configs(self):
        """Load all package configurations from YAML files"""
        yaml_files = list(self.cfg_dir.glob('*.yaml'))
        yaml_files = [f for f in yaml_files if not f.name.startswith('_') 
                     and f.name not in ['rmf_deps.yaml', 'files.yaml']]
        
        print(f"Loading {len(yaml_files)} package configurations from {self.cfg_dir}...")
        
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as f:
                    config = yaml.safe_load(f)
                
                package_name = config.get('package_name')
                if package_name:
                    if package_name in self.skip_packages:
                        print(f"  Skipping (per --skip-package): {package_name}")
                        continue
                    self.packages[package_name] = config
                    print(f"  Loaded: {package_name}")
                else:
                    print(f"  Warning: No package_name in {yaml_file}")
                    
            except Exception as e:
                print(f"  Error loading {yaml_file}: {e}")
        
        print(f"Loaded {len(self.packages)} packages (after skips).")
        
    def _map_system_dep_to_internal(self, dep: str) -> Optional[str]:
        """Map a system_dep string like 'ros-jazzy-rmf-utils' or 'ros2-jazzy-rmf-utils' back to an internal package name like 'rmf_utils'.
        Returns None if not an internal package or mapping not possible.
        """
        prefixes = [f"ros-{self.distro}-", f"ros2-{self.distro}-"]
        for prefix in prefixes:
            if dep.startswith(prefix):
                name_part = dep[len(prefix):]
                # Accept both hyphens and underscores; normalize to underscores to match our YAML package names
                internal = name_part.replace('-', '_')
                # Some ROS RPMs may include additional suffixes (e.g., -devel). Drop common suffixes
                for suffix in ("_devel", "-devel"):
                    if internal.endswith(suffix):
                        internal = internal[: -len(suffix)]
                # Only treat as internal if we actually have this package loaded
                return internal
        return None
        
    def build_dependency_graph(self):
        """Build dependency graph from package configurations"""
        print("Building dependency graph...")
        
        # Add all packages as nodes
        for pkg_name, config in self.packages.items():
            self.dependency_graph.add_node(pkg_name, **config)
        
        # Add dependency edges (include system_depends-derived internal deps)
        for pkg_name, config in self.packages.items():
            build_deps = config.get('build_depends', []) or []
            exec_deps = config.get('exec_depends', []) or []
            system_deps = config.get('system_depends', []) or []
            
            # Map system_depends to potential internal package names
            mapped_internal_from_system = []
            for dep in system_deps:
                internal = self._map_system_dep_to_internal(dep)
                if internal and internal in self.packages:
                    mapped_internal_from_system.append(internal)
            
            # Combine all deps
            all_deps = set(build_deps + exec_deps + mapped_internal_from_system)
            
            for dep in all_deps:
                # Only add edges for RMF packages (ignore system/ROS packages)
                if dep in self.packages and dep != pkg_name:
                    self.dependency_graph.add_edge(dep, pkg_name)
                    print(f"  {pkg_name} depends on {dep}")
        
        print(f"Dependency graph built with {len(self.dependency_graph.nodes)} nodes and {len(self.dependency_graph.edges)} edges.")
        
    def check_circular_dependencies(self):
        """Check for circular dependencies in the graph"""
        try:
            cycles = list(nx.simple_cycles(self.dependency_graph))
            if cycles:
                print("ERROR: Circular dependencies detected!")
                for cycle in cycles:
                    print(f"  Cycle: {' -> '.join(cycle + [cycle[0]])}")
                return False
            else:
                print("No circular dependencies found.")
                return True
        except Exception as e:
            print(f"Error checking for cycles: {e}")
            return False
    
    def get_build_order(self):
        """Get topological build order"""
        try:
            build_order = list(nx.topological_sort(self.dependency_graph))
            print(f"Build order determined for {len(build_order)} packages:")
            for i, pkg in enumerate(build_order, 1):
                priority = self.packages[pkg].get('build_priority', 999)
                print(f"  {i:2d}. {pkg} (priority: {priority})")
            return build_order
        except nx.NetworkXError as e:
            print(f"Error: Cannot determine build order - {e}")
            return []
    
    def save_build_order(self, build_order, filename='build-order.json'):
        """Save a simple dependency JSON with only:
        - depends: {pkg: [internal_deps]}
        """
        # Build depends mapping (internal edges only)
        depends: Dict[str, List[str]] = {}
        for pkg in build_order:
            if pkg in self.dependency_graph:
                deps = [u for u, v in self.dependency_graph.in_edges(pkg)]
                depends[pkg] = sorted(deps)
            else:
                depends[pkg] = []

        build_data = {
            'depends': depends,
        }

        # Ensure directory exists
        filepath = Path(filename)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        try:
            with open(filepath, 'w') as f:
                json.dump(build_data, f, indent=2)
            print(f"Build order saved to: {filepath}")
        except Exception as e:
            print(f"Error saving build order: {e}")
            return False

        return True
    
    def load_build_order(self, filename='build_order.json'):
        """Load build order from JSON file"""
        build_order_file = self.build_dir / filename
        
        if not build_order_file.exists():
            print(f"Build order file not found: {build_order_file}")
            return None
        
        try:
            with open(build_order_file, 'r') as f:
                build_data = json.load(f)
            print(f"Build order loaded from: {build_order_file}")
            return build_data
        except Exception as e:
            print(f"Error loading build order: {e}")
            return None
    
    def download_source(self, package_name, force=False):
        """Download source tarball for a package"""
        if package_name not in self.packages:
            print(f"Package {package_name} not found")
            return False
        
        config = self.packages[package_name]
        repository = config.get('repository', '')
        version = config.get('version', '0.0.0')
        
        if not repository or not version:
            print(f"  Missing repository or version for {package_name}")
            return False
        
        # Construct source URL from repository + version
        source_url = f"{repository}/archive/refs/tags/{version}.tar.gz"
        
        # Determine filename based on what the tarball will extract to
        filename = f"{package_name}-{version}.tar.gz"
        target_file = self.sources_dir / filename
        
        # Check if file already exists
        if target_file.exists() and not force:
            print(f"  Source already exists: {filename}")
            return True
        
        print(f"  Downloading: {source_url}")
        print(f"  Target: {target_file}")
        
        try:
            # Download with progress
            urllib.request.urlretrieve(source_url, target_file)
            
            # Verify download
            if target_file.exists() and target_file.stat().st_size > 0:
                print(f"  Downloaded successfully: {filename} ({target_file.stat().st_size} bytes)")
                return True
            else:
                print(f"  Download failed: {filename}")
                return False
                
        except urllib.error.URLError as e:
            print(f"  Download error: {e}")
            return False
        except Exception as e:
            print(f"  Unexpected error: {e}")
            return False
    
    def download_all_sources(self, build_order=None, force=False):
        """Download source tarballs for all packages in build order"""
        if build_order is None:
            build_order = list(self.packages.keys())
        
        print(f"Downloading sources for {len(build_order)} packages...")
        
        success_count = 0
        fail_count = 0
        
        for pkg_name in build_order:
            print(f"Downloading source for {pkg_name}...")
            if self.download_source(pkg_name, force=force):
                success_count += 1
            else:
                fail_count += 1
        
        print(f"\nDownload summary:")
        print(f"  Success: {success_count}")
        print(f"  Failed: {fail_count}")
        
        return fail_count == 0
    
    def show_package_info(self, package_name):
        """Show detailed information about a package"""
        if package_name not in self.packages:
            print(f"Package {package_name} not found")
            return
        
        config = self.packages[package_name]
        
        print(f"\nPackage: {package_name}")
        print(f"  Version: {config.get('version', 'N/A')}")
        print(f"  Release: {config.get('release', 'N/A')}")
        print(f"  Release Tag: {config.get('release_tag', 'N/A')}")
        print(f"  Repository: {config.get('repository', 'N/A')}")
        print(f"  Source URL: {config.get('source_url', 'N/A')}")
        print(f"  Build Priority: {config.get('build_priority', 'N/A')}")
        print(f"  Enable Net: {config.get('enable_net', False)}")
        print(f"  Vendor Package: {config.get('vendor_package', False)}")
        
        build_deps = config.get('build_depends', [])
        if build_deps:
            print(f"  Build Dependencies: {', '.join(build_deps)}")
        
        exec_deps = config.get('exec_depends', [])
        if exec_deps:
            print(f"  Runtime Dependencies: {', '.join(exec_deps)}")
        
        # Show what depends on this package
        dependents = [pkg for pkg, pkg_config in self.packages.items() 
                     if package_name in pkg_config.get('build_depends', []) + pkg_config.get('exec_depends', [])]
        if dependents:
            print(f"  Required by: {', '.join(dependents)}")
    
    def show_dependency_tree(self, package_name, max_depth=3):
        """Show dependency tree for a package"""
        if package_name not in self.packages:
            print(f"Package {package_name} not found")
            return
        
        def print_tree(pkg, depth=0, visited=None):
            if visited is None:
                visited = set()
            
            if depth > max_depth or pkg in visited:
                return
            
            visited.add(pkg)
            indent = "  " * depth
            print(f"{indent}{pkg}")
            
            if pkg in self.packages:
                config = self.packages[pkg]
                deps = config.get('build_depends', []) + config.get('exec_depends', [])
                rmf_deps = [dep for dep in deps if dep in self.packages]
                
                for dep in sorted(rmf_deps):
                    print_tree(dep, depth + 1, visited.copy())
        
        print(f"\nDependency tree for {package_name}:")
        print_tree(package_name)

def main():
    parser = argparse.ArgumentParser(description='Generate build order (depends JSON) from <cfg-dir>/<distro> and save to <cfg-dir>/build-order.json')
    parser.add_argument('--cfg-dir', default='cfg', help='Base configuration directory (e.g., cfg). Package YAMLs are read from <cfg-dir>/<distro>.')
    parser.add_argument('--build-dir', default='fedrmf', help='Build directory')
    parser.add_argument('--distro', default='jazzy', help='ROS distro name (e.g., jazzy)')
    parser.add_argument('--skip-package', action='append', default=[], help='Package to skip (can be specified multiple times)')
    parser.add_argument('--output', default=None, help='Path to write build order JSON (default: <cfg>/build-order.json)')

    args = parser.parse_args()

    # Resolve cfg directory: prefer <cfg-dir>/<distro> if it exists, otherwise use cfg-dir as-is
    requested_cfg = Path(args.cfg_dir)
    candidate_cfg = requested_cfg / args.distro
    resolved_cfg_dir = str(candidate_cfg if candidate_cfg.exists() else requested_cfg)

    manager = RMFDependencyManager(
        cfg_dir=resolved_cfg_dir,
        build_dir=args.build_dir,
        distro=args.distro,
        skip_packages=set(args.skip_package or [])
    )

    manager.load_package_configs()
    manager.build_dependency_graph()

    if not manager.check_circular_dependencies():
        sys.exit(1)

    build_order = manager.get_build_order()
    if not build_order:
        sys.exit(1)

    # Default output to <cfg-dir>/build-order.json if --output not provided
    output_path = args.output if args.output else str(Path(args.cfg_dir) / 'build-order.json')
    ok = manager.save_build_order(build_order, filename=output_path)
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main() 