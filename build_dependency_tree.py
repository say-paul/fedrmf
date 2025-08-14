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
from typing import Dict, List, Set, Tuple
import hashlib
import networkx as nx

class RMFDependencyManager:
    def __init__(self, cfg_dir='cfg', sources_dir='fedrmf/SOURCES', build_dir='fedrmf'):
        self.cfg_dir = Path(cfg_dir)
        self.sources_dir = Path(sources_dir)
        self.build_dir = Path(build_dir)
        
        # Ensure directories exist
        self.sources_dir.mkdir(parents=True, exist_ok=True)
        self.build_dir.mkdir(parents=True, exist_ok=True)
        
        self.packages = {}
        self.dependency_graph = nx.DiGraph()
        
    def load_package_configs(self):
        """Load all package configurations from YAML files"""
        yaml_files = list(self.cfg_dir.glob('*.yaml'))
        yaml_files = [f for f in yaml_files if not f.name.startswith('_') 
                     and f.name not in ['rmf_deps.yaml', 'files.yaml']]
        
        print(f"Loading {len(yaml_files)} package configurations...")
        
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as f:
                    config = yaml.safe_load(f)
                
                package_name = config.get('package_name')
                if package_name:
                    self.packages[package_name] = config
                    print(f"  Loaded: {package_name}")
                else:
                    print(f"  Warning: No package_name in {yaml_file}")
                    
            except Exception as e:
                print(f"  Error loading {yaml_file}: {e}")
        
        print(f"Loaded {len(self.packages)} packages.")
        
    def build_dependency_graph(self):
        """Build dependency graph from package configurations"""
        print("Building dependency graph...")
        
        # Add all packages as nodes
        for pkg_name, config in self.packages.items():
            self.dependency_graph.add_node(pkg_name, **config)
        
        # Add dependency edges
        for pkg_name, config in self.packages.items():
            build_deps = config.get('build_depends', [])
            exec_deps = config.get('exec_depends', [])
            
            # Combine build and exec dependencies
            all_deps = set(build_deps + exec_deps)
            
            for dep in all_deps:
                # Only add edges for RMF packages (ignore system/ROS packages)
                if dep in self.packages:
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
    
    def save_build_order(self, build_order, filename='cfg/build_order.json'):
        """Save build order to JSON file"""
        build_data = {
            'build_order': build_order,
            'packages': {},
            'dependency_info': {
                'total_packages': len(build_order),
                'has_cycles': False
            }
        }
        
        # Add package information
        for pkg_name in build_order:
            if pkg_name in self.packages:
                config = self.packages[pkg_name]
                build_data['packages'][pkg_name] = {
                    'build_priority': config.get('build_priority', 999),
                    'build_depends': config.get('build_depends', []),
                    'exec_depends': config.get('exec_depends', []),
                    'enable_net': config.get('enable_net', False),
                    'vendor_package': config.get('vendor_package', False)
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
    parser = argparse.ArgumentParser(description='RMF Dependency Manager')
    parser.add_argument('--cfg-dir', default='cfg', help='Configuration directory')
    parser.add_argument('--sources-dir', default='fedrmf/SOURCES', help='Sources directory')
    parser.add_argument('--build-dir', default='fedrmf', help='Build directory')
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Build order command
    build_parser = subparsers.add_parser('build-order', help='Generate build order')
    build_parser.add_argument('--save', action='store_true', help='Save build order to file')
    
    # Download command
    download_parser = subparsers.add_parser('download', help='Download source tarballs')
    download_parser.add_argument('--package', help='Download specific package')
    download_parser.add_argument('--all', action='store_true', help='Download all packages')
    download_parser.add_argument('--force', action='store_true', help='Force re-download')
    download_parser.add_argument('--build-order', action='store_true', help='Use build order for downloads')
    
    # Info command
    info_parser = subparsers.add_parser('info', help='Show package information')
    info_parser.add_argument('package', help='Package name')
    
    # Tree command
    tree_parser = subparsers.add_parser('tree', help='Show dependency tree')
    tree_parser.add_argument('package', help='Package name')
    tree_parser.add_argument('--depth', type=int, default=3, help='Maximum depth')
    
    # List command
    list_parser = subparsers.add_parser('list', help='List packages')
    list_parser.add_argument('--with-deps', action='store_true', help='Show dependencies')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    manager = RMFDependencyManager(
        cfg_dir=args.cfg_dir,
        sources_dir=args.sources_dir,
        build_dir=args.build_dir
    )
    
    manager.load_package_configs()
    manager.build_dependency_graph()
    
    if args.command == 'build-order':
        if not manager.check_circular_dependencies():
            sys.exit(1)
        
        build_order = manager.get_build_order()
        if not build_order:
            sys.exit(1)
        
        if args.save:
            manager.save_build_order(build_order)
    
    elif args.command == 'download':
        if args.package:
            success = manager.download_source(args.package, force=args.force)
            sys.exit(0 if success else 1)
        elif args.all or args.build_order:
            build_order = None
            if args.build_order:
                if not manager.check_circular_dependencies():
                    sys.exit(1)
                build_order = manager.get_build_order()
                if not build_order:
                    sys.exit(1)
            
            success = manager.download_all_sources(build_order, force=args.force)
            sys.exit(0 if success else 1)
        else:
            print("Error: Must specify --package, --all, or --build-order")
            sys.exit(1)
    
    elif args.command == 'info':
        manager.show_package_info(args.package)
    
    elif args.command == 'tree':
        manager.show_dependency_tree(args.package, max_depth=args.depth)
    
    elif args.command == 'list':
        packages = sorted(manager.packages.keys())
        print(f"Available packages ({len(packages)}):")
        for pkg in packages:
            if args.with_deps:
                config = manager.packages[pkg]
                deps = config.get('build_depends', []) + config.get('exec_depends', [])
                rmf_deps = [dep for dep in deps if dep in manager.packages]
                if rmf_deps:
                    print(f"  {pkg} -> {', '.join(rmf_deps)}")
                else:
                    print(f"  {pkg}")
            else:
                print(f"  {pkg}")

if __name__ == '__main__':
    main() 