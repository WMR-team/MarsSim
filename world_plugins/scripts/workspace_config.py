#!/usr/bin/env python3
"""
Unified Path Configuration System for Mars Simulation
Provides a simple prefix-based path resolution without ROS dependencies
"""

import os
import sys
from pathlib import Path


class WorkspaceConfig:
    """
    Unified workspace configuration management.
    Supports both environment variable and direct prefix configuration.
    """
    
    # Default prefix - can be overridden by environment variable
    _workspace_root = None
    _prefix_name = "MARS_SIM_ROOT"
    
    # Path mapping
    _path_mapping = {
        'rover_gazebo': 'rover_gazebo',
        'rover_gazebo_plugins': 'rover_gazebo_plugins',
        'rover_descriptions': 'rover_descriptions',
        'rover_control': 'rover_control',
        'rover_msgs': 'rover_msgs',
        'world_plugins': 'world_plugins',
        'velodyne_simulator': 'velodyne_simulator',
    }
    
    @classmethod
    def set_workspace_root(cls, root_path):
        """
        Set the workspace root path.
        
        Args:
            root_path: Absolute path to workspace root (e.g., /home/user/mars_ws/src/mars_sim)
        """
        root_path = Path(root_path).resolve()
        if not root_path.exists():
            raise ValueError(f"Workspace root does not exist: {root_path}")
        cls._workspace_root = root_path
    
    @classmethod
    def get_workspace_root(cls):
        """
        Get the workspace root path.
        Uses environment variable MARS_SIM_ROOT if set, otherwise uses detected or configured root.
        """
        if cls._workspace_root is None:
            # Try environment variable first
            env_root = os.environ.get(cls._prefix_name)
            if env_root and Path(env_root).exists():
                cls._workspace_root = Path(env_root).resolve()
            else:
                # Try to detect from script location
                # This file should be in world_plugins/scripts/
                script_dir = Path(__file__).parent.parent.parent.resolve()
                if (script_dir / 'rover_gazebo').exists():
                    cls._workspace_root = script_dir
                else:
                    raise RuntimeError(
                        f"Cannot determine workspace root. "
                        f"Please set {cls._prefix_name} environment variable or call set_workspace_root()"
                    )
        
        return cls._workspace_root
    
    @classmethod
    def get_package_path(cls, package_name):
        """
        Get the absolute path to a package.
        
        Args:
            package_name: Name of the package (e.g., 'rover_gazebo', 'world_plugins')
            
        Returns:
            Absolute path to the package
        """
        if package_name not in cls._path_mapping:
            raise ValueError(f"Unknown package: {package_name}")
        
        root = cls.get_workspace_root()
        package_path = root / cls._path_mapping[package_name]
        
        if not package_path.exists():
            raise FileNotFoundError(f"Package not found: {package_path}")
        
        return package_path
    
    @classmethod
    def resolve_path(cls, package_name, *path_parts):
        """
        Resolve a path within a package.
        
        Args:
            package_name: Name of the package
            *path_parts: Path components relative to package root
            
        Returns:
            Absolute path as string
        """
        package_path = cls.get_package_path(package_name)
        full_path = package_path / Path(*path_parts)
        return str(full_path)
    
    @classmethod
    def get_data_path(cls, *path_parts):
        """Get path in rover_gazebo/data directory."""
        return cls.resolve_path('rover_gazebo', 'data', *path_parts)
    
    @classmethod
    def get_models_path(cls, *path_parts):
        """Get path in rover_gazebo/models directory."""
        return cls.resolve_path('rover_gazebo', 'models', *path_parts)
    
    @classmethod
    def get_config_path(cls, *path_parts):
        """Get path in world_plugins/config directory."""
        return cls.resolve_path('world_plugins', 'config', *path_parts)
    
    @classmethod
    def get_scripts_path(cls, *path_parts):
        """Get path in world_plugins/scripts directory."""
        return cls.resolve_path('world_plugins', 'scripts', *path_parts)
    
    @classmethod
    def get_worlds_path(cls, *path_parts):
        """Get path in rover_gazebo/worlds directory."""
        return cls.resolve_path('rover_gazebo', 'worlds', *path_parts)
    
    @classmethod
    def get_launch_path(cls, *path_parts):
        """Get path in rover_gazebo/launch directory."""
        return cls.resolve_path('rover_gazebo', 'launch', *path_parts)
    
    @classmethod
    def print_config(cls):
        """Print current configuration."""
        try:
            root = cls.get_workspace_root()
            print(f"Workspace Root: {root}")
            print(f"Available packages:")
            for pkg_name, pkg_dir in cls._path_mapping.items():
                pkg_path = root / pkg_dir
                exists = "✓" if pkg_path.exists() else "✗"
                print(f"  {exists} {pkg_name}: {pkg_path}")
        except Exception as e:
            print(f"Error: {e}")


# Convenience functions
def get_workspace_root():
    """Get workspace root path."""
    return str(WorkspaceConfig.get_workspace_root())


def get_package_path(package_name):
    """Get package path."""
    return str(WorkspaceConfig.get_package_path(package_name))


def resolve_path(package_name, *path_parts):
    """Resolve path in package."""
    return WorkspaceConfig.resolve_path(package_name, *path_parts)


def get_data_path(*path_parts):
    """Get data path."""
    return WorkspaceConfig.get_data_path(*path_parts)


def get_models_path(*path_parts):
    """Get models path."""
    return WorkspaceConfig.get_models_path(*path_parts)


def get_config_path(*path_parts):
    """Get config path."""
    return WorkspaceConfig.get_config_path(*path_parts)


def get_scripts_path(*path_parts):
    """Get scripts path."""
    return WorkspaceConfig.get_scripts_path(*path_parts)


def get_worlds_path(*path_parts):
    """Get worlds path."""
    return WorkspaceConfig.get_worlds_path(*path_parts)


def get_launch_path(*path_parts):
    """Get launch path."""
    return WorkspaceConfig.get_launch_path(*path_parts)


if __name__ == '__main__':
    # Test the configuration
    print("Mars Simulation Workspace Configuration Test")
    print("=" * 60)
    
    # Set workspace root if provided as argument
    if len(sys.argv) > 1:
        try:
            WorkspaceConfig.set_workspace_root(sys.argv[1])
        except Exception as e:
            print(f"Error setting workspace root: {e}")
            sys.exit(1)
    
    try:
        WorkspaceConfig.print_config()
        print("\nCommon paths:")
        print(f"  Data: {get_data_path()}")
        print(f"  Models: {get_models_path()}")
        print(f"  Config: {get_config_path()}")
        print(f"  Scripts: {get_scripts_path()}")
        print(f"  Worlds: {get_worlds_path()}")
        print("\nConfiguration successful!")
    except Exception as e:
        print(f"Error: {e}")
        print(f"\nPlease set the {WorkspaceConfig._prefix_name} environment variable:")
        print(f"  export {WorkspaceConfig._prefix_name}=/path/to/mars_sim")
        sys.exit(1)
