#!/usr/bin/env python3
"""
Path Configuration Helper for Mars Simulation
This module provides utilities to resolve paths from the centralized config file.
"""

import os
import yaml
import rospkg
from pathlib import Path


class PathConfig:
    """Manages path configuration for the mars simulation workspace."""
    
    def __init__(self, config_file=None):
        """
        Initialize PathConfig.
        
        Args:
            config_file: Path to the path_config.yaml file. 
                        If None, searches for it in standard locations.
        """
        self.rospack = rospkg.RosPack()
        
        # Find config file
        if config_file is None:
            config_file = self._find_config_file()
        
        # Load configuration
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Cache for resolved paths
        self._path_cache = {}
    
    def _find_config_file(self):
        """Find the path_config.yaml file."""
        # Try environment variable first
        if 'MARS_SIM_CONFIG' in os.environ:
            config_path = os.environ['MARS_SIM_CONFIG']
            if os.path.exists(config_path):
                return config_path
        
        # Try to find in workspace
        try:
            # Assuming this script is in world_plugins/scripts
            world_plugins_path = self.rospack.get_path('world_plugins')
            workspace_root = Path(world_plugins_path).parent
            config_path = workspace_root / 'config' / 'path_config.yaml'
            if config_path.exists():
                return str(config_path)
        except Exception:
            pass
        
        # Fallback to relative path
        script_dir = Path(__file__).parent.parent.parent
        config_path = script_dir / 'config' / 'path_config.yaml'
        if config_path.exists():
            return str(config_path)
        
        raise FileNotFoundError("Could not find path_config.yaml")
    
    def _resolve_ros_path(self, path_str):
        """
        Resolve ROS package paths like $(find package_name)/path.
        
        Args:
            path_str: String possibly containing $(find package) syntax
            
        Returns:
            Resolved absolute path
        """
        if path_str in self._path_cache:
            return self._path_cache[path_str]
        
        resolved = path_str
        
        # Handle $(find package_name) syntax
        if '$(find ' in resolved:
            import re
            pattern = r'\$\(find\s+(\w+)\)'
            
            def replace_find(match):
                package_name = match.group(1)
                try:
                    return self.rospack.get_path(package_name)
                except rospkg.ResourceNotFound:
                    print(f"Warning: Package '{package_name}' not found")
                    return match.group(0)
            
            resolved = re.sub(pattern, replace_find, resolved)
        
        # Handle environment variables
        resolved = os.path.expandvars(resolved)
        
        # Convert to absolute path
        resolved = os.path.abspath(resolved)
        
        self._path_cache[path_str] = resolved
        return resolved
    
    def get_path(self, *keys, create_if_missing=False):
        """
        Get a path from the configuration.
        
        Args:
            *keys: Nested keys to access the path (e.g., 'terrain', 'heightmap_dir')
            create_if_missing: If True, create the directory if it doesn't exist
            
        Returns:
            Resolved absolute path as string
        """
        # Navigate through nested dictionary
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                raise KeyError(f"Path not found: {'.'.join(keys)}")
        
        # Resolve the path
        resolved_path = self._resolve_ros_path(value)
        
        # Create directory if requested
        if create_if_missing:
            os.makedirs(resolved_path, exist_ok=True)
        
        return resolved_path
    
    def get_heightmap_path(self):
        """Get the full path to the heightmap file."""
        heightmap_dir = self.get_path('terrain', 'heightmap_dir')
        heightmap_name = self.config['terrain']['heightmap_name']
        return os.path.join(heightmap_dir, heightmap_name)
    
    def get_terrain_data_file(self):
        """Get the path to the simulated terrain data file."""
        return self.get_path('data', 'simulated_terrain_data')
    
    def get_plugin_config(self):
        """Get the path to the terramechanics plugin config."""
        return self.get_path('plugins', 'terramechanics_config')
    
    def update_config_value(self, value, *keys):
        """
        Update a value in the configuration.
        
        Args:
            value: New value to set
            *keys: Nested keys to access the location
        """
        # Navigate to parent
        config = self.config
        for key in keys[:-1]:
            if key not in config:
                config[key] = {}
            config = config[key]
        
        # Set the value
        config[keys[-1]] = value
        
        # Clear cache
        self._path_cache.clear()


# Global instance for easy access
_global_config = None


def get_config(config_file=None):
    """
    Get the global PathConfig instance.
    
    Args:
        config_file: Optional path to config file (only used on first call)
        
    Returns:
        PathConfig instance
    """
    global _global_config
    if _global_config is None:
        _global_config = PathConfig(config_file)
    return _global_config


# Convenience functions
def get_path(*keys, **kwargs):
    """Shortcut to get_config().get_path()"""
    return get_config().get_path(*keys, **kwargs)


def get_heightmap_path():
    """Shortcut to get the heightmap path"""
    return get_config().get_heightmap_path()


def get_terrain_data_file():
    """Shortcut to get the terrain data file path"""
    return get_config().get_terrain_data_file()


if __name__ == '__main__':
    # Test the configuration
    config = PathConfig()
    
    print("Path Configuration Test")
    print("=" * 50)
    
    try:
        print(f"Heightmap dir: {config.get_path('terrain', 'heightmap_dir')}")
        print(f"Heightmap full path: {config.get_heightmap_path()}")
        print(f"Terrain data file: {config.get_terrain_data_file()}")
        print(f"Plugin config: {config.get_plugin_config()}")
        print(f"World save path: {config.get_path('world', 'world_save_path')}")
        print("\nAll paths resolved successfully!")
    except Exception as e:
        print(f"Error: {e}")
