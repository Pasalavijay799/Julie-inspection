"""
Configuration manager for JAKA Visual Inspection.

Handles loading/saving settings from config.ini files.
"""

import os
import numpy as np
from configparser import ConfigParser


# Default settings for JAKA ZU5
DEFAULT_SETTINGS = {
    'samples': 1,
    'spacing': 0.01,
    'offset_y': 0.10,
    'offset_z': 0.0,
    'trim_base': 0.05,
    'manual_offset': 0.0,
    'cluster_centered': True,
    'cluster_idx': 0,
    'cluster_discard': 20,
    'eps': 0.05,
    'min_points': 10,
    'cluster_trim': 0.0,
    'tgt_coord_samples': 3,
    'tgt_final_trim': 0.0,
    'tgt_reverse': True,
    'tgt_preview': True,
    'z_offset': 0.25,
    'coord_skip': 2,
    'tgt_motion_delay': 0.0,
    'tgt_save': True,
    'dbug': False,
}

# JAKA ZU5 initial poses (6-DOF joint values in radians)
# These define common scanning positions for the JAKA ZU5 with camera
DEFAULT_POSES = {
    'down_default': [0.0, 1.5707, -1.5707, 1.5707, 1.5707, 0.0],
    'down_high': [0.0, 0.7854, -0.7854, 0.0, 1.5707, 0.0],
    'front_low': [0.0, 1.5707, -0.7854, 1.5707, 1.5707, 0.0],
    'front_med': [0.0, 1.2, -1.0, 1.2, 1.5707, 0.0],
    'left_low': [1.5707, 1.5707, -0.7854, 1.5707, 1.5707, 0.0],
    'right_low': [-1.5707, 1.5707, -0.7854, 1.5707, 1.5707, 0.0],
    'front_box': [0.0, 1.5707, 0.0, 0.0, 0.0, -2.4758],
    'current_pose': None,  # Will use whatever the robot's current pose is
}


class ConfigManager:
    """Manages configuration loading and saving for the auto planner."""

    def __init__(self, config_path=None):
        """
        Args:
            config_path: path to config.ini file. If None, uses defaults.
        """
        self.config = ConfigParser()
        self.settings = dict(DEFAULT_SETTINGS)
        self.poses = dict(DEFAULT_POSES)
        self.config_path = config_path

        if config_path and os.path.exists(config_path):
            self.load(config_path)

    def load(self, config_path):
        """Load settings from config.ini file."""
        self.config_path = config_path
        self.config.read(config_path)

        if 'Settings' in self.config:
            s = self.config['Settings']
            self.settings['samples'] = s.getint('samples', DEFAULT_SETTINGS['samples'])
            self.settings['spacing'] = s.getfloat('spacing', DEFAULT_SETTINGS['spacing'])
            self.settings['offset_y'] = s.getfloat('offset_y', DEFAULT_SETTINGS['offset_y'])
            self.settings['offset_z'] = s.getfloat('offset_z', DEFAULT_SETTINGS['offset_z'])
            self.settings['trim_base'] = s.getfloat('trim_base', DEFAULT_SETTINGS['trim_base'])
            self.settings['manual_offset'] = s.getfloat('manual_offset', DEFAULT_SETTINGS['manual_offset'])
            self.settings['cluster_centered'] = s.getboolean('cluster_centered', DEFAULT_SETTINGS['cluster_centered'])
            self.settings['cluster_idx'] = s.getint('cluster_idx', DEFAULT_SETTINGS['cluster_idx'])
            self.settings['cluster_discard'] = s.getint('cluster_discard', DEFAULT_SETTINGS['cluster_discard'])
            self.settings['eps'] = s.getfloat('eps', DEFAULT_SETTINGS['eps'])
            self.settings['min_points'] = s.getint('min_points', DEFAULT_SETTINGS['min_points'])
            self.settings['cluster_trim'] = s.getfloat('cluster_trim', DEFAULT_SETTINGS['cluster_trim'])
            self.settings['tgt_coord_samples'] = s.getint('tgt_coord_samples', DEFAULT_SETTINGS['tgt_coord_samples'])
            self.settings['tgt_final_trim'] = s.getfloat('tgt_final_trim', DEFAULT_SETTINGS['tgt_final_trim'])
            self.settings['tgt_reverse'] = s.getboolean('tgt_reverse', DEFAULT_SETTINGS['tgt_reverse'])
            self.settings['tgt_preview'] = s.getboolean('tgt_preview', DEFAULT_SETTINGS['tgt_preview'])
            self.settings['z_offset'] = s.getfloat('z_offset', DEFAULT_SETTINGS['z_offset'])
            self.settings['coord_skip'] = s.getint('coord_skip', DEFAULT_SETTINGS['coord_skip'])
            self.settings['tgt_motion_delay'] = s.getfloat('tgt_motion_delay', DEFAULT_SETTINGS['tgt_motion_delay'])
            self.settings['tgt_save'] = s.getboolean('tgt_save', DEFAULT_SETTINGS['tgt_save'])
            self.settings['dbug'] = s.getboolean('dbug', DEFAULT_SETTINGS['dbug'])

        if 'Init_Pose' in self.config:
            p = self.config['Init_Pose']
            for key in self.poses:
                if key in p:
                    try:
                        vals = eval(p[key])
                        self.poses[key] = vals
                    except Exception:
                        pass

    def save(self, config_path=None):
        """Save current settings to config.ini file."""
        path = config_path or self.config_path
        if path is None:
            return

        self.config['Settings'] = {}
        for key, val in self.settings.items():
            self.config['Settings'][key] = str(val)

        self.config['Init_Pose'] = {}
        for key, val in self.poses.items():
            if val is not None:
                self.config['Init_Pose'][key] = str(val)

        with open(path, 'w') as f:
            self.config.write(f)

    def get_settings_list(self):
        """Get settings as a flat list for GUI."""
        s = self.settings
        return [
            s['samples'], s['spacing'], s['offset_y'], s['offset_z'],
            s['trim_base'], s['manual_offset'], s['cluster_centered'],
            s['cluster_idx'], s['cluster_discard'], s['eps'], s['min_points'],
            s['cluster_trim'], s['tgt_coord_samples'], s['tgt_final_trim'],
            s['tgt_reverse'], s['tgt_preview'], s['z_offset'], s['coord_skip'],
            s['tgt_motion_delay'], s['tgt_save'], s['dbug']
        ]

    def update_settings_from_list(self, settings_list):
        """Update settings from GUI output list."""
        keys = [
            'samples', 'spacing', 'offset_y', 'offset_z', 'trim_base',
            'manual_offset', 'cluster_centered', 'cluster_idx', 'cluster_discard',
            'eps', 'min_points', 'cluster_trim', 'tgt_coord_samples',
            'tgt_final_trim', 'tgt_reverse', 'tgt_preview', 'z_offset',
            'coord_skip', 'tgt_motion_delay', 'tgt_save', 'dbug'
        ]
        types = [
            int, float, float, float, float, float, bool, int, int,
            float, int, float, int, float, bool, bool, float, int,
            float, bool, bool
        ]
        for i, (key, typ) in enumerate(zip(keys, types)):
            if i < len(settings_list):
                try:
                    self.settings[key] = typ(settings_list[i])
                except (ValueError, TypeError):
                    pass

    def get_pose_names(self):
        """Get list of available pose names."""
        return list(self.poses.keys())

    def get_pose(self, name):
        """Get joint values for a named pose."""
        return self.poses.get(name, None)
