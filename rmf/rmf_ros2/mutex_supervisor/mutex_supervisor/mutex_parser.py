#!/usr/bin/env python3

"""
Mutex Zone Parser

Generic parser for extracting mutex zones from navigation graph YAML files.
Dynamically discovers all mutex groups and computes entry/exit vertices.
"""

import yaml
import math
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class MutexZone:
    """Represents a mutex zone with entry and exit coordinates"""
    name: str
    entry: Tuple[float, float]  # (x, y) in map frame
    exit: Tuple[float, float]    # (x, y) in map frame
    vertices: List[Tuple[float, float]]  # All vertices in the zone


class MutexZoneParser:
    """Parser for extracting mutex zones from navigation graph YAML"""
    
    def __init__(self, nav_graph_path: str, logger=None):
        """
        Initialize parser
        
        Args:
            nav_graph_path: Path to the navigation graph YAML file (0.yaml)
            logger: Optional logger for debug messages
        """
        self.nav_graph_path = nav_graph_path
        self.logger = logger
    
    def _log(self, level: str, message: str):
        """Log message if logger is available"""
        if self.logger:
            if level == 'info':
                self.logger.info(message)
            elif level == 'warn':
                self.logger.warn(message)
            elif level == 'debug':
                self.logger.debug(message)
            elif level == 'error':
                self.logger.error(message)
    
    def parse_mutex_zones(self) -> Dict[str, Dict]:
        """
        Parse mutex zones from navigation graph YAML file
        
        Returns:
            Dictionary mapping mutex name to zone info:
            {
                'M1': {
                    'name': 'M1',
                    'entry': (x, y),
                    'exit': (x, y),
                    'vertices': [(x1, y1), (x2, y2), ...]
                },
                ...
            }
        """
        with open(self.nav_graph_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # Extract level data (assuming first level for now)
        levels = data.get('levels', {})
        if not levels:
            self._log('error', 'No levels found in navigation graph')
            return {}
        
        # Get first level (typically L1)
        level_name = list(levels.keys())[0]
        level_data = levels[level_name]
        
        # Parse vertices
        vertices = level_data.get('vertices', [])
        if not vertices:
            self._log('error', 'No vertices found in navigation graph')
            return {}
        
        # Build vertex index -> coordinates mapping
        vertex_coords = {}
        for i, vertex in enumerate(vertices):
            if isinstance(vertex, list) and len(vertex) >= 2:
                x = float(vertex[0])
                y = float(vertex[1])
                vertex_coords[i] = (x, y)
        
        # Group vertices by mutex name
        mutex_vertices: Dict[str, List[Tuple[int, Tuple[float, float]]]] = {}
        
        for i, vertex in enumerate(vertices):
            if isinstance(vertex, list) and len(vertex) >= 3:
                properties = vertex[2] if isinstance(vertex[2], dict) else {}
                mutex_name = properties.get('mutex', '')
                
                if mutex_name:
                    if mutex_name not in mutex_vertices:
                        mutex_vertices[mutex_name] = []
                    
                    if i in vertex_coords:
                        mutex_vertices[mutex_name].append((i, vertex_coords[i]))
        
        # Process each mutex group
        # Store all vertices for each mutex (no pre-ordering)
        # Entry/exit will be determined based on robot's actual path order
        mutex_zones = {}
        
        for mutex_name, vertex_list in mutex_vertices.items():
            if len(vertex_list) < 1:
                self._log('warn', f'Mutex {mutex_name} has no vertices, skipping')
                continue
            
            # Extract all vertex coordinates (keep all, don't pre-order)
            all_vertices = [v[1] for v in vertex_list]
            
            mutex_zones[mutex_name] = {
                'name': mutex_name,
                'vertices': all_vertices  # All mutex waypoints for this mutex name
            }
            
            self._log('debug', 
                f'Mutex {mutex_name}: {len(all_vertices)} vertices (entry/exit determined by robot path)'
            )
        
        return mutex_zones
    
    def _order_vertices_along_corridor(
        self, 
        vertex_list: List[Tuple[int, Tuple[float, float]]]
    ) -> List[Tuple[int, Tuple[float, float]]]:
        """
        Order vertices along a corridor by finding the axis with largest spread
        and sorting along that axis.
        
        Args:
            vertex_list: List of (vertex_index, (x, y)) tuples
            
        Returns:
            Ordered list of (vertex_index, (x, y)) tuples
        """
        if len(vertex_list) <= 1:
            return vertex_list
        
        # Extract coordinates
        coords = [v[1] for v in vertex_list]
        
        # Find axis with largest spread
        x_coords = [c[0] for c in coords]
        y_coords = [c[1] for c in coords]
        
        x_spread = max(x_coords) - min(x_coords)
        y_spread = max(y_coords) - min(y_coords)
        
        # Determine sort axis and direction
        if x_spread >= y_spread:
            # Sort by x coordinate
            sort_axis = 0  # x
            # Determine direction: if first vertex has smaller x, sort ascending
            # Otherwise sort descending
            first_x = coords[0][0]
            last_x = coords[-1][0]
            reverse = first_x > last_x
        else:
            # Sort by y coordinate
            sort_axis = 1  # y
            first_y = coords[0][1]
            last_y = coords[-1][1]
            reverse = first_y > last_y
        
        # Sort vertices along the chosen axis
        sorted_vertices = sorted(
            vertex_list,
            key=lambda v: v[1][sort_axis],
            reverse=reverse
        )
        
        # Optional safety check: verify consecutive points are close enough
        # Warn if corridor seems branched
        max_gap = 0.0
        for i in range(len(sorted_vertices) - 1):
            p1 = sorted_vertices[i][1]
            p2 = sorted_vertices[i + 1][1]
            distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            max_gap = max(max_gap, distance)
        
        # Warn if gap is suspiciously large (more than 2x average spacing)
        if len(sorted_vertices) > 2:
            avg_spacing = max_gap / (len(sorted_vertices) - 1) if len(sorted_vertices) > 1 else 0
            if max_gap > 2.0 * avg_spacing and max_gap > 1.0:
                self._log('warn', 
                    f'Large gap detected in mutex zone ordering: {max_gap:.2f}m. '
                    f'Corridor may be branched or vertices may be misordered.'
                )
        
        return sorted_vertices

