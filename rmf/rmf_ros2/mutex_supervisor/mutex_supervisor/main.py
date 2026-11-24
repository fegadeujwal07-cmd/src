#!/usr/bin/env python3

"""
Mutex Supervisor Node

External mutex supervisor for Nav2-based fleet management with RMF.
Monitors robot plans and positions to enforce mutex zone access control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math
import time
from typing import Dict, List, Tuple, Optional
from collections import deque

from rmf_traffic_msgs.msg import ItinerarySet
from rmf_fleet_msgs.msg import FleetState, RobotState, PauseRequest, MutexGroupRequestStatus


class MutexSupervisor(Node):
    """Mutex supervisor node for managing mutex zone access"""
    
    def __init__(self):
        super().__init__('mutex_supervisor')
        
        # Declare parameters
        self.declare_parameter('nav_graph_path', '')
        self.declare_parameter('match_tolerance', 0.5)
        self.declare_parameter('claim_distance', 4.0)
        self.declare_parameter('exit_reached_distance', 0.7)
        self.declare_parameter('clearance_distance', 0.9)
        
        # Get parameters
        nav_graph_path = self.get_parameter('nav_graph_path').value
        self.match_tolerance = self.get_parameter('match_tolerance').value
        self.claim_distance = self.get_parameter('claim_distance').value
        self.exit_reached_distance = self.get_parameter('exit_reached_distance').value
        self.clearance_distance = self.get_parameter('clearance_distance').value
        
        # Command cooldown period (seconds) to prevent spam (internal constant)
        self.command_cooldown = 0.5
        
        if not nav_graph_path:
            self.get_logger().error('nav_graph_path parameter is required')
            raise ValueError('nav_graph_path parameter must be set')
        
        # Load mutex zones from nav graph
        from mutex_supervisor.mutex_parser import MutexZoneParser
        parser = MutexZoneParser(nav_graph_path, self.get_logger())
        self.mutex_zones = parser.parse_mutex_zones()
        
        # Print parameter values on startup
        self.get_logger().info('=' * 60)
        self.get_logger().info('Mutex Supervisor - Configuration Parameters')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  nav_graph_path: {nav_graph_path}')
        self.get_logger().info(f'  match_tolerance: {self.match_tolerance} m')
        self.get_logger().info(f'  claim_distance: {self.claim_distance} m')
        self.get_logger().info(f'  exit_reached_distance: {self.exit_reached_distance} m')
        self.get_logger().info(f'  clearance_distance: {self.clearance_distance} m')
        self.get_logger().info(f'  command_cooldown: {self.command_cooldown} s')
        self.get_logger().info('=' * 60)
        
        self.get_logger().info(f'Loaded {len(self.mutex_zones)} mutex groups from nav graph')
        for name, zone in self.mutex_zones.items():
            self.get_logger().info(
                f'  {name}: {len(zone["vertices"])} mutex waypoints '
                f'(entry/exit detected dynamically per robot per plan)'
            )
        
        # Robot state tracking
        # participant_id -> robot state (from itinerary_set)
        self.robot_states: Dict[int, Dict] = {}
        
        # robot_name -> participant_id mapping (for matching fleet_states to itinerary data)
        self.robot_name_to_participant: Dict[str, int] = {}
        
        # participant_id -> robot_name mapping (reverse)
        self.participant_to_robot_name: Dict[int, str] = {}
        
        # robot_name -> fleet_name mapping (from fleet_states)
        self.robot_name_to_fleet: Dict[str, str] = {}
        
        # robot_name -> latest position mapping (from fleet_states, for matching)
        self.robot_positions: Dict[str, tuple] = {}  # robot_name -> (x, y)
        
        # Rate limiting for publishing: (robot_name, mutex_name) -> last_publish_time
        self.last_publish_time: Dict[tuple, float] = {}  # (robot_name, mutex_name) -> timestamp
        self.publish_interval: float = 1.0  # Publish at most once per second
        
        # robot_name -> pause command tracking
        # {robot_name: {'pause_sent': bool, 'resume_sent': bool, 'mode_request_id': int, 'last_cmd_time': float}}
        self.robot_pause_tracking: Dict[str, Dict] = {}
        
        # Mapping uncertainty tracking
        self.mapping_warnings_shown: set = set()
        
        # Mutex state machine (per mutex)
        # mutex_name -> {locked: bool, owner: robot_name | None, queue: deque}
        self.mutex_states: Dict[str, Dict] = {}
        for mutex_name in self.mutex_zones.keys():
            self.mutex_states[mutex_name] = {
                'locked': False,
                'owner': None,
                'queue': deque()
            }
        
        # Setup QoS for RMF topics
        rmf_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Step 3: Subscribe to /rmf_traffic/itinerary_set
        self.itinerary_sub = self.create_subscription(
            ItinerarySet,
            '/rmf_traffic/itinerary_set',
            self._itinerary_set_callback,
            rmf_qos
        )
        
        self.get_logger().info('Subscribed to /rmf_traffic/itinerary_set')
        
        # Step 4: Subscribe to /fleet_states
        self.fleet_states_sub = self.create_subscription(
            FleetState,
            '/fleet_states',
            self._fleet_states_callback,
            rmf_qos
        )
        
        self.get_logger().info('Subscribed to /fleet_states')
        
        # Step 6: Publisher for /robot_pause_requests
        self.pause_request_pub = self.create_publisher(
            PauseRequest,
            '/robot_pause_requests',
            rmf_qos
        )
        
        self.get_logger().info('Publisher created for /robot_pause_requests')
        self.get_logger().info(f'Publisher QoS: depth={rmf_qos.depth}, reliability={rmf_qos.reliability}, durability={rmf_qos.durability}')
        
        # Unified mutex group request status publisher
        # Publishes status for all robots requesting/approaching mutexes
        # Uses proper ROS2 message type (like FleetState) for proper display in ros2 topic echo
        self.mutex_group_request_pub = self.create_publisher(
            MutexGroupRequestStatus,
            '/mutex_group_request',
            rmf_qos
        )
        
        self.get_logger().info('Publisher created for /mutex_group_request (unified status stream)')
    
    def _euclidean_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two 2D points"""
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    
    def _find_closest_waypoint_index(
        self,
        target: Tuple[float, float],
        waypoints: List[Tuple[float, float]],
        tolerance: float
    ) -> Optional[int]:
        """
        Find the index of the waypoint closest to target coordinate
        
        Args:
            target: (x, y) target coordinate
            waypoints: List of (x, y) waypoint coordinates
            tolerance: Maximum distance to consider a match
            
        Returns:
            Index of closest waypoint if within tolerance, None otherwise
        """
        if not waypoints:
            return None
        
        min_distance = float('inf')
        closest_index = None
        
        for i, waypoint in enumerate(waypoints):
            distance = self._euclidean_distance(target, waypoint)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        if min_distance <= tolerance:
            return closest_index
        
        return None
    
    def _extract_itinerary_points(self, msg: ItinerarySet) -> List[Tuple[float, float]]:
        """
        Extract all waypoint coordinates from itinerary_set message
        
        Args:
            msg: ItinerarySet message
            
        Returns:
            List of (x, y) coordinates from all itineraries
        """
        itinerary_points = []
        
        for itinerary in msg.itinerary:
            for wp in itinerary.trajectory.waypoints:
                try:
                    # Extract position (x, y) - ignore z/yaw for 2D matching
                    if hasattr(wp.position, '__len__') and len(wp.position) >= 2:
                        x = float(wp.position[0])
                        y = float(wp.position[1])
                    elif hasattr(wp.position, '__iter__'):
                        pos_list = list(wp.position)
                        x = float(pos_list[0])
                        y = float(pos_list[1])
                    else:
                        x = float(wp.position[0])
                        y = float(wp.position[1])
                    
                    itinerary_points.append((x, y))
                except (IndexError, TypeError, AttributeError) as e:
                    self.get_logger().warn(f'Error extracting waypoint position: {e}')
                    continue
        
        return itinerary_points
    
    def _detect_mutex_intersections(
        self,
        itinerary_points: List[Tuple[float, float]]
    ) -> Dict[str, Dict]:
        """
        Detect which mutex zones intersect with the itinerary trajectory
        
        Checks if robot's path passes through ANY mutex waypoint for each mutex name.
        Entry/exit are determined by the order in the robot's path.
        
        Args:
            itinerary_points: List of (x, y) coordinates from itinerary
            
        Returns:
            Dictionary mapping mutex name to intersection info:
            {
                'Mutex_zone_1': {
                    'entry_index': 5,
                    'exit_index': 12,
                    'entry_coord': (x, y),
                    'exit_coord': (x, y)
                },
                ...
            }
        """
        intersections = {}
        
        for mutex_name, mutex_zone in self.mutex_zones.items():
            mutex_vertices = mutex_zone['vertices']  # All waypoints for this mutex
            
            # Find all itinerary waypoints that match any mutex vertex
            matching_indices = []
            matching_itinerary_coords = []  # Store actual itinerary coordinates, not mutex vertices
            
            for i, itinerary_point in enumerate(itinerary_points):
                for mutex_vertex in mutex_vertices:
                    distance = self._euclidean_distance(itinerary_point, mutex_vertex)
                    if distance <= self.match_tolerance:
                        matching_indices.append(i)
                        matching_itinerary_coords.append(itinerary_point)  # Store itinerary point, not mutex vertex
                        break  # Found match, move to next itinerary point
            
            # If we found matches, determine entry/exit based on robot's path order
            if len(matching_indices) >= 1:
                # Entry is first match, exit is last match (based on robot's path)
                entry_index = matching_indices[0]
                exit_index = matching_indices[-1]
                # Use actual itinerary coordinates, not mutex vertex coordinates
                entry_coord = matching_itinerary_coords[0]  # entry_point_from_itinerary
                exit_coord = matching_itinerary_coords[-1]  # exit_point_from_itinerary
                
                # Only record if exit comes after entry (robot traverses the mutex)
                if exit_index > entry_index:
                    intersections[mutex_name] = {
                        'entry_index': entry_index,
                        'exit_index': exit_index,
                        'entry_coord': entry_coord,  # entry_point_from_itinerary
                        'exit_coord': exit_coord     # exit_point_from_itinerary
                    }
                    self.get_logger().info(
                        f'✅ Mutex {mutex_name} detected in itinerary: '
                        f'entry at waypoint {entry_index} {entry_coord} (from itinerary), '
                        f'exit at waypoint {exit_index} {exit_coord} (from itinerary) '
                        f'({len(matching_indices)} matching waypoints)'
                    )
                elif exit_index == entry_index:
                    # Robot passes through only one mutex waypoint
                    intersections[mutex_name] = {
                        'entry_index': entry_index,
                        'exit_index': exit_index,
                        'entry_coord': entry_coord,  # entry_point_from_itinerary
                        'exit_coord': exit_coord     # exit_point_from_itinerary
                    }
                    self.get_logger().info(
                        f'✅ Mutex {mutex_name} detected in itinerary: '
                        f'single waypoint {entry_index} {entry_coord} (from itinerary)'
                    )
                else:
                    self.get_logger().warn(
                        f'⚠️ Mutex {mutex_name} found but exit ({exit_index}) '
                        f'comes before entry ({entry_index}) - skipping'
                    )
            else:
                self.get_logger().debug(
                    f'Mutex {mutex_name} not found in itinerary '
                    f'(checked {len(mutex_vertices)} mutex waypoints)'
                )
        
        return intersections
    
    def _itinerary_set_callback(self, msg: ItinerarySet):
        """
        Callback for /rmf_traffic/itinerary_set topic
        
        Detects mutex zone intersections in the planned route.
        """
        try:
            participant_id = msg.participant
            plan_id = msg.plan
            
            self.get_logger().info(
                f'📨 [ITINERARY SET] Received for participant {participant_id}, plan {plan_id}'
            )
            
            # Extract itinerary points
            itinerary_points = self._extract_itinerary_points(msg)
            
            if not itinerary_points:
                self.get_logger().warn(
                    f'⚠️ [ITINERARY SET] No waypoints extracted for participant {participant_id}'
                )
                return
            
            self.get_logger().info(
                f'📋 [ITINERARY SET] Extracted {len(itinerary_points)} waypoints for participant {participant_id}'
            )
            
            # Detect mutex intersections
            mutex_intersections = self._detect_mutex_intersections(itinerary_points)
            
            # Map participant_id <-> robot_name primarily using the robot_name field
            # now available in ItinerarySet.msg. This makes the mapping explicit
            # and avoids relying on position heuristics.
            robot_name_from_msg = getattr(msg, 'robot_name', '')
            if robot_name_from_msg:
                # Record/refresh the explicit mapping using the itinerary_set robot_name
                prev_robot = self.participant_to_robot_name.get(participant_id)
                if prev_robot and prev_robot != robot_name_from_msg:
                    self.get_logger().warn(
                        f'⚠️ [ITINERARY SET] participant_id={participant_id} was previously '
                        f'mapped to robot_name={prev_robot}, now seen as '
                        f'robot_name={robot_name_from_msg}. Updating mapping.'
                    )
                self.participant_to_robot_name[participant_id] = robot_name_from_msg
                self.robot_name_to_participant[robot_name_from_msg] = participant_id
                self.get_logger().info(
                    f'🔗 [ITINERARY SET] Mapped participant_id={participant_id} ↔ '
                    f'robot_name={robot_name_from_msg} (from ItinerarySet.robot_name)'
                )
            else:
                # If robot_name is missing, do not try to guess from positions.
                # Leave this participant unmapped until a robot_name appears.
                self.get_logger().warn(
                    f'⚠️ [ITINERARY SET] robot_name field is empty for participant_id={participant_id}; '
                    f'participant will remain unmapped until a robot_name is provided.'
                )
            
            # Initialize or update robot state
            if participant_id not in self.robot_states:
                # First time seeing this participant - initialize state
                # Set mutex_zones to detected intersections (not empty dict)
                # This ensures that even if the first path has a mutex, it's captured
                # Get robot_name if already mapped
                robot_name = self.participant_to_robot_name.get(participant_id)
                self.robot_states[participant_id] = {
                    'participant_id': participant_id,
                    'itinerary_points': itinerary_points,
                    'mutex_zones': mutex_intersections,  # Use detected intersections, not empty dict
                    'robot_name': robot_name,  # Set if mapping exists, otherwise None
                    'mutex_status': {},
                    'active_mutex': None
                }
                if robot_name:
                    self.get_logger().info(
                        f'🆕 [ITINERARY SET] New participant {participant_id} initialized with robot_name={robot_name}'
                    )
                if mutex_intersections:
                    self.get_logger().info(
                        f'🆕 [ITINERARY SET] New participant {participant_id} initialized with '
                        f'{len(mutex_intersections)} mutex zone(s) detected'
                    )
                else:
                    self.get_logger().info(
                        f'🆕 [ITINERARY SET] New participant {participant_id} initialized. '
                        f'No mutex zones detected in current path ({len(itinerary_points)} waypoints). '
                        f'Robot will be tracked and mutex will be detected when mutex-traversing path is received.'
                    )
            else:
                # Safety: Reset state when new itinerary arrives (new plan)
                # This prevents stale state from previous plans
                self.get_logger().info(
                    f'🔄 [ITINERARY SET] New plan for participant {participant_id}, resetting mutex state'
                )
                
                # Update robot_name if mapping exists
                robot_name = self.participant_to_robot_name.get(participant_id)
                if robot_name:
                    self.robot_states[participant_id]['robot_name'] = robot_name
                else:
                    robot_name = self.robot_states[participant_id].get('robot_name')
                old_mutex_zones = self.robot_states[participant_id].get('mutex_zones', {})
                
                # Check if robot was owner of any mutex that's no longer in the new plan
                for mutex_name in old_mutex_zones.keys():
                    mutex_state = self.mutex_states.get(mutex_name)
                    if mutex_state:
                        owner = mutex_state.get('owner')
                        # Check if this participant was the owner
                        is_owner = False
                        if owner and robot_name and owner == robot_name:
                            is_owner = True
                        elif owner and not robot_name:
                            # robot_name not set yet, but we can check if owner matches any participant
                            # This is a fallback - try to find participant_id from owner
                            for pid, state in self.robot_states.items():
                                if pid == participant_id and state.get('robot_name') == owner:
                                    is_owner = True
                                    robot_name = owner  # Set robot_name for logging
                                    break
                        
                        if is_owner:
                            # Robot was owner but new plan doesn't include this mutex
                            # Release the mutex to prevent deadlock
                            self.get_logger().warn(
                                f'⚠️ [{robot_name or "Unknown"}] New plan received but was owner of mutex {mutex_name}. '
                                f'Releasing mutex to prevent deadlock.'
                            )
                            # Unlock the mutex
                            mutex_state['locked'] = False
                            mutex_state['owner'] = None
                            # Grant to next robot in queue if available
                            queue = mutex_state['queue']
                            if queue:
                                next_robot = queue.popleft()
                                mutex_state['locked'] = True
                                mutex_state['owner'] = next_robot
                                self.get_logger().info(
                                    f'➡️ Mutex {mutex_name} granted to next robot {next_robot} (after {robot_name or "Unknown"} plan change)'
                                )
                                # Update next robot's status to PASSING_MUTEX
                                self._update_robot_mutex_status(next_robot, mutex_name, 'PASSING_MUTEX')
                                # Send resume request when mutex is granted
                                self._send_resume_request(next_robot, mutex_name)
                
                # Reset mutex status flags
                for mutex_name in self.robot_states[participant_id].get('mutex_status', {}):
                    status_info = self.robot_states[participant_id]['mutex_status'][mutex_name]
                    status_info['exit_reached'] = False
                    status_info['exit_cleared'] = False
                    status_info['status'] = 'NORMAL'  # Reset status to NORMAL
                    # Keep status but reset exit flags
                
                # Reset pause/resume tracking for this robot
                if robot_name and robot_name in self.robot_pause_tracking:
                    tracking = self.robot_pause_tracking[robot_name]
                    tracking['pause_sent'] = False
                    tracking['resume_sent'] = False
                    self.get_logger().debug(
                        f'🔄 [ITINERARY SET] Reset pause/resume flags for {robot_name}'
                    )
            
            # Update robot state with new itinerary and mutex intersections
            # Check if mutex is present in the path - if yes, store it. If no, mutex_zones will be empty.
            self.robot_states[participant_id]['itinerary_points'] = itinerary_points
            self.robot_states[participant_id]['mutex_zones'] = mutex_intersections  # Store detected mutex intersections
            self.robot_states[participant_id]['active_mutex'] = None  # Reset active mutex for new itinerary
            
            if mutex_intersections:
                # Try to get robot_name for logging (may not be set yet if fleet_states hasn't arrived)
                robot_name = self.robot_states[participant_id].get('robot_name')
                robot_name_str = robot_name if robot_name else f'participant_{participant_id}'
                self.get_logger().info(
                    f'🔒 [ITINERARY SET] Participant {participant_id} ({robot_name_str}) will pass through '
                    f'{len(mutex_intersections)} mutex zone(s): {list(mutex_intersections.keys())}'
                )
                for mutex_name, intersection in mutex_intersections.items():
                    self.get_logger().info(
                        f'  {mutex_name}: waypoints {intersection["entry_index"]} → {intersection["exit_index"]}'
                    )
            else:
                # No mutex detected in this itinerary - log for visibility
                # Try to get robot_name for logging (may not be set yet if fleet_states hasn't arrived)
                robot_name = self.robot_states[participant_id].get('robot_name')
                robot_name_str = robot_name if robot_name else f'participant_{participant_id}'
                self.get_logger().info(
                    f'📋 [ITINERARY SET] Participant {participant_id} ({robot_name_str}): '
                    f'No mutex zones in current path ({len(itinerary_points)} waypoints). '
                    f'Robot is tracked and will process mutex when mutex-traversing path is received.'
                )
                
        except Exception as e:
            self.get_logger().error(f'❌ Error processing itinerary_set: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _fleet_states_callback(self, msg: FleetState):
        """
        Callback for /fleet_states topic
        
        Tracks robot live positions and computes mutex approach/exit status.
        """
        try:
            fleet_name = msg.name
            self.get_logger().debug(f'📡 [FLEET STATES] Received from fleet: {fleet_name}, {len(msg.robots)} robots')
            
            for robot_state in msg.robots:
                robot_name = robot_state.name
                if not robot_name:
                    self.get_logger().warn('⚠️ [FLEET STATES] Robot with empty name, skipping')
                    continue
                
                # Store fleet_name mapping
                self.robot_name_to_fleet[robot_name] = fleet_name
                self.get_logger().debug(
                    f'🔗 [FLEET STATES] Mapped robot {robot_name} → fleet {fleet_name}'
                )
                
                # Initialize pause tracking if not present
                if robot_name not in self.robot_pause_tracking:
                    self.robot_pause_tracking[robot_name] = {
                        'pause_sent': False,
                        'resume_sent': False,
                        'mode_request_id': 0
                    }
                
                # Extract robot position
                location = robot_state.location
                robot_pose = (float(location.x), float(location.y))
                level_name = location.level_name if location.level_name else ''
                
                # Store robot position for matching (CRITICAL: for participant_id mapping)
                self.robot_positions[robot_name] = robot_pose
                
                self.get_logger().debug(
                    f'🤖 [FLEET STATES] Robot {robot_name}: pose=({robot_pose[0]:.2f}, {robot_pose[1]:.2f}), '
                    f'level={level_name}, task_id={robot_state.task_id}'
                )
                
                # Look up participant_id based on explicit mapping from itinerary_set
                participant_id = self.robot_name_to_participant.get(robot_name)
                
                if participant_id is None:
                    # No explicit mapping yet – rely purely on robot_name for now.
                    # We will map this robot when an itinerary_set with robot_name arrives.
                    self.get_logger().debug(
                        f'📋 [FLEET STATES] Robot {robot_name} not yet mapped to participant_id. '
                        f'Will process mutex state by robot_name only until itinerary_set provides a mapping.'
                    )
                    self._process_robot_mutex_status_by_name(robot_name, robot_pose, robot_state.task_id)
                    continue
                
                # Update robot state with live position
                if participant_id in self.robot_states:
                    state = self.robot_states[participant_id]
                    state['robot_name'] = robot_name
                    state['current_pose'] = robot_pose
                    state['level_name'] = level_name
                    state['task_id'] = robot_state.task_id
                    
                    # Process mutex zones for this robot
                    self._process_robot_mutex_status(participant_id, robot_name, robot_pose)
                    
                    # Step 6: Handle pause/resume commands based on status
                    self._handle_pause_resume_commands(robot_name, state)
                else:
                    self.get_logger().warn(
                        f'⚠️ [FLEET STATES] Robot {robot_name} (participant {participant_id}) '
                        f'not found in robot_states'
                    )
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error processing fleet_states: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _select_active_mutex(
        self,
        mutex_zones: Dict[str, Dict],
        mutex_status: Dict[str, Dict],
        robot_pose: Tuple[float, float]
    ) -> Optional[str]:
        """
        Select the active mutex for a robot (closest not-yet-cleared mutex)
        
        Args:
            mutex_zones: Dictionary of intersection data (from _detect_mutex_intersections)
                        Contains: entry_coord, exit_coord, entry_index, exit_index
            mutex_status: Dictionary of status info for each mutex
            robot_pose: Current robot position
            
        Returns:
            Name of active mutex, or None if all cleared
        """
        active_mutex = None
        min_dist = float('inf')
        
        for mutex_name, intersection in mutex_zones.items():
            # Use entry_coord from intersection (determined by robot's path order)
            entry_coord = intersection['entry_coord']
            
            # Check if this mutex is already cleared
            status_info = mutex_status.get(mutex_name, {})
            if status_info.get('exit_cleared', False):
                continue  # Skip cleared mutexes
            
            # Compute distance to entry
            dist_to_entry = self._euclidean_distance(robot_pose, entry_coord)
            
            if dist_to_entry < min_dist:
                min_dist = dist_to_entry
                active_mutex = mutex_name
        
        return active_mutex
    
    def _process_robot_mutex_status(
        self,
        participant_id: int,
        robot_name: str,
        robot_pose: Tuple[float, float]
    ):
        """
        Process mutex status for a robot based on current position
        
        Args:
            participant_id: Participant ID
            robot_name: Robot name
            robot_pose: Current (x, y) position
        """
        state = self.robot_states[participant_id]
        mutex_zones = state.get('mutex_zones', {})
        
        if not mutex_zones:
            # Robot has no mutex zones on its route - nothing to process
            # Log that robot is being tracked but has no mutex (for visibility)
            self.get_logger().debug(
                f'📋 [{robot_name}] Robot tracked, but no mutex zones in current path. '
                f'Pose=({robot_pose[0]:.2f}, {robot_pose[1]:.2f})'
            )
            return
        
        # Initialize status tracking if not present
        if 'mutex_status' not in state:
            state['mutex_status'] = {}
        
        mutex_status = state['mutex_status']
        
        # Select active mutex (closest not-yet-cleared)
        active_mutex = self._select_active_mutex(mutex_zones, mutex_status, robot_pose)
        state['active_mutex'] = active_mutex
        
        if active_mutex is None:
            # All mutexes cleared, nothing to process
            return
        
        # Process the active mutex
        mutex_name = active_mutex
        intersection = mutex_zones[mutex_name]
        mutex_zone = self.mutex_zones[mutex_name]
        # Entry/exit coordinates are determined from robot's path, stored in intersection
        entry_coord = intersection['entry_coord']
        exit_coord = intersection['exit_coord']
        
        # Initialize status for this mutex if not present
        if mutex_name not in mutex_status:
            # Calculate pre-mutex waypoint (waypoint before entry)
            entry_index = intersection['entry_index']
            pre_mutex_index = max(0, entry_index - 1)  # Waypoint before entry, or 0 if entry is first
            
            # Get pre-mutex coordinates from itinerary
            itinerary_points = state.get('itinerary_points', [])
            if pre_mutex_index < len(itinerary_points):
                pre_mutex_coord = itinerary_points[pre_mutex_index]
            else:
                # Fallback: use entry coordinate if pre-mutex not available
                pre_mutex_coord = entry_coord
                self.get_logger().warn(
                    f'⚠️ [{robot_name}] Pre-mutex waypoint index {pre_mutex_index} out of range, using entry coordinate'
                )
            
            # Check if robot is already within claim_distance when mutex is first detected
            # This handles the case where robot starts very close to the mutex entry
            # IMPORTANT: robot_pose must be available (from fleet_states) for distance check
            initial_status = 'NORMAL'
            if robot_pose is not None:
                dist_to_entry_init = self._euclidean_distance(robot_pose, entry_coord)
                # If robot is already within claim_distance, set status to APPROACHING immediately
                if dist_to_entry_init <= self.claim_distance:
                    initial_status = 'APPROACHING'
                    self.get_logger().info(
                        f'🔄 [{robot_name}] Mutex {mutex_name} detected and robot is already within claim_distance '
                        f'({dist_to_entry_init:.2f}m <= {self.claim_distance}m), setting status to APPROACHING'
                    )
            else:
                # Robot pose not available yet - will be checked when fleet_states arrives
                self.get_logger().debug(
                    f'📋 [{robot_name}] Mutex {mutex_name} detected but robot pose not available yet. '
                    f'Will check distance when fleet_states arrives.'
                )
            
            mutex_status[mutex_name] = {
                'status': initial_status,
                'entry_index': entry_index,
                'exit_index': intersection['exit_index'],
                'pre_mutex_index': pre_mutex_index,
                'pre_mutex_coord': pre_mutex_coord,
                'exit_reached': False,
                'exit_cleared': False
            }
            
            self.get_logger().info(
                f'📍 [{robot_name}] Pre-mutex stop point for {mutex_name}: '
                f'waypoint {pre_mutex_index} at ({pre_mutex_coord[0]:.2f}, {pre_mutex_coord[1]:.2f})'
            )
            
            # If robot is already APPROACHING, trigger claim logic immediately
            if initial_status == 'APPROACHING' and robot_pose is not None:
                # Get mutex state for checking
                if mutex_name not in self.mutex_states:
                    self.mutex_states[mutex_name] = {
                        'locked': False,
                        'owner': None,
                        'queue': deque()
                    }
                mutex_state = self.mutex_states[mutex_name]
                self._handle_mutex_claim(robot_name, mutex_name)
        
        status_info = mutex_status[mutex_name]
        old_status = status_info['status']
        
        # Compute distances
        dist_to_entry = self._euclidean_distance(robot_pose, entry_coord)
        dist_to_exit = self._euclidean_distance(robot_pose, exit_coord)
        
        # Get mutex state for checking
        # Note: Mutex locking is by mutex_name, not coordinates
        # All robots with same mutex_name share the same lock, even if coordinates differ
        if mutex_name not in self.mutex_states:
            self.get_logger().warn(
                f'⚠️ Mutex {mutex_name} not found in mutex_states, creating dynamically'
            )
            self.mutex_states[mutex_name] = {
                'locked': False,
                'owner': None,
                'queue': deque()
            }
        mutex_state = self.mutex_states[mutex_name]
        
        # Status transitions based on distance to entry
        # CRITICAL: This check happens every time fleet_states arrives with robot pose
        # If robot was NORMAL and is now within 4m, transition to APPROACHING and publish
        if old_status == 'NORMAL' and dist_to_entry <= self.claim_distance:
            status_info['status'] = 'APPROACHING'
            mutex_locked = mutex_state.get('locked', False)
            mutex_owner = mutex_state.get('owner', None)
            self.get_logger().info(
                f'🔄 [{robot_name}] Status: NORMAL → APPROACHING mutex {mutex_name} '
                f'(dist_to_entry={dist_to_entry:.2f}m <= {self.claim_distance}m) - '
                f'Mutex locked: {mutex_locked}, Owner: {mutex_owner}'
            )
            # Trigger claim logic (will claim if free, or queue if locked)
            self._handle_mutex_claim(robot_name, mutex_name)
            # Immediately publish status since robot is now within 4m (as per requirement: 4m distance = request)
            self._publish_mutex_group_request_status(robot_name, mutex_name, dist_to_entry, 'APPROACHING', mutex_state)
        
        # Also check if robot is already APPROACHING but hasn't claimed/queued yet
        # (This can happen if robot was APPROACHING, then got a new plan that reset status,
        # or if robot transitioned to APPROACHING but claim logic didn't queue it properly)
        current_status = status_info['status']
        if current_status == 'APPROACHING' and dist_to_entry <= self.claim_distance:
            # Robot is APPROACHING - ensure it has attempted to claim/queue
            if mutex_state['owner'] != robot_name and robot_name not in mutex_state['queue']:
                # Robot is APPROACHING but not owner and not queued - trigger claim logic
                self.get_logger().info(
                    f'🔄 [{robot_name}] APPROACHING but not queued/owner, triggering claim logic (mutex locked by {mutex_state.get("owner", "unknown")})'
                )
                self._handle_mutex_claim(robot_name, mutex_name)
        
        # Check if robot is already APPROACHING/QUEUED and mutex becomes available
        # (e.g., if robot was queued and mutex was just granted)
        if old_status in ['APPROACHING', 'QUEUED_STOPPING', 'STOPPED_WAITING']:
            # If robot is now the owner, transition to PASSING_MUTEX
            if mutex_state['owner'] == robot_name and old_status != 'PASSING_MUTEX':
                status_info['status'] = 'PASSING_MUTEX'
                self.get_logger().info(
                    f'🔄 [{robot_name}] Status: {old_status} → PASSING_MUTEX mutex {mutex_name} '
                    f'(mutex granted)'
                )
                # Send resume request when transitioning to PASSING_MUTEX from waiting state
                if old_status in ['QUEUED_STOPPING', 'STOPPED_WAITING']:
                    self._send_resume_request(robot_name, mutex_name)
        
        # Exit detection and clearing (only for the owner)
        mutex_state = self.mutex_states[mutex_name]
        is_owner = mutex_state.get('owner') == robot_name
        
        if is_owner:
            # Only check exit distances for the owner
            if not status_info['exit_reached']:
                if dist_to_exit <= self.exit_reached_distance:
                    status_info['exit_reached'] = True
                    self.get_logger().info(
                        f'✅ [{robot_name}] Exit reached for mutex {mutex_name} '
                        f'(dist_to_exit={dist_to_exit:.2f}m <= {self.exit_reached_distance}m)'
                    )
            
            if status_info['exit_reached'] and not status_info['exit_cleared']:
                if dist_to_exit >= self.clearance_distance:
                    status_info['exit_cleared'] = True
                    self.get_logger().info(
                        f'✅ [{robot_name}] Exit cleared for mutex {mutex_name} '
                        f'(dist_to_exit={dist_to_exit:.2f}m >= {self.clearance_distance}m)'
                    )
                    # Trigger unlock logic (this robot is the owner)
                    self._handle_mutex_unlock(robot_name, mutex_name)
        
        # Log status on change (compact format)
        current_status = status_info['status']
        if current_status != old_status:
            self.get_logger().info(
                f'📊 [{robot_name}] pose=({robot_pose[0]:.2f}, {robot_pose[1]:.2f}), '
                f'mutex={mutex_name}, status={current_status}, '
                f'dist_to_entry={dist_to_entry:.2f}m, dist_to_exit={dist_to_exit:.2f}m'
            )
        
        # Publish unified mutex group request status
        # Publish when robot is within 4.0m of entry:
        # - Owner's request is published (state=OWNER, queue_position=0)
        # - Queued robots are also published (state=QUEUED, queue_position=1,2,3...) so queue is visible
        # - This allows monitoring of queue status for debugging
        is_owner = mutex_state.get('owner') == robot_name
        is_queued = robot_name in mutex_state.get('queue', deque())
        should_publish = (
            dist_to_entry <= self.claim_distance and (is_owner or is_queued)
        )
        
        if should_publish:
            self._publish_mutex_group_request_status(robot_name, mutex_name, dist_to_entry, current_status, mutex_state)
    
    def _handle_mutex_claim(self, robot_name: str, mutex_name: str):
        """
        Handle mutex claim logic when robot becomes APPROACHING
        
        Mutex locking is based on mutex_name (not coordinates).
        All robots with the same mutex_name share the same lock,
        even if their entry/exit coordinates are different.
        
        Args:
            robot_name: Name of the robot
            mutex_name: Name of the mutex zone (this is the key for locking)
        """
        # Ensure mutex state exists (create if not found, e.g., if mutex_name wasn't in 0.yaml)
        if mutex_name not in self.mutex_states:
            self.get_logger().warn(
                f'⚠️ Mutex {mutex_name} not found in initial parse, creating state dynamically'
            )
            self.mutex_states[mutex_name] = {
                'locked': False,
                'owner': None,
                'queue': deque()
            }
        
        mutex_state = self.mutex_states[mutex_name]
        
        if not mutex_state['locked']:
            # Mutex is free - claim it
            # Note: Locking is by mutex_name, not coordinates
            # All robots with same mutex_name share the same lock
            mutex_state['locked'] = True
            mutex_state['owner'] = robot_name
            self.get_logger().info(
                f'🔒 Mutex {mutex_name} claimed by {robot_name} '
                f'(mutex locked by name, not coordinates)'
            )
            
            # Update robot status to PASSING_MUTEX
            self._update_robot_mutex_status(robot_name, mutex_name, 'PASSING_MUTEX')
            
            # Publish status update (robot is now OWNER)
            # Get robot's current distance to entry for publishing
            # Only publish if within claim_distance (consistent with main publishing logic)
            dist_to_entry = self._get_robot_dist_to_entry(robot_name, mutex_name)
            if dist_to_entry is not None and dist_to_entry <= self.claim_distance:
                self._publish_mutex_group_request_status(
                    robot_name, mutex_name, dist_to_entry, 'PASSING_MUTEX', mutex_state
                )
        else:
            # Mutex is locked by mutex_name (not by coordinates)
            # Even if this robot's entry/exit coordinates are different,
            # it cannot enter because mutex_name matches
            if mutex_state['owner'] == robot_name:
                # Robot is already the owner - no action needed
                self.get_logger().debug(
                    f'[{robot_name}] Already owner of mutex {mutex_name}'
                )
            else:
                # Mutex is owned by another robot - queue this robot
                # This happens even if coordinates are different, because mutex_name matches
                queue = mutex_state['queue']
                owner = mutex_state['owner']
                
                # Check if robot is already queued
                if robot_name not in queue:
                    queue.append(robot_name)
                    position = len(queue)
                    self.get_logger().info(
                        f'⏳ {robot_name} queued for mutex {mutex_name}, position={position} '
                        f'(owner: {owner}) - Mutex locked by name, not coordinates'
                    )
                    
                    # Get pre-mutex coordinates for logging
                    pre_mutex_coord = None
                    for participant_id, state in self.robot_states.items():
                        if state.get('robot_name') == robot_name:
                            mutex_status = state.get('mutex_status', {})
                            if mutex_name in mutex_status:
                                status_info = mutex_status[mutex_name]
                                pre_mutex_coord = status_info.get('pre_mutex_coord', None)
                                pre_mutex_index = status_info.get('pre_mutex_index', 0)
                                if pre_mutex_coord:
                                    self.get_logger().info(
                                        f'📍 [{robot_name}] Will stop at pre-mutex waypoint {pre_mutex_index} '
                                        f'({pre_mutex_coord[0]:.2f}, {pre_mutex_coord[1]:.2f}) before mutex {mutex_name}'
                                    )
                            break
                    
                    # Update robot status to QUEUED_STOPPING
                    self._update_robot_mutex_status(robot_name, mutex_name, 'QUEUED_STOPPING')
                    # Send deceleration pause request immediately when queued (type=1)
                    self.get_logger().info(
                        f'📤 [{robot_name}] Calling _send_pause_request for mutex {mutex_name}'
                    )
                    self._send_pause_request(robot_name, mutex_name, immediate=False)
                    # Update status to STOPPED_WAITING after pause is sent
                    self._update_robot_mutex_status(robot_name, mutex_name, 'STOPPED_WAITING')
                    
                    # Do NOT publish queued robot's request (as per requirement)
                    # Only owner's request is published
                    # Queued robots will be published when they become owner
                else:
                    self.get_logger().debug(
                        f'[{robot_name}] Already in queue for mutex {mutex_name}'
                    )
    
    def _handle_mutex_unlock(self, robot_name: str, mutex_name: str):
        """
        Handle mutex unlock logic when owner robot clears exit
        
        Args:
            robot_name: Name of the robot (should be the owner)
            mutex_name: Name of the mutex zone
        """
        mutex_state = self.mutex_states[mutex_name]
        
        # Verify this robot is the owner
        if mutex_state['owner'] != robot_name:
            self.get_logger().warn(
                f'⚠️ [{robot_name}] Attempted to unlock mutex {mutex_name} but is not the owner '
                f'(owner is {mutex_state["owner"]})'
            )
            return
        
        # Unlock the mutex
        mutex_state['locked'] = False
        mutex_state['owner'] = None
        self.get_logger().info(f'🔓 Mutex {mutex_name} released by {robot_name}')
        
        # Do NOT publish when robot releases mutex (no longer owner)
        # Only owner's requests are published (as per requirement)
        
        # Grant to next robot in queue if available
        queue = mutex_state['queue']
        if queue:
            next_robot = queue.popleft()
            mutex_state['locked'] = True
            mutex_state['owner'] = next_robot
            self.get_logger().info(
                f'➡️ Mutex {mutex_name} granted to next robot {next_robot}'
            )
            
            # Update next robot's status to PASSING_MUTEX
            self._update_robot_mutex_status(next_robot, mutex_name, 'PASSING_MUTEX')
            # Send resume request when mutex is granted
            self._send_resume_request(next_robot, mutex_name)
            
            # Publish status update (next robot is now OWNER)
            # CRITICAL: Now that next robot is owner, publish its request (as per requirement)
            dist_to_entry = self._get_robot_dist_to_entry(next_robot, mutex_name)
            if dist_to_entry is not None and dist_to_entry <= self.claim_distance:
                self._publish_mutex_group_request_status(
                    next_robot, mutex_name, dist_to_entry, 'PASSING_MUTEX', mutex_state
                )
    
    def _update_robot_mutex_status(
        self,
        robot_name: str,
        mutex_name: str,
        new_status: str
    ):
        """
        Update robot's mutex status
        
        Args:
            robot_name: Name of the robot
            mutex_name: Name of the mutex zone
            new_status: New status (PASSING_MUTEX, QUEUED_STOPPING, etc.)
        """
        # Find robot state by robot_name
        for participant_id, state in self.robot_states.items():
            if state.get('robot_name') == robot_name:
                mutex_status = state.get('mutex_status', {})
                if mutex_name in mutex_status:
                    old_status = mutex_status[mutex_name]['status']
                    mutex_status[mutex_name]['status'] = new_status
                    
                    if old_status != new_status:
                        self.get_logger().info(
                            f'🔄 [{robot_name}] Mutex {mutex_name} status: '
                            f'{old_status} → {new_status}'
                        )
                break
    
    def _send_pause_request(self, robot_name: str, mutex_name: str, immediate: bool = False):
        """
        Send pause request for a robot.
        
        If immediate is False (default), uses TYPE_PAUSE_AT_CHECKPOINT with the
        pre-mutex waypoint index so the controller can decelerate toward that
        point. If immediate is True, uses TYPE_PAUSE_IMMEDIATELY (type=0) so
        the controller can perform a hard stop at the current pose.
        
        Args:
            robot_name: Name of the robot
            mutex_name: Name of the mutex zone (for logging)
            immediate: Whether to send an immediate-stop pause (type=0)
        """
        if robot_name not in self.robot_pause_tracking:
            self.robot_pause_tracking[robot_name] = {
                'pause_sent': False,
                'resume_sent': False,
                'mode_request_id': 0
            }
        
        tracking = self.robot_pause_tracking[robot_name]
        
        # Anti-spam: Check cooldown period (allows periodic re-publishing)
        current_time = time.time()
        last_cmd_time = tracking.get('last_cmd_time', 0.0)
        time_since_last = current_time - last_cmd_time
        if time_since_last < self.command_cooldown:
            self.get_logger().debug(
                f'[{robot_name}] Command cooldown active ({time_since_last:.2f}s < {self.command_cooldown}s), skipping pause request'
            )
            return
        
        # Get fleet_name
        fleet_name = self.robot_name_to_fleet.get(robot_name, '')
        if not fleet_name:
            self.get_logger().error(
                f'❌ [{robot_name}] Fleet name not available, cannot send pause request. '
                f'Available fleet mappings: {dict(self.robot_name_to_fleet)}'
            )
            return
        
        # Get pre-mutex coordinates from robot state
        pre_mutex_index = 0
        pre_mutex_coord = None
        for participant_id, state in self.robot_states.items():
            if state.get('robot_name') == robot_name:
                mutex_status = state.get('mutex_status', {})
                if mutex_name in mutex_status:
                    status_info = mutex_status[mutex_name]
                    pre_mutex_index = status_info.get('pre_mutex_index', 0)
                    pre_mutex_coord = status_info.get('pre_mutex_coord', None)
                break
        
        # Increment mode_request_id for new command
        tracking['mode_request_id'] += 1
        
        # Create pause request
        pause_msg = PauseRequest()
        pause_msg.fleet_name = fleet_name
        pause_msg.robot_name = robot_name
        pause_msg.mode_request_id = tracking['mode_request_id']
        if immediate:
            # Immediate / hard stop at current robot pose
            pause_msg.type = PauseRequest.TYPE_PAUSE_IMMEDIATELY
            pause_msg.at_checkpoint = 0
        else:
            # Decelerate toward a specific pre-mutex waypoint
            pause_msg.type = PauseRequest.TYPE_PAUSE_AT_CHECKPOINT
            pause_msg.at_checkpoint = pre_mutex_index  # Waypoint index where robot should stop
        
        # Publish
        self.pause_request_pub.publish(pause_msg)
        tracking['pause_sent'] = True
        tracking['resume_sent'] = False  # Reset resume flag
        tracking['last_cmd_time'] = time.time()  # Update command timestamp
        
        # Log with coordinates
        coord_str = f"({pre_mutex_coord[0]:.2f}, {pre_mutex_coord[1]:.2f})" if pre_mutex_coord else "unknown"
        self.get_logger().info(
            f'⛔ Pause requested for robot {robot_name} (mutex {mutex_name}, queued) - '
            f'Stop at waypoint {pre_mutex_index} {coord_str}'
        )
        self.get_logger().info(
            f'📤 Published PauseRequest: fleet_name={fleet_name}, robot_name={robot_name}, '
            f'mode_request_id={pause_msg.mode_request_id}, type={pause_msg.type}, '
            f'at_checkpoint={pause_msg.at_checkpoint}'
        )
    
    def _send_resume_request(self, robot_name: str, mutex_name: str):
        """
        Send resume request for a robot
        
        Args:
            robot_name: Name of the robot
            mutex_name: Name of the mutex zone (for logging)
        """
        if robot_name not in self.robot_pause_tracking:
            self.robot_pause_tracking[robot_name] = {
                'pause_sent': False,
                'resume_sent': False,
                'mode_request_id': 0
            }
        
        tracking = self.robot_pause_tracking[robot_name]
        
        # Anti-spam: Check cooldown period
        current_time = time.time()
        last_cmd_time = tracking.get('last_cmd_time', 0.0)
        if current_time - last_cmd_time < self.command_cooldown:
            self.get_logger().debug(
                f'[{robot_name}] Command cooldown active, skipping resume request'
            )
            return
        
        # Debounce: only send if not already sent
        if tracking['resume_sent']:
            self.get_logger().debug(
                f'[{robot_name}] Resume already sent, skipping duplicate'
            )
            return
        
        # Get fleet_name
        fleet_name = self.robot_name_to_fleet.get(robot_name, '')
        if not fleet_name:
            self.get_logger().warn(
                f'⚠️ [{robot_name}] Fleet name not available, cannot send resume request'
            )
            return
        
        # Increment mode_request_id for new command
        tracking['mode_request_id'] += 1
        
        # Create resume request
        resume_msg = PauseRequest()
        resume_msg.fleet_name = fleet_name
        resume_msg.robot_name = robot_name
        resume_msg.mode_request_id = tracking['mode_request_id']
        resume_msg.type = PauseRequest.TYPE_RESUME
        resume_msg.at_checkpoint = 0
        
        # Publish
        self.pause_request_pub.publish(resume_msg)
        tracking['resume_sent'] = True
        tracking['pause_sent'] = False  # Reset pause flag
        tracking['last_cmd_time'] = time.time()  # Update command timestamp
        
        self.get_logger().info(
            f'▶️ Resume requested for robot {robot_name} (mutex {mutex_name} granted)'
        )
    
    def _handle_pause_resume_commands(self, robot_name: str, state: Dict):
        """
        Handle pause/resume commands based on robot status transitions
        
        Args:
            robot_name: Name of the robot
            state: Robot state dictionary
        """
        mutex_status = state.get('mutex_status', {})
        active_mutex = state.get('active_mutex')
        
        if active_mutex is None:
            return
        
        status_info = mutex_status.get(active_mutex, {})
        current_status = status_info.get('status', 'NORMAL')
        robot_pose = state.get('current_pose')
        
        if robot_pose is None:
            return
        
        # Do not send pause commands for the current owner of the mutex.
        mutex_state = self.mutex_states.get(active_mutex)
        if mutex_state and mutex_state.get('owner') == robot_name:
            return
        
        # While a robot is queued for a mutex, periodically send pause commands
        # so downstream controllers can decelerate / stop appropriately.
        if current_status in ['QUEUED_STOPPING', 'STOPPED_WAITING']:
            # Decide whether to use immediate stop (type=0) or checkpoint stop (type=1)
            pre_mutex_coord = status_info.get('pre_mutex_coord')
            immediate = False
            if pre_mutex_coord is not None:
                # Distance from current pose to pre-mutex waypoint
                dist_to_pre_mutex = self._euclidean_distance(robot_pose, pre_mutex_coord)
                # If robot is within 0.5m of pre-mutex waypoint and still queued, request immediate stop
                immediate = dist_to_pre_mutex <= 0.5
            self._send_pause_request(robot_name, active_mutex, immediate=immediate)
    
    def _process_robot_mutex_status_by_name(
        self,
        robot_name: str,
        robot_pose: Tuple[float, float],
        task_id: str
    ):
        """
        Process mutex status for a robot when participant_id mapping is unavailable
        Uses robot_name as primary key (safety fallback)
        
        Args:
            robot_name: Name of the robot
            robot_pose: Current (x, y) position
            task_id: Current task ID
        """
        # Find robot state by robot_name (fallback when participant_id mapping fails)
        # This is a safety mechanism to avoid losing robot state
        for participant_id, state in self.robot_states.items():
            if state.get('robot_name') == robot_name:
                self._process_robot_mutex_status(participant_id, robot_name, robot_pose)
                return
        
        # If no state found, it means the robot hasn't received an itinerary_set yet
        # This is normal for robots that just started - they'll get an itinerary_set soon
        # Log at debug level to avoid spam, but make it clear this is expected
        self.get_logger().debug(
            f'📋 [FLEET STATES] Robot {robot_name} not yet in robot_states. '
            f'Waiting for itinerary_set message to establish participant_id mapping. '
            f'This is normal for newly started robots.'
        )
    
    def _publish_mutex_group_request_status(
        self,
        robot_name: str,
        mutex_name: str,
        dist_to_entry: float,
        state: str,
        mutex_state: Dict
    ):
        """
        Publish unified mutex group request status to /mutex_group_request
        
        This provides a unified stream showing all robots' mutex status:
        - robot_name: Name of the robot
        - mutex_name: Name of the mutex group
        - dist_to_entry: Distance to mutex entry (meters)
        - timestamp: Current time
        - state: REQUESTING, QUEUED, OWNER, or NORMAL
        - queue_position: Position in queue (0 if not queued)
        
        Args:
            robot_name: Name of the robot
            mutex_name: Name of the mutex group
            dist_to_entry: Distance to mutex entry point (meters)
            state: Current robot status (NORMAL, APPROACHING, QUEUED_STOPPING, etc.)
            mutex_state: Mutex state dictionary (contains owner, queue info)
        """
        # Map internal status to unified state
        unified_state = 'NORMAL'
        queue_position = 0
        
        if state == 'APPROACHING':
            # Robot is requesting/approaching mutex
            if mutex_state.get('owner') == robot_name:
                unified_state = 'OWNER'
            elif mutex_state.get('locked', False):
                unified_state = 'QUEUED'
                # Find queue position
                queue = mutex_state.get('queue', deque())
                try:
                    queue_position = list(queue).index(robot_name) + 1  # 1-indexed
                except ValueError:
                    queue_position = 0
            else:
                unified_state = 'REQUESTING'
        elif state in ['QUEUED_STOPPING', 'STOPPED_WAITING']:
            unified_state = 'QUEUED'
            queue = mutex_state.get('queue', deque())
            try:
                queue_position = list(queue).index(robot_name) + 1  # 1-indexed
            except ValueError:
                queue_position = 0
        elif state == 'PASSING_MUTEX':
            # CRITICAL: Only set OWNER if robot is actually the owner in mutex_state
            # A robot can have PASSING_MUTEX status but not be owner if mutex was released/granted
            if mutex_state.get('owner') == robot_name:
                unified_state = 'OWNER'
            else:
                # Robot has PASSING_MUTEX status but is not owner - this shouldn't happen, but handle gracefully
                # Set to REQUESTING or QUEUED based on mutex lock status
                if mutex_state.get('locked', False):
                    unified_state = 'QUEUED'
                    queue = mutex_state.get('queue', deque())
                    try:
                        queue_position = list(queue).index(robot_name) + 1
                    except ValueError:
                        queue_position = 0
                else:
                    unified_state = 'REQUESTING'
        elif dist_to_entry <= self.claim_distance:
            # Within claim distance but status not yet updated
            if mutex_state.get('owner') == robot_name:
                unified_state = 'OWNER'
            elif mutex_state.get('locked', False):
                unified_state = 'QUEUED'
                queue = mutex_state.get('queue', deque())
                try:
                    queue_position = list(queue).index(robot_name) + 1
                except ValueError:
                    queue_position = 0
            else:
                unified_state = 'REQUESTING'
        
        # Rate limiting: Only publish at most once per second per (robot, mutex) pair
        publish_key = (robot_name, mutex_name)
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        
        last_publish = self.last_publish_time.get(publish_key, 0.0)
        time_since_last_publish = current_time - last_publish
        
        if time_since_last_publish < self.publish_interval:
            # Too soon since last publish - skip this publish
            return
        
        # Update last publish time
        self.last_publish_time[publish_key] = current_time
        
        # Get current timestamp
        now = self.get_clock().now()
        timestamp_msg = now.to_msg()
        
        # Create proper ROS2 message (like FleetState format)
        # This displays properly in ros2 topic echo without truncation - each field shows separately
        msg = MutexGroupRequestStatus()
        msg.robot_name = robot_name
        msg.mutex_name = mutex_name
        msg.dist_to_entry = round(dist_to_entry, 3)
        msg.timestamp = timestamp_msg
        msg.state = unified_state
        msg.queue_position = queue_position
        
        # Publish structured message (displays properly in ros2 topic echo, just like FleetState)
        self.mutex_group_request_pub.publish(msg)
        
        self.get_logger().debug(
            f'📡 Published mutex_group_request: {robot_name} → {mutex_name}, '
            f'state={unified_state}, dist={dist_to_entry:.2f}m, queue_pos={queue_position}'
        )
    
    def _get_robot_dist_to_entry(self, robot_name: str, mutex_name: str) -> Optional[float]:
        """
        Get robot's current distance to mutex entry point
        
        Args:
            robot_name: Name of the robot
            mutex_name: Name of the mutex group
            
        Returns:
            Distance to entry (meters), or None if not available
        """
        # Find robot state by robot_name
        for participant_id, state in self.robot_states.items():
            if state.get('robot_name') == robot_name:
                robot_pose = state.get('current_pose')
                mutex_zones = state.get('mutex_zones', {})
                
                if robot_pose and mutex_name in mutex_zones:
                    intersection = mutex_zones[mutex_name]
                    entry_coord = intersection.get('entry_coord')
                    if entry_coord:
                        return self._euclidean_distance(robot_pose, entry_coord)
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MutexSupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

