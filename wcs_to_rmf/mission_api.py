# =============================================================================
# RMF Mission API - FastAPI Server for WCS Integration
# =============================================================================
# This module provides a REST API that bridges WCS (Warehouse Control System)
# with RMF (Robot Management Framework) for creating and monitoring patrol missions.
# 
# Key Features:
# - POST /api/MissionCreate: Create new missions from WCS
# - GET /api/GetMissions: Get real-time status of all missions
# - Real-time mission tracking via ROS 2 topics
# - Automatic robot assignment and status updates
# - API key authentication for security
# - Professional robot assignment synchronization with RMF
# =============================================================================

from fastapi import FastAPI, Request, HTTPException, Header, Query
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import json
import logging
import asyncio
import uuid
import os
from datetime import datetime
from collections import defaultdict
import httpx

# ROS 2 imports for RMF integration (used for monitoring fleet states)
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState

# Configure logging for debugging and monitoring
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Fixed handlers for delivery operations
# These are predefined handlers that RMF uses for delivery operations
DELIVERY_CATEGORY = "delivery"    # Category for delivery tasks
DEFAULT_PICKUP_DISPENSER = "coke_dispenser"    # Default pickup dispenser
DEFAULT_DROPOFF_INGESTOR = "coke_ingestor"     # Default dropoff ingestor

# =============================================================================
# API AUTHENTICATION CONFIGURATION
# =============================================================================

# API key for WCS authentication
# In production, set this via environment variable: export WCS_API_KEY="123456789"
WCS_API_KEY = os.getenv("WCS_API_KEY", "123456789")
API_KEY_HEADER = "Apikey"

# RMF API Server configuration
# In production, set these via environment variables:
# export RMF_API_SERVER_URL="http://localhost:8000"
# export RMF_API_JWT_TOKEN="your_jwt_token_here"
RMF_API_SERVER_URL = os.getenv("RMF_API_SERVER_URL", "http://localhost:8000")
# Hardcoded token (never expires - expires Dec 31, 2099)
# Environment variable takes precedence if set
DEFAULT_JWT_TOKEN = "eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJwcmVmZXJyZWRfdXNlcm5hbWUiOiJhZG1pbiIsImF1ZCI6InJtZl9hcGlfc2VydmVyIiwiaXNzIjoic3R1YiIsImlhdCI6MTc2OTU2MDU3MCwiZXhwIjo0MTAyNDI0OTk5fQ.-0o6p7gAHV4_3eBDdN7h2LxUU7zC8-mNTuYzZyNESpg"
RMF_API_JWT_TOKEN = os.getenv("RMF_API_JWT_TOKEN", DEFAULT_JWT_TOKEN)

# Initialize FastAPI application
app = FastAPI(title="RMF Mission API", version="1.0.0")

@app.on_event("startup")
async def startup_event():
    """Initialize the mission API when FastAPI starts"""
    logger.info("🚀 Starting RMF Mission API...")
    
    # Start background ROS 2 monitoring
    await start_background_monitoring()
    logger.info("✅ Background ROS 2 monitoring started")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup when FastAPI shuts down"""
    logger.info("🛑 Shutting down RMF Mission API...")
    
    # Stop background monitoring
    global background_task
    if background_task:
        background_task.cancel()
        try:
            await background_task
        except asyncio.CancelledError:
            pass
        logger.info("✅ Background ROS 2 monitoring stopped")
    
    # Shutdown ROS 2
    try:
        rclpy.shutdown()
        logger.info("✅ ROS 2 shutdown completed")
    except Exception as e:
        logger.warning(f"ROS 2 shutdown warning: {e}")

# =============================================================================
# GLOBAL DATA STORAGE
# =============================================================================

# In-memory database to store mission information
# In production, this should be replaced with a proper database
missions_db = {}  # Store mission data with mission_id as key
mission_counter = 100  # Start mission IDs from 100 to avoid conflicts

# ROS 2 context management
ros_context_initialized = False

# =============================================================================
# SIMPLIFIED DIRECT MAPPING: Numeric ID → Conveyor Name
# =============================================================================

# Direct mapping from MES numeric IDs to RMF Conveyor waypoint names
# MES sends numeric IDs (1, 2, 3) which map directly to Conveyor waypoints
ID_TO_CONVEYOR_MAP = {
    1: "Conveyor1",
    2: "Conveyor2",
    3: "Conveyor3",
    # Add more IDs as needed if they map to conveyors
    # Example: 4: "Conveyor1", 5: "Conveyor2", etc.
}

# Reverse mapping for responses (Conveyor → ID for WCS compatibility)
CONVEYOR_TO_ID_MAP = {v: k for k, v in ID_TO_CONVEYOR_MAP.items()}

def get_rmf_waypoint_from_id(target_id: int) -> str:
    """
    Convert MES numeric ID directly to RMF Conveyor waypoint name
    
    Args:
        target_id: Numeric ID from MES (e.g., 1, 2, 3)
        
    Returns:
        RMF waypoint name (e.g., "Conveyor1", "Conveyor2", "Conveyor3")
    """
    return ID_TO_CONVEYOR_MAP.get(target_id, f"Conveyor{target_id}")

def get_id_from_conveyor(conveyor_name: str) -> int:
    """
    Convert RMF Conveyor name back to numeric ID for WCS responses
    
    Args:
        conveyor_name: RMF waypoint name (e.g., "Conveyor1", "Conveyor2")
        
    Returns:
        Numeric ID for WCS (e.g., 1, 2, 3) or -1 if not found
    """
    return CONVEYOR_TO_ID_MAP.get(conveyor_name, -1)

def create_rmf_priority(mes_priority: Optional[int] = None) -> Optional[Dict[str, Any]]:
    """
    Convert MES priority integer to RMF binary priority format
    
    RMF uses binary priority format: {"type": "binary", "value": 1} for high priority,
    {"type": "binary", "value": 0} for low priority.
    
    Args:
        mes_priority: MES priority level (integer). If > 0, creates high priority, else low priority.
                     If None, returns None (no priority set).
        
    Returns:
        RMF priority dict in binary format, or None if mes_priority is None
    """
    if mes_priority is None:
        return None
    
    # Convert MES priority to RMF binary format
    # If priority > 0, it's high priority (value = 1), else low priority (value = 0)
    priority_value = 1 if mes_priority > 0 else 0
    
    return {
        "type": "binary",
        "value": priority_value
    }

def ensure_different_conveyors(pickup_waypoint: str, dropoff_waypoint: str) -> tuple[str, str]:
    """
    Ensure pickup and dropoff are on different conveyors.
    
    Since each conveyor can act as both pickup and dropoff, but a task cannot
    pick and drop on the same conveyor, this function ensures they map to different conveyors.
    
    Args:
        pickup_waypoint: RMF waypoint name for pickup (e.g., "Conveyor1")
        dropoff_waypoint: RMF waypoint name for dropoff (e.g., "Conveyor2")
        
    Returns:
        Tuple of (pickup_conveyor, dropoff_conveyor) ensuring they are different
    """
    pickup_conveyor = pickup_waypoint
    dropoff_conveyor = dropoff_waypoint
    
    # If they map to the same conveyor, we need to adjust
    if pickup_conveyor == dropoff_conveyor:
        logger.warning(
            f"⚠️ Pickup ({pickup_waypoint}) and dropoff ({dropoff_waypoint}) "
            f"map to the same conveyor ({pickup_conveyor}). This is not allowed."
        )
        
        # Strategy: Keep pickup as-is, rotate dropoff to next conveyor
        conveyors = ["Conveyor1", "Conveyor2", "Conveyor3"]
        current_index = conveyors.index(pickup_conveyor) if pickup_conveyor in conveyors else 0
        next_index = (current_index + 1) % 3
        dropoff_conveyor = conveyors[next_index]
        
        logger.warning(
            f"🔄 Cannot use same conveyor for pickup and dropoff. "
            f"Would need to adjust dropoff to {dropoff_conveyor}, but this requires WCS to send different places."
        )
    
    return pickup_conveyor, dropoff_conveyor

# =============================================================================
# PYDANTIC MODELS FOR REQUEST/RESPONSE VALIDATION
# =============================================================================

class LoadOptions(BaseModel):
    """Model for load requirements in mission steps"""
    RequiredLoadStatus: str    # e.g., "LoadAtLocation", "LocationHasRoom"
    RequiredLoadType: int      # Load type identifier

class StepOptions(BaseModel):
    """Model for step configuration options"""
    Load: LoadOptions
    SortingRules: Optional[List[str]] = None  # e.g., ["Priority", "Closest"]

class Target(BaseModel):
    """Model for target locations in mission steps"""
    Id: str  # Place name like "R1", "R7" (changed from int to str for direct RMF integration)

class Step(BaseModel):
    """Model for individual mission steps (Pickup/Dropoff for WCS compatibility)"""
    StepType: str              # "Pickup" or "Dropoff" for WCS compatibility
    Options: StepOptions       # Step configuration
    AllowedTargets: List[Target]  # Valid target locations
    AllowedWaits: Optional[List[Any]] = None

class MissionOptions(BaseModel):
    """Model for mission-level configuration"""
    Priority: int              # Mission priority level
    Rounds: Optional[int] = 1  # Number of patrol rounds

class MissionRequest(BaseModel):
    """Model for incoming mission creation requests from WCS"""
    ExternalId: str            # WCS-provided mission identifier
    Name: str                  # Mission name/description
    Options: MissionOptions    # Mission configuration
    Steps: List[Step]         # List of patrol steps

class MissionResponse(BaseModel):
    """Model for mission creation response to WCS"""
    ExternalId: str            # Original WCS mission ID
    InternalId: int            # Internal mission ID (our system)
    Success: bool              # Whether mission creation succeeded
    Description: str           # Success/error description
    ReplyResult: str           # Result code for WCS processing

# =============================================================================
# GET MISSIONS RESPONSE MODELS
# =============================================================================

class StepData(BaseModel):
    """Model for step information in GetMissions response"""
    StepType: str              # "Pickup" or "Dropoff" for WCS compatibility
    StepStatus: str            # "Generated", "DrivingToPickup", "DrivingToDropoff", "Complete"
    CurrentTarget: str         # Human-readable target description
    CurrentTargetId: int       # Numerical target ID for WCS
    WaitTarget: str            # Target to wait at (if any)
    TargetShelfId: int         # Shelf ID (-1 if not applicable)
    LoadType: str              # Type of load being transported
    LoadTypeId: int            # Load type identifier
    ReservingLocation: bool    # Whether location is being reserved

class MissionData(BaseModel):
    """Model for mission information in GetMissions response"""
    Id: int                    # Internal mission ID
    MissionType: str           # Always "Mission"
    ExternalId: str            # WCS mission ID
    Name: str                  # Mission name
    State: str                 # "Generated", "Executing", "Complete", "Failed", "Cancelled"
    AssignedMachine: str       # Robot name (e.g., "FourD_robot_3")
    AssignedMachineId: int     # Robot ID (1, 2, 3, etc.)
    CurrentStepIndex: int      # Current step being executed
    FinalTarget: str           # Human-readable final destination
    FinalTargetId: int         # Numerical ID of final destination
    Steps: List[StepData]      # List of mission steps with status

# =============================================================================
# ROS 2 NODE CLASSES FOR RMF INTEGRATION
# =============================================================================

class MissionTracker(Node):
    """
    Mission Tracker Node - Monitors fleet states and manages mission lifecycle
    
    This is a singleton node that continuously monitors RMF fleet states
    and updates mission status in real-time. It follows ROS 2 best practices
    for persistent node management and ensures perfect synchronization with RMF.
    """
    
    def __init__(self):
        """Initialize the mission tracker node"""
        super().__init__('mission_tracker')
        
        # Subscribe to fleet states for robot-level status updates
        self.fleet_state_sub = self.create_subscription(
            FleetState, 'fleet_states', self.fleet_state_callback, 10
        )
        
        # Store fleet state data for status tracking
        self.fleet_states = {}     # Fleet name -> fleet info
        self.previous_task_ids = {}  # Robot name -> previous task_id for completion detection
        
        # Timer will be created later when the node is properly running
        self.update_timer = None
        
        logger.info("Mission tracker initialized")
    
    def start_timer(self):
        """Start the update timer - called when node is properly running"""
        if self.update_timer is None:
            try:
                self.update_timer = self.create_timer(1.0, self.update_mission_status)
                logger.info("✅ Mission tracker timer started")
            except Exception as e:
                logger.error(f"Failed to create timer: {e}")
                raise
    
    def fleet_state_callback(self, msg: FleetState):
        """
        Handle fleet state updates from RMF
        
        Args:
            msg: Fleet state message from RMF
        """
        try:
            fleet_name = msg.name
            
            # Store fleet state data
            self.fleet_states[fleet_name] = {
                'robots': msg.robots,
                'timestamp': datetime.now()
            }
            
            # Update previous task IDs for completion detection
            for robot in msg.robots:
                robot_name = robot.name
                current_task_id = robot.task_id
                
                # If this is the first time we've seen this robot, initialize previous_task_id
                if robot_name not in self.previous_task_ids:
                    self.previous_task_ids[robot_name] = ''
                
                # Store current task_id for next comparison
                self.previous_task_ids[robot_name] = current_task_id
            
            logger.debug(f"Fleet {fleet_name} state updated with {len(msg.robots)} robots")
            
        except Exception as e:
            logger.error(f"Error processing fleet state: {e}")
    
    def update_mission_status(self):
        """
        Periodic mission status update - called every 1 second
        
        This function provides real-time mission monitoring and ensures
        synchronization between our mission data and RMF's actual state.
        """
        try:
            if not self.fleet_states:
                return  # No fleet data yet
            
            # Get all active RMF task IDs from fleet states
            active_rmf_tasks = self._get_active_rmf_tasks()
            robot_task_mapping = self._get_robot_task_mapping()
            
            logger.debug(f"🔄 Active RMF tasks: {len(active_rmf_tasks)}")
            logger.debug(f"🤖 Robot task mapping: {robot_task_mapping}")
            
            # Update mission states based on active tasks
            self._update_mission_states(active_rmf_tasks)
            
            # Assign available robots to unassigned missions
            self._assign_available_robots()
            
        except Exception as e:
            logger.error(f"🚨 Error in mission status update: {e}")
    
    def _get_active_rmf_tasks(self) -> set:
        """Get set of currently active RMF task IDs"""
        active_rmf_tasks = set()
        
        for fleet_name, fleet_data in self.fleet_states.items():
            for robot in fleet_data['robots']:
                if robot.task_id and robot.task_id != '':
                    active_rmf_tasks.add(robot.task_id)
        
        return active_rmf_tasks
    
    def _get_robot_task_mapping(self) -> dict:
        """Get mapping of robot names to their current task IDs"""
        robot_task_mapping = {}
        
        for fleet_name, fleet_data in self.fleet_states.items():
            for robot in fleet_data['robots']:
                if robot.task_id and robot.task_id != '':
                    robot_task_mapping[robot.name] = robot.task_id
        
        return robot_task_mapping
    
    def _update_mission_states(self, active_rmf_tasks: set):
        """Update mission states based on active RMF tasks"""
        for mission_id, mission_data in missions_db.items():
            mission_rmf_task_id = mission_data.get('rmf_task_id')
            current_state = mission_data.get('state')
            
            if mission_rmf_task_id:
                if mission_rmf_task_id in active_rmf_tasks:
                    # Task is active - sync with RMF
                    logger.debug(f"✅ Mission {mission_id} has active RMF task {mission_rmf_task_id}")
                    self._update_active_mission(mission_id, mission_data, mission_rmf_task_id)
                else:
                    # Task is not active - check completion
                    self._check_mission_completion(mission_id, mission_data)
            else:
                logger.warning(f"Mission {mission_id} has no RMF task ID")
    
    def _check_mission_completion(self, mission_id: int, mission_data: dict):
        """
        Check if a mission should be marked as complete or canceled
        This checks the actual RMF task status via API to detect cancellations
        """
        mission_rmf_task_id = mission_data.get('rmf_task_id')
        if not mission_rmf_task_id:
            return
        
        # Check RMF task status via API to detect cancellations
        try:
            task_status = self._get_rmf_task_status(mission_rmf_task_id)
            if task_status:
                status = task_status.get('status', 'unknown')
                
                if status in ['canceled', 'killed']:
                    logger.info(f"🚫 Mission {mission_id} was canceled in RMF (status: {status})")
                    self._mark_mission_canceled(mission_id, mission_data, status)
                    return
                elif status == 'completed':
                    logger.info(f"✅ Mission {mission_id} completed in RMF")
                    self._mark_mission_complete(mission_id, mission_data)
                    return
                elif status == 'failed':
                    logger.warning(f"❌ Mission {mission_id} failed in RMF")
                    self._mark_mission_failed(mission_id, mission_data)
                    return
        except Exception as e:
            logger.debug(f"Could not check RMF task status for mission {mission_id}: {e}")
        
        # Fallback: Use time-based completion check if API check fails
        mission_created = mission_data.get('created_at')
        if mission_created:
            time_since_creation = (datetime.now() - mission_created).total_seconds()
            
            if time_since_creation < 30:  # Less than 30 seconds old
                logger.debug(f"⏳ Mission {mission_id} is recent ({time_since_creation:.1f}s old) "
                            f"- RMF task might still be processing")
            else:
                logger.info(f"⏰ Mission {mission_id} task completed - marking as complete")
                self._mark_mission_complete(mission_id, mission_data)
    
    def _get_rmf_task_status(self, rmf_task_id: str) -> Optional[dict]:
        """
        Get RMF task status from API server
        
        This uses synchronous httpx since it's called from a ROS 2 timer callback
        which runs in a synchronous context.
        
        Args:
            rmf_task_id: RMF task identifier
            
        Returns:
            Task state dict if successful, None otherwise
        """
        if not RMF_API_SERVER_URL:
            logger.debug(f"No RMF_API_SERVER_URL configured, cannot check task status for {rmf_task_id}")
            return None
        
        try:
            url = f"{RMF_API_SERVER_URL}/tasks/{rmf_task_id}/state"
            headers = {}
            if RMF_API_JWT_TOKEN:
                headers["Authorization"] = f"Bearer {RMF_API_JWT_TOKEN}"
            
            # Use synchronous httpx.Client for this check (called from ROS 2 callback)
            with httpx.Client(timeout=2.0) as client:
                response = client.get(url, headers=headers)
                if response.status_code == 200:
                    task_state = response.json()
                    status = task_state.get('status', 'unknown')
                    logger.debug(f"📋 Task {rmf_task_id} status from RMF: {status}")
                    return task_state
                else:
                    logger.debug(f"Failed to get task status for {rmf_task_id}: HTTP {response.status_code}")
        except httpx.TimeoutException:
            logger.debug(f"Timeout checking RMF task status for {rmf_task_id}")
        except Exception as e:
            logger.debug(f"Error checking RMF task status for {rmf_task_id}: {e}")
        
        return None
    
    def _mark_mission_canceled(self, mission_id: int, mission_data: dict, status: str):
        """Mark a mission as canceled"""
        current_state = mission_data.get('state')
        if current_state not in ['Cancelled', 'Complete', 'Failed']:
            mission_data['state'] = 'Cancelled'
            # Update step statuses
            for step in mission_data.get('steps', []):
                if step['step_status'] not in ['Complete', 'Failed']:
                    step['step_status'] = 'Cancelled'
            logger.info(f"🚫 Mission {mission_id} marked as Cancelled (was: {current_state}, RMF status: {status})")
        elif current_state == 'Complete':
            # If already marked complete, override with Cancelled since cancellation takes precedence
            logger.warning(f"⚠️ Mission {mission_id} was Complete but RMF shows canceled - updating to Cancelled")
            mission_data['state'] = 'Cancelled'
            for step in mission_data.get('steps', []):
                if step['step_status'] == 'Complete':
                    step['step_status'] = 'Cancelled'
    
    def _mark_mission_failed(self, mission_id: int, mission_data: dict):
        """Mark a mission as failed"""
        if mission_data.get('state') not in ['Cancelled', 'Complete']:
            mission_data['state'] = 'Failed'
            # Update step statuses
            for step in mission_data.get('steps', []):
                if step['step_status'] not in ['Complete', 'Cancelled']:
                    step['step_status'] = 'Failed'
            logger.info(f"❌ Mission {mission_id} marked as Failed")
    
    def _update_active_mission(self, mission_id: int, mission_data: dict, rmf_task_id: str):
        """
        Update an active mission with current robot assignment and status
        
        This function ensures that our mission data stays synchronized with RMF's
        actual robot assignments, preventing discrepancies between what we think
        is assigned and what RMF actually assigned.
        
        Also checks for cancellations even if task appears active.
        
        Args:
            mission_id: Internal mission identifier
            mission_data: Mission data dictionary
            rmf_task_id: RMF task identifier for tracking
        """
        try:
            # First, check if task was canceled (even if it appears active)
            # This handles cases where cancellation happened but task is still in active list
            task_status = self._get_rmf_task_status(rmf_task_id)
            if task_status:
                status = task_status.get('status', 'unknown')
                if status in ['canceled', 'killed']:
                    logger.info(f"🚫 Mission {mission_id} was canceled in RMF (status: {status})")
                    self._mark_mission_canceled(mission_id, mission_data, status)
                    return  # Don't process further if canceled
                elif status == 'completed':
                    logger.info(f"✅ Mission {mission_id} completed in RMF")
                    self._mark_mission_complete(mission_id, mission_data)
                    return
                elif status == 'failed':
                    logger.warning(f"❌ Mission {mission_id} failed in RMF")
                    self._mark_mission_failed(mission_id, mission_data)
                    return
            
            # Find which robot RMF actually assigned to this task
            assigned_robot = self._find_robot_by_task_id(rmf_task_id)
            
            if assigned_robot:
                # Always update robot assignment to match RMF reality
                previous_assignment = mission_data.get('assigned_machine', 'Unknown')
                
                if previous_assignment != assigned_robot:
                    # Log the assignment correction for audit trail
                    self._log_assignment_change(
                        mission_id, previous_assignment, assigned_robot, rmf_task_id
                    )
                    
                    # Update assignment fields
                    mission_data['assigned_machine'] = assigned_robot
                    mission_data['assigned_machine_id'] = _get_robot_id(assigned_robot)
                    mission_data['rmf_assignment_confirmed'] = True
                    mission_data['last_assignment_update'] = datetime.now()
                    
                    # Add to assignment history
                    self._add_assignment_history_entry(
                        mission_data, 'corrected', previous_assignment, assigned_robot
                    )
                    
                    logger.info(f"✅ Mission {mission_id} robot assignment synchronized: "
                               f"'{previous_assignment}' → '{assigned_robot}' (RMF Task: {rmf_task_id})")
                else:
                    # Assignment is already correct, just update timestamp
                    mission_data['last_assignment_update'] = datetime.now()
                    logger.debug(f"Mission {mission_id} robot assignment already correct: {assigned_robot}")
                
                # Update mission state if needed
                self._update_mission_execution_state(mission_id, mission_data)
                
            else:
                # RMF task exists but no robot is assigned - this indicates a problem
                self._handle_orphaned_rmf_task(mission_id, mission_data, rmf_task_id)
                
        except Exception as e:
            logger.error(f"🚨 Error updating mission {mission_id} assignment: {e}")
            # Don't raise - we want to continue processing other missions
            self._add_assignment_history_entry(
                mission_data, 'error', 
                mission_data.get('assigned_machine', 'Unknown'), 
                f"Error: {str(e)}"
            )
    
    def _find_robot_by_task_id(self, rmf_task_id: str) -> Optional[str]:
        """
        Find which robot is assigned to a specific RMF task
        
        Args:
            rmf_task_id: RMF task identifier
            
        Returns:
            Robot name if found, None otherwise
        """
        if not self.fleet_states:
            return None
        
        for fleet_name, fleet_data in self.fleet_states.items():
            for robot in fleet_data['robots']:
                if robot.task_id == rmf_task_id:
                    return robot.name
        
        return None
    
    def _log_assignment_change(self, mission_id: int, old_robot: str, new_robot: str, rmf_task_id: str):
        """
        Log robot assignment changes for audit and debugging
        
        Args:
            mission_id: Internal mission identifier
            old_robot: Previous robot assignment
            new_robot: New robot assignment
            rmf_task_id: RMF task identifier
        """
        change_log = {
            'timestamp': datetime.now().isoformat(),
            'mission_id': mission_id,
            'old_robot': old_robot,
            'new_robot': new_robot,
            'rmf_task_id': rmf_task_id,
            'change_type': 'robot_assignment_sync',
            'source': 'rmf_fleet_state'
        }
        
        logger.info(f"🔄 Robot assignment change logged: {json.dumps(change_log, indent=2)}")
    
    def _add_assignment_history_entry(self, mission_data: dict, change_type: str, 
                                    old_value: str, new_value: str):
        """
        Add entry to mission assignment history
        
        Args:
            mission_data: Mission data dictionary
            change_type: Type of change (e.g., 'corrected', 'error', 'initial')
            old_value: Previous value
            new_value: New value
        """
        if 'assignment_history' not in mission_data:
            mission_data['assignment_history'] = []
        
        history_entry = {
            'timestamp': datetime.now().isoformat(),
            'change_type': change_type,
            'old_value': old_value,
            'new_value': new_value,
            'source': 'mission_tracker'
        }
        
        mission_data['assignment_history'].append(history_entry)
        
        # Keep only last 20 entries to prevent memory bloat
        if len(mission_data['assignment_history']) > 20:
            mission_data['assignment_history'] = mission_data['assignment_history'][-20:]
    
    def _update_mission_execution_state(self, mission_id: int, mission_data: dict):
        """
        Update mission execution state based on current robot assignment
        
        Args:
            mission_id: Internal mission identifier
            mission_data: Mission data dictionary
        """
        current_state = mission_data.get('state')
        
        if current_state != 'Executing':
            mission_data['state'] = 'Executing'
            mission_data['current_step_index'] = 0
            
            # Update step statuses
            if 'steps' in mission_data and len(mission_data['steps']) >= 2:
                mission_data['steps'][0]['step_status'] = 'DrivingToPickup'
                mission_data['steps'][1]['step_status'] = 'Generated'
            
            logger.info(f"🚀 Mission {mission_id} state updated to Executing "
                       f"(Robot: {mission_data.get('assigned_machine')})")
    
    def _handle_orphaned_rmf_task(self, mission_id: int, mission_data: dict, rmf_task_id: str):
        """
        Handle cases where RMF task exists but no robot is assigned
        
        This can happen during task transitions or if there's a timing issue
        between RMF task creation and robot assignment.
        
        Args:
            mission_id: Internal mission identifier
            mission_data: Mission data dictionary
            rmf_task_id: RMF task identifier
        """
        logger.warning(f"⚠️ Mission {mission_id} has orphaned RMF task {rmf_task_id} "
                       f"- no robot currently assigned")
        
        # Add to assignment history
        self._add_assignment_history_entry(
            mission_data, 'orphaned', 
            mission_data.get('assigned_machine', 'Unknown'), 
            'No robot assigned'
        )
        
        # Check if this is a recent task (might still be processing)
        mission_created = mission_data.get('created_at')
        if mission_created:
            time_since_creation = (datetime.now() - mission_created).total_seconds()
            if time_since_creation < 60:  # Less than 1 minute old
                logger.info(f"⏳ Mission {mission_id} is recent ({time_since_creation:.1f}s old) "
                           f"- RMF task assignment might still be processing")
            else:
                logger.warning(f"⏰ Mission {mission_id} is {time_since_creation:.1f}s old "
                              f"- orphaned task might indicate a problem")
    
    def _mark_mission_complete(self, mission_id: int, mission_data: dict):
        """Mark a mission as complete"""
        if mission_data.get('state') != 'Complete':
            mission_data['state'] = 'Complete'
            mission_data['current_step_index'] = 1
            mission_data['steps'][0]['step_status'] = 'Complete'
            mission_data['steps'][1]['step_status'] = 'Complete'
            logger.info(f"Mission {mission_id} marked as complete")
    
    def _assign_available_robots(self):
        """
        Assign available robots to unassigned missions
        
        This function provides initial robot assignments based on availability,
        but these assignments will be corrected by RMF sync when the actual
        robot is assigned by RMF.
        """
        try:
            # Get available robots from fleet states
            available_robots = self._get_available_robots()
            
            if not available_robots:
                logger.debug("No available robots for assignment")
                return
            
            # Get unassigned missions
            unassigned_missions = self._get_unassigned_missions()
            
            if not unassigned_missions:
                logger.debug("No unassigned missions for robot assignment")
                return
            
            # Assign available robots to missions
            self._process_robot_assignments(available_robots, unassigned_missions)
            
        except Exception as e:
            logger.error(f"🚨 Error in robot assignment process: {e}")
    
    def _get_available_robots(self) -> List[str]:
        """Get list of currently available robots"""
        available_robots = []
        
        if not self.fleet_states:
            return available_robots
        
        for fleet_name, fleet_data in self.fleet_states.items():
            for robot in fleet_data['robots']:
                if robot.task_id == '' or robot.task_id is None:
                    available_robots.append(robot.name)
        
        logger.debug(f"Found {len(available_robots)} available robots: {available_robots}")
        return available_robots
    
    def _get_unassigned_missions(self) -> List[tuple]:
        """Get list of missions that need robot assignment"""
        unassigned_missions = []
        
        for mission_id, mission_data in missions_db.items():
            if (mission_data.get('state') == 'Generated' and 
                mission_data.get('assigned_machine') == 'Unknown'):
                unassigned_missions.append((mission_id, mission_data))
        
        logger.debug(f"Found {len(unassigned_missions)} unassigned missions")
        return unassigned_missions
    
    def _process_robot_assignments(self, available_robots: List[str], 
                                  unassigned_missions: List[tuple]):
        """Process robot assignments for unassigned missions"""
        assigned_count = 0
        
        for mission_id, mission_data in unassigned_missions:
            if available_robots:
                # Get next available robot
                assigned_robot = available_robots.pop(0)
                
                # Make initial assignment (will be corrected by RMF sync)
                self._make_initial_assignment(mission_id, mission_data, assigned_robot)
                assigned_count += 1
                
                logger.info(f"📋 Mission {mission_id} initially assigned to {assigned_robot} "
                           f"(pending RMF confirmation)")
            else:
                logger.warning(f"⚠️ No more available robots for mission {mission_id}")
                break
        
        if assigned_count > 0:
            logger.info(f"✅ Initial robot assignments completed: {assigned_count} missions assigned")
    
    def _make_initial_assignment(self, mission_id: int, mission_data: dict, robot_name: str):
        """Make initial robot assignment for a mission"""
        # Store initial assignment
        mission_data['assigned_machine'] = robot_name
        mission_data['assigned_machine_id'] = _get_robot_id(robot_name)
        mission_data['state'] = 'Queued'
        mission_data['rmf_assignment_confirmed'] = False
        mission_data['last_assignment_update'] = datetime.now()
        
        # Add to assignment history
        self._add_assignment_history_entry(
            mission_data, 'initial', 'Unknown', robot_name
        )
        
        logger.debug(f"Initial assignment: Mission {mission_id} → {robot_name}")

# =============================================================================
# HTTP-BASED RMF TASK DISPATCH (Uses API Server instead of direct ROS 2)
# =============================================================================

async def dispatch_task_via_api_server(
    pickup_place: str, 
    dropoff_place: str, 
    external_id: str,
    priority: Optional[int] = None
) -> Dict[str, Any]:
    """
    Dispatch RMF delivery task via API server HTTP endpoint
    
    This function sends tasks to the RMF API server via HTTP, which ensures:
    - Tasks are saved to the database
    - Tasks are visible in the RMF Dashboard
    - Task state updates are tracked properly
        
        Args:
            pickup_place: RMF Conveyor waypoint name for pickup (e.g., "Conveyor1")
            dropoff_place: RMF Conveyor waypoint name for dropoff (e.g., "Conveyor2")
            external_id: WCS mission ID for tracking
            priority: Optional MES priority level (integer). If > 0, task will be high priority.
        
    Returns:
        dict: RMF task dispatch response containing task state and booking ID
        
    Raises:
        HTTPException: If API server request fails
    """
    # Validate API server configuration
    if not RMF_API_SERVER_URL:
        raise HTTPException(
            status_code=500, 
            detail="RMF_API_SERVER_URL not configured. Set environment variable RMF_API_SERVER_URL"
        )
    
    if not RMF_API_JWT_TOKEN:
        logger.warning("⚠️ RMF_API_JWT_TOKEN not set. Task may fail if authentication is required.")
        
        # Use 0 for sim time (consistent with dispatch_delivery.py)
        start_time = 0
        
    # Default dispenser and ingestor names
    pickup_dispenser = DEFAULT_PICKUP_DISPENSER
    dropoff_ingestor = DEFAULT_DROPOFF_INGESTOR
        
        # Default payload (empty for basic delivery)
        payload_items = []
    
    # MES sends numeric IDs which are already converted to Conveyor names
    # pickup_place and dropoff_place are already Conveyor waypoint names (e.g., "Conveyor1", "Conveyor2")
    pickup_waypoint = pickup_place  # Already a Conveyor name
    dropoff_waypoint = dropoff_place  # Already a Conveyor name
    
    logger.info(
        f"📍 Using RMF waypoints: "
        f"pickup={pickup_waypoint}, dropoff={dropoff_waypoint}"
    )
    
    # Validate that waypoints are different (can't pick and drop on same conveyor)
    if pickup_waypoint == dropoff_waypoint:
        error_msg = (
            f"Cannot create task: pickup and dropoff map to the same waypoint ({pickup_waypoint}). "
            f"Please ensure MES sends different IDs that map to different conveyors."
        )
        logger.error(f"❌ {error_msg}")
        raise HTTPException(status_code=400, detail=error_msg)
    
    # Get friendly names for dashboard display
    pickup_conveyor, dropoff_conveyor = ensure_different_conveyors(pickup_waypoint, dropoff_waypoint)
    
    # Convert MES priority to RMF binary priority format
    rmf_priority = create_rmf_priority(priority)
        
        # Construct the RMF delivery task request payload
    # NOTE: Priority is optional but can help with robot assignment
    # Some fleets require priority to be set for tasks to be assigned
    task_request = {
            "type": "dispatch_task_request",
            "request": {
                "unix_millis_request_time": start_time,
                "unix_millis_earliest_start_time": start_time,
            "requester": "wcs",
            "category": DELIVERY_CATEGORY,
            # Priority in RMF binary format: {"type": "binary", "value": 1} for high, {"type": "binary", "value": 0} for low
            "priority": rmf_priority,  # None if not provided, otherwise RMF binary format
                "description": {
                    "pickup": {
                    "place": pickup_waypoint,  # ACTUAL RMF waypoint name (e.g., "Conveyor1") - CRITICAL!
                        "handler": pickup_dispenser,
                        "payload": payload_items
                    },
                    "dropoff": {
                    "place": dropoff_waypoint,  # ACTUAL RMF waypoint name (e.g., "Conveyor2") - CRITICAL!
                        "handler": dropoff_ingestor,
                        "payload": payload_items
                    }
            },
            "labels": [
                f"source=wcs", 
                f"external_id={external_id}",
                f"pickup={pickup_conveyor}",  # Friendly name for dashboard (e.g., "Conveyor1")
                f"destination={dropoff_conveyor}"  # Friendly name for dashboard (e.g., "Conveyor2")
            ]
        }
    }
    
    # Prepare HTTP request
    url = f"{RMF_API_SERVER_URL}/tasks/dispatch_task"
    headers = {
        "Content-Type": "application/json"
    }
    
    # Add JWT token if configured
    if RMF_API_JWT_TOKEN:
        headers["Authorization"] = f"Bearer {RMF_API_JWT_TOKEN}"
    
    logger.info(f"📤 Dispatching task via API server: {json.dumps(task_request, indent=2)}")
    logger.info(f"🌐 API Server URL: {url}")
    
    try:
        # Make HTTP POST request to API server
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(url, json=task_request, headers=headers)
            
            # Check response status
            if response.status_code == 200:
                response_data = response.json()
                logger.info(f"✅ Task dispatched successfully: {response_data}")
                logger.debug(f"Response type: {type(response_data)}, keys: {response_data.keys() if isinstance(response_data, dict) else 'N/A'}")
                
                # Log task assignment status for debugging
                if isinstance(response_data, dict):
                    task_state = response_data.get("state") or response_data.get("root", {}).get("state", {})
                    if task_state:
                        dispatch_status = task_state.get("dispatch", {}).get("status", "unknown")
                        assigned_to = task_state.get("assigned_to")
                        logger.info(f"📋 Task assignment status: dispatch={dispatch_status}, assigned_to={assigned_to}")
                        if not assigned_to:
                            logger.warning(
                                f"⚠️ Task created but no robot assigned yet. "
                                f"Dispatch status: {dispatch_status}. "
                                f"This is normal if robots are not available or task is queued."
                            )
                
                return response_data
            elif response.status_code == 400:
                # Bad request - task dispatch failed
                error_data = response.json()
                logger.error(f"❌ Task dispatch failed: {error_data}")
                raise HTTPException(
                    status_code=500,
                    detail=f"RMF task dispatch failed: {error_data.get('root', {}).get('error', 'Unknown error')}"
                )
            elif response.status_code == 401:
                logger.error("❌ Authentication failed - check RMF_API_JWT_TOKEN")
                raise HTTPException(
                    status_code=500,
                    detail="Authentication failed. Please configure RMF_API_JWT_TOKEN environment variable"
                )
            elif response.status_code == 403:
                error_detail = "Not authenticated"
                try:
                    error_data = response.json()
                    error_detail = error_data.get("detail", "Not authenticated")
                except:
                    pass
                logger.error(f"❌ Authentication required (403) - {error_detail}")
                raise HTTPException(
                    status_code=500,
                    detail=f"Authentication required. Please set RMF_API_JWT_TOKEN environment variable. Error: {error_detail}"
                )
            else:
                logger.error(f"❌ Unexpected API server response: {response.status_code} - {response.text}")
                raise HTTPException(
                    status_code=500,
                    detail=f"API server returned status {response.status_code}: {response.text}"
                )
                
    except httpx.TimeoutException:
        logger.error("❌ Timeout waiting for API server response")
        raise HTTPException(
            status_code=500,
            detail=f"Timeout waiting for RMF API server response. Is the API server running at {RMF_API_SERVER_URL}?"
        )
    except httpx.ConnectError as e:
        logger.error(f"❌ Connection error: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to connect to RMF API server at {RMF_API_SERVER_URL}. Is it running? Error: {str(e)}"
        )
    except httpx.RequestError as e:
        logger.error(f"❌ Request error: {e}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to connect to RMF API server at {RMF_API_SERVER_URL}: {str(e)}"
        )
            except Exception as e:
        import traceback
        error_traceback = traceback.format_exc()
        logger.error(f"❌ Unexpected error dispatching task: {e}")
        logger.error(f"Traceback: {error_traceback}")
        raise HTTPException(
            status_code=500,
            detail=f"Unexpected error dispatching task: {str(e)}"
        )

# =============================================================================
# GLOBAL MISSION TRACKER INSTANCE
# =============================================================================

# Global mission tracker instance (singleton pattern)
mission_tracker = None
background_task = None

def get_mission_tracker():
    """
    Get or create mission tracker instance
    
    Returns:
        MissionTracker: Singleton mission tracker instance
    """
    global mission_tracker
    if mission_tracker is None:
        try:
            # Ensure ROS 2 context is initialized
            if not ensure_ros_context():
                raise RuntimeError("Failed to initialize ROS 2 context")
            
            # Create mission tracker node
            mission_tracker = MissionTracker()
            logger.info("✅ Mission tracker created")
        
        except Exception as e:
            logger.error(f"Failed to create mission tracker: {e}")
            raise
    
    return mission_tracker

async def ensure_background_monitoring():
    """Ensure background monitoring is running, restart if needed"""
    global background_task
    
    # Check if background task is running
    if background_task is None or background_task.done():
        logger.warning("⚠️ Background monitoring is not running, restarting...")
        await start_background_monitoring()
        return True
    
    # Check if task has an exception (only if it's done and has an exception)
    if background_task.done() and background_task.exception():
        logger.error(f"🚨 Background task has exception: {background_task.exception()}")
        logger.info("🔄 Restarting background monitoring...")
        await start_background_monitoring()
        return True
    
    return False

async def start_background_monitoring():
    """Start background mission monitoring"""
    global background_task
    
    # Cancel existing task if running
    if background_task and not background_task.done():
        background_task.cancel()
        try:
            await background_task
        except asyncio.CancelledError:
            pass
        logger.info("✅ Existing background task cancelled")
    
    # Ensure mission tracker exists
    tracker = get_mission_tracker()
    
    # Start the ROS 2 node in the background
    background_task = asyncio.create_task(run_ros_node_with_restart(tracker))
    logger.info("✅ Background ROS 2 monitoring started")

async def run_ros_node_with_restart(node: Node):
    """Run ROS 2 node with automatic restart on failure"""
    restart_count = 0
    max_restarts = 5
    
    while restart_count < max_restarts:
        try:
            logger.info(f"🚀 Starting ROS 2 background task (attempt {restart_count + 1}/{max_restarts})")
            
            # Check if we need to reinitialize ROS 2 context
            if not is_ros_context_valid():
                logger.warning("🔄 ROS 2 context invalid, attempting to reinitialize...")
                try:
                    # Reset the global flag
                    global ros_context_initialized
                    ros_context_initialized = False
                    
                    # Try to reinitialize
                    if ensure_ros_context():
                        logger.info("✅ ROS 2 context reinitialized successfully")
                    else:
                        logger.error("❌ Failed to reinitialize ROS 2 context")
                        raise RuntimeError("ROS 2 context reinitialization failed")
                except Exception as e:
                    logger.error(f"🚨 ROS 2 context reinitialization failed: {e}")
                    restart_count += 1
                    if restart_count < max_restarts:
                        logger.info(f"🔄 Waiting 10 seconds before retry...")
                        await asyncio.sleep(10.0)
                        continue
                    else:
                        break
            
            await run_ros_node(node)
            
            # If we get here, the task completed normally
            logger.info("✅ ROS 2 background task completed normally")
            break
            
        except Exception as e:
            restart_count += 1
            logger.error(f"🚨 ROS 2 background task failed (attempt {restart_count}/{max_restarts}): {e}")
                        
            if restart_count < max_restarts:
                logger.info(f"🔄 Restarting ROS 2 background task in 5 seconds...")
                await asyncio.sleep(5.0)  # Wait before restarting
            else:
                logger.error(f"💥 Maximum restart attempts reached. ROS 2 monitoring stopped.")
                break

def ensure_ros_context():
    """Ensure ROS 2 context is initialized, but only once"""
    global ros_context_initialized
    
    if ros_context_initialized:
        logger.debug("✅ ROS 2 context already initialized")
        return True
    
    try:
        rclpy.init()
        ros_context_initialized = True
        logger.info("✅ ROS 2 context initialized successfully")
        return True
    except Exception as e:
        if "already initialized" in str(e):
            ros_context_initialized = True
            logger.info("✅ ROS 2 context already initialized")
            return True
        else:
            logger.error(f"Failed to initialize ROS 2: {e}")
            return False

def is_ros_context_valid():
    """Check if ROS 2 context is still valid"""
    try:
        # Try to create a simple node to test context validity
        test_node = rclpy.create_node('test_context')
        test_node.destroy_node()
        return True
    except Exception as e:
        error_msg = str(e)
        if "rclpy.init() has not been called" in error_msg:
            logger.warning("ROS 2 context not initialized")
            return False
        elif "rcl_shutdown already called" in error_msg:
            logger.warning("ROS 2 context already shut down")
            return False
        elif "Context.init() must only be called once" in error_msg:
            logger.warning("ROS 2 context initialization conflict")
            return False
        else:
            logger.warning(f"ROS 2 context validation error: {e}")
            return False

async def run_ros_node(node: Node):
    """Run ROS 2 node in background task"""
    try:
        logger.info("🚀 Starting ROS 2 node execution...")
        
        # Check if ROS 2 context is still valid
        if not is_ros_context_valid():
            logger.error("🚨 ROS 2 context is not valid, cannot start node")
            raise RuntimeError("ROS 2 context is not valid")
        
        # Create executor with proper error handling
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        logger.info("✅ ROS 2 executor created and node added")
        
        # Start the mission tracker timer now that the node is running
        try:
            node.start_timer()
            logger.info("✅ Mission tracker timer started successfully")
        except Exception as e:
            logger.error(f"Failed to start mission tracker timer: {e}")
            raise
        
        # Spin the node continuously with proper error handling
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while True:
            try:
                # Check context validity before each spin
                if not is_ros_context_valid():
                    logger.error("🚨 ROS 2 context became invalid during execution")
                    break
                
                # Process pending ROS 2 messages
                executor.spin_once(timeout_sec=0.1)
                await asyncio.sleep(0.1)  # Small delay to prevent blocking
                
                # Reset error counter on successful spin
                consecutive_errors = 0
                
            except Exception as e:
                consecutive_errors += 1
                logger.error(f"Error in ROS 2 spin_once (attempt {consecutive_errors}/{max_consecutive_errors}): {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    logger.error(f"🚨 Too many consecutive errors, stopping ROS 2 node")
                    break
                
                await asyncio.sleep(1.0)  # Wait before retrying
                continue
                
    except Exception as e:
        logger.error(f"Critical error in ROS 2 background task: {e}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        raise  # Re-raise to trigger restart
    finally:
        try:
            # Clean up executor
            if 'executor' in locals():
                executor.shutdown()
                logger.info("✅ ROS 2 executor shutdown completed")
        except Exception as cleanup_error:
            logger.warning(f"Warning during executor cleanup: {cleanup_error}")
        
        logger.info("🛑 ROS 2 background task stopped")

# =============================================================================
# MISSION STATUS UPDATE FUNCTION (Simplified)
# =============================================================================

def update_mission_status_from_fleet():
    """
    Update mission status from the latest fleet state data
    This function is now simplified since the MissionTracker handles updates automatically
    """
    # The MissionTracker now handles all updates automatically via timer
    # This function is kept for backward compatibility but does minimal work
    logger.debug("Mission status updates are now handled automatically by MissionTracker")

def debug_mission_states():
    """Debug function to show current mission and robot states"""
    logger.info("=== DEBUG: MISSION AND ROBOT STATES ===")
    
    # Show all missions
    for mission_id, mission_data in missions_db.items():
        logger.info(f"Mission {mission_id}: {mission_data.get('external_id')}")
        logger.info(f"  State: {mission_data.get('state')}")
        logger.info(f"  Robot: {mission_data.get('assigned_machine')}")
        logger.info(f"  RMF Task: {mission_data.get('rmf_task_id')}")
        logger.info(f"  RMF Confirmed: {mission_data.get('rmf_assignment_confirmed', False)}")
        logger.info(f"  Assignment History: {len(mission_data.get('assignment_history', []))} entries")
    
    # Show fleet state if available
    if mission_tracker and mission_tracker.fleet_states:
        for fleet_name, fleet_data in mission_tracker.fleet_states.items():
            logger.info(f"Fleet {fleet_name}:")
            for robot in fleet_data['robots']:
                logger.info(f"  Robot {robot.name}: task_id='{robot.task_id}', mode={robot.mode.mode}")
    else:
        logger.info("No fleet state data available")
    
    logger.info("=========================================")

def get_assignment_sync_status():
    """
    Get detailed assignment synchronization status for monitoring
    
    Returns:
        Dictionary containing sync status information
    """
    sync_status = {
        'timestamp': datetime.now().isoformat(),
        'total_missions': len(missions_db),
        'sync_summary': {
            'rmf_confirmed': 0,
            'pending_confirmation': 0,
            'assignment_errors': 0
        },
        'mission_details': {}
    }
    
    for mission_id, mission_data in missions_db.items():
        mission_sync_info = {
            'external_id': mission_data.get('external_id'),
            'current_robot': mission_data.get('assigned_machine'),
            'rmf_confirmed': mission_data.get('rmf_assignment_confirmed', False),
            'last_update': mission_data.get('last_assignment_update'),
            'assignment_history_count': len(mission_data.get('assignment_history', [])),
            'state': mission_data.get('state')
        }
        
        sync_status['mission_details'][mission_id] = mission_sync_info
        
        # Update summary counts
        if mission_sync_info['rmf_confirmed']:
            sync_status['sync_summary']['rmf_confirmed'] += 1
        else:
            sync_status['sync_summary']['pending_confirmation'] += 1
        
        # Check for assignment errors
        if any(entry.get('change_type') == 'error' 
               for entry in mission_data.get('assignment_history', [])):
            sync_status['sync_summary']['assignment_errors'] += 1
    
    return sync_status

def check_rmf_task_status():
    """Check the status of RMF tasks and robot assignments for debugging"""
    logger.info("🔍 Checking RMF task status...")
    
    if not mission_tracker or not mission_tracker.fleet_states:
        logger.warning("No fleet state data available")
        return {"error": "No fleet state data"}
    
    # Get current fleet state
    fleet_info = {}
    active_tasks = set()
    
    for fleet_name, fleet_data in mission_tracker.fleet_states.items():
        fleet_info[fleet_name] = {
            'robots': [],
            'active_tasks': []
        }
        
        for robot in fleet_data['robots']:
            robot_info = {
                'name': robot.name,
                'task_id': robot.task_id,
                'mode': robot.mode.mode,
                'available': robot.task_id == '' or robot.task_id is None
            }
            fleet_info[fleet_name]['robots'].append(robot_info)
            
            if robot.task_id and robot.task_id != '':
                active_tasks.add(robot.task_id)
                fleet_info[fleet_name]['active_tasks'].append(robot.task_id)
    
    # Check mission status
    mission_status = {}
    for mission_id, mission_data in missions_db.items():
        mission_status[mission_id] = {
            'external_id': mission_data.get('external_id'),
            'rmf_task_id': mission_data.get('rmf_task_id'),
            'state': mission_data.get('state'),
            'assigned_robot': mission_data.get('assigned_machine'),
            'created_at': mission_data.get('created_at'),
            'task_active': mission_data.get('rmf_task_id') in active_tasks if mission_data.get('rmf_task_id') else False
        }
    
    return {
        'fleet_info': fleet_info,
        'active_tasks': list(active_tasks),
        'mission_status': mission_status,
        'timestamp': datetime.now().isoformat()
    }

@app.get("/api/CheckRMFStatus")
async def check_rmf_status_endpoint(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Check RMF task status and robot assignments for debugging
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Current RMF system status
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning("❌ API key missing in CheckRMFStatus request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f"❌ Invalid API key provided in CheckRMFStatus: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f"✅ API key validated successfully for CheckRMFStatus request")
    try:
        return check_rmf_task_status()
    except Exception as e:
        logger.error(f"Error checking RMF status: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to check RMF status: {str(e)}")

@app.post("/api/RestartBackgroundTask")
async def restart_background_task_endpoint(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Manually restart the background ROS 2 monitoring task
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Restart operation result
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning("❌ API key missing in RestartBackgroundTask request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f"❌ Invalid API key provided in RestartBackgroundTask: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f"✅ API key validated successfully for RestartBackgroundTask request")
    try:
        await start_background_monitoring()
        
        return {
            "success": True,
            "message": "Background ROS 2 monitoring restarted",
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Error restarting background task: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to restart background task: {str(e)}")

@app.get("/api/DebugStatus")
async def debug_status_endpoint(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Debug endpoint to get detailed system status for troubleshooting
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Detailed system status information
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning("❌ API key missing in DebugStatus request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f"❌ Invalid API key provided in DebugStatus: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f"✅ API key validated successfully for DebugStatus request")
    try:
        global background_task, mission_tracker, ros_context_initialized
        
        # Get detailed status
        status_info = {
            "timestamp": datetime.now().isoformat(),
            "ros_context": {
                "initialized": ros_context_initialized,
                "valid": is_ros_context_valid(),
                "can_create_node": False
            },
            "background_task": {
                "exists": background_task is not None,
                "status": "unknown"
            },
            "mission_tracker": {
                "exists": mission_tracker is not None,
                "status": "unknown"
            },
            "missions": {
                "total": len(missions_db),
                "details": {}
            }
        }
        
        # Test ROS 2 context
        try:
            if is_ros_context_valid():
                test_node = rclpy.create_node('debug_test')
                test_node.destroy_node()
                status_info["ros_context"]["can_create_node"] = True
        except Exception as e:
            status_info["ros_context"]["error"] = str(e)
        
        # Check background task
        if background_task:
            if background_task.done():
                if background_task.cancelled():
                    status_info["background_task"]["status"] = "cancelled"
                elif background_task.exception():
                    status_info["background_task"]["status"] = "failed"
                    status_info["background_task"]["error"] = str(background_task.exception())
                else:
                    status_info["background_task"]["status"] = "completed"
            else:
                status_info["background_task"]["status"] = "running"
        
        # Check mission tracker
        if mission_tracker:
            try:
                if hasattr(mission_tracker, 'get_name'):
                    status_info["mission_tracker"]["status"] = "active"
                    status_info["mission_tracker"]["name"] = mission_tracker.get_name()
                    status_info["mission_tracker"]["fleet_states_count"] = len(mission_tracker.fleet_states) if hasattr(mission_tracker, 'fleet_states') else 0
                else:
                    status_info["mission_tracker"]["status"] = "invalid"
            except Exception as e:
                status_info["mission_tracker"]["status"] = "error"
                status_info["mission_tracker"]["error"] = str(e)
        
        # Get mission details
        for mission_id, mission_data in missions_db.items():
            status_info["missions"]["details"][mission_id] = {
                "external_id": mission_data.get('external_id'),
                "state": mission_data.get('state'),
                "assigned_robot": mission_data.get('assigned_machine'),
                "rmf_task_id": mission_data.get('rmf_task_id'),
                "created_at": str(mission_data.get('created_at', 'unknown'))
            }
        
        return status_info
        
    except Exception as e:
        logger.error(f"Error in debug status endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to get debug status: {str(e)}")

@app.get("/api/AssignmentSyncStatus")
async def assignment_sync_status_endpoint(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Get detailed assignment synchronization status for monitoring
    
    This endpoint shows how well your system is synchronized with RMF
    robot assignments, helping identify any sync issues.
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Assignment synchronization status information
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning("❌ API key missing in AssignmentSyncStatus request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f"❌ Invalid API key provided in AssignmentSyncStatus: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f"✅ API key validated successfully for AssignmentSyncStatus request")
    try:
        return get_assignment_sync_status()
    except Exception as e:
        logger.error(f"Error getting assignment sync status: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to get assignment sync status: {str(e)}")

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def _get_robot_id(robot_name: str) -> int:
    """
    Get numerical robot ID from robot name
    
    Args:
        robot_name: Robot name (e.g., "FourD_robot_3")
        
    Returns:
        Numerical robot ID (e.g., 3)
    """
    robot_id_map = {
        'FourD_robot_1': 1,
        'FourD_robot_2': 2,
        'FourD_robot_3': 3
    }
    return robot_id_map.get(robot_name, 0)

# Note: State mapping is now handled directly in update_mission_status_from_fleet()
# based on task assignment/completion detection from fleet states

# =============================================================================
# RMF TASK CREATION FUNCTION
# =============================================================================

async def create_rmf_delivery_task(pickup_place: str, dropoff_place: str, external_id: str, priority: Optional[int] = None):
    """
    Create and send RMF delivery task request via API server HTTP endpoint
    
    This function now uses the HTTP API endpoint instead of direct ROS 2 publishing,
    ensuring tasks are properly saved to the database and visible in the dashboard.
    
    Args:
        pickup_place: RMF place name for pickup
        dropoff_place: RMF place name for dropoff
        external_id: WCS mission ID for tracking
        priority: Optional MES priority level (integer). If > 0, task will be high priority.
        
    Returns:
        dict: RMF task dispatch response
    """
    try:
        # Dispatch task via API server HTTP endpoint
        response = await dispatch_task_via_api_server(pickup_place, dropoff_place, external_id, priority)
    
        # Handle different response formats from API server
        # Format 1: {"root": {"success": true, "state": {...}}}
        # Format 2: {"success": true, "state": {...}}
        success = False
        task_state = None
        
        if "root" in response:
            # Nested format
            root_response = response.get("root", {})
            success = root_response.get("success", False)
            task_state = root_response.get("state")
        else:
            # Direct format
            success = response.get("success", False)
            task_state = response.get("state")
        
        # Check if task dispatch was successful
        # success can be True (bool) or "True" (string), so check explicitly
        is_success = (success is True) or (success == "True") or (str(success).lower() == "true")
        has_state = task_state is not None and isinstance(task_state, dict) and len(task_state) > 0
        
        logger.debug(f"Response parsing: success={success} (type={type(success)}), is_success={is_success}, has_state={has_state}")
        
        if is_success and has_state:
            logger.info(f"✅ Delivery task created successfully via API server")
            logger.debug(f"Task state: {json.dumps(task_state, indent=2)}")
            # Normalize response to always have root structure for consistency
            return {"root": {"success": True, "state": task_state}}
        else:
            # Log the actual response for debugging
            logger.error(f"❌ Task dispatch check failed. Response: {json.dumps(response, indent=2)}")
            logger.error(f"   success={success} (type={type(success)}), is_success={is_success}")
            logger.error(f"   task_state={task_state} (type={type(task_state)}), has_state={has_state}")
            error_msg = response.get("root", {}).get("error") or response.get("error", "Unknown error")
            logger.error(f"❌ Task dispatch failed: {error_msg}")
            raise HTTPException(
                status_code=500,
                detail=f"RMF task dispatch failed: {error_msg}"
            )
            
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes and details
        raise
    except Exception as e:
        import traceback
        error_traceback = traceback.format_exc()
        logger.error(f"❌ Error creating RMF delivery task: {e}")
        logger.error(f"Traceback: {error_traceback}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to create RMF delivery task: {str(e)}"
        )

# =============================================================================
# FASTAPI ENDPOINTS
# =============================================================================

@app.post("/api/TestAuth")
async def test_authentication(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Test endpoint to verify API key authentication
    
    Returns:
        dict: Authentication test result
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning("❌ API key missing in test request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f"❌ Invalid API key provided in test: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f"✅ API key validation test passed")
    
    return {
        "success": True,
        "message": "API key authentication successful",
        "timestamp": datetime.now().isoformat(),
        "api_key_provided": x_api_key,
        "api_key_valid": True
    }

@app.get("/api/TestAPIServer")
async def test_api_server(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Test endpoint to verify connection to RMF API server
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Connection test result
    """
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        raise HTTPException(status_code=401, detail="API key required")
    if x_api_key != WCS_API_KEY:
        raise HTTPException(status_code=401, detail="Invalid API key")
    
    try:
        # Test connection to API server
        url = f"{RMF_API_SERVER_URL}/tasks"
        headers = {}
        if RMF_API_JWT_TOKEN:
            headers["Authorization"] = f"Bearer {RMF_API_JWT_TOKEN}"
        
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(url, headers=headers)
            
            return {
                "success": True,
                "api_server_url": RMF_API_SERVER_URL,
                "status_code": response.status_code,
                "message": "API server is accessible" if response.status_code < 500 else "API server returned error",
                "jwt_configured": bool(RMF_API_JWT_TOKEN)
            }
    except httpx.ConnectError as e:
        return {
            "success": False,
            "api_server_url": RMF_API_SERVER_URL,
            "error": f"Cannot connect to API server: {str(e)}",
            "message": "Make sure RMF API server is running at the configured URL"
        }
    except Exception as e:
        return {
            "success": False,
            "api_server_url": RMF_API_SERVER_URL,
            "error": str(e),
            "message": "Error testing API server connection"
    }

@app.post("/api/MissionCreate", response_model=List[MissionResponse])
async def mission_create(
    request: Request,
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Create a new patrol mission from WCS request
    
    Requires valid API key for authentication via Apikey header
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning("❌ API key missing in request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f"❌ Invalid API key provided: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f"✅ API key validated successfully for request")
    
    global mission_counter
    
    try:
        # Parse the JSON request from WCS
        data = await request.json()
        logger.info(f"Received patrol mission request: {json.dumps(data, indent=2)}")
        
        # Extract mission details from WCS request
        external_id = data.get("ExternalId", "")
        mission_name = data.get("Name", "")
        steps = data.get("Steps", [])
        options = data.get("Options", {})
        rounds = options.get("Rounds", 1) # Default to 1 for delivery tasks
        priority = options.get("Priority")  # Extract priority from MES Options (integer, optional)
        
        # Initialize list to store patrol locations
        pickup_place = ""
        dropoff_place = ""
        
        # Process each step to extract pickup and dropoff locations
        # MES sends numeric IDs (1, 2, 3) in AllowedTargets
        for step in steps:
            step_type = step.get("StepType", "")
            allowed_targets = step.get("AllowedTargets", [])
            
            # Extract numeric IDs directly from MES targets
            target_ids = [target.get("Id") for target in allowed_targets if target.get("Id") is not None]
            
            # Convert numeric ID directly to Conveyor waypoint name
            if step_type == "Pickup":
                target_id = target_ids[0] if target_ids else None
                if target_id is not None:
                    pickup_place = get_rmf_waypoint_from_id(target_id)  # e.g., 2 → "Conveyor2"
                    logger.info(f"Pickup step found: ID {target_id} → {pickup_place}")
            elif step_type == "Dropoff":
                target_id = target_ids[0] if target_ids else None
                if target_id is not None:
                    dropoff_place = get_rmf_waypoint_from_id(target_id)  # e.g., 1 → "Conveyor1"
                    logger.info(f"Dropoff step found: ID {target_id} → {dropoff_place}")
        
        # Validate that we have both pickup and dropoff places
        if not pickup_place or not dropoff_place:
            error_response = MissionResponse(
                ExternalId=external_id,
                InternalId=-1,
                Success=False,
                Description="Delivery mission generation failed, need both pickup and dropoff places",
                ReplyResult="StepTargetsNotValid"
            )
            logger.warning(f"Delivery mission failed: {error_response.Description}")
            return [error_response]
        
        logger.info(f"Using pickup place: {pickup_place}, dropoff place: {dropoff_place}")
        
        # Create RMF delivery task using the extracted locations
        try:
            rmf_response = await create_rmf_delivery_task(pickup_place, dropoff_place, external_id, priority)
            
            # Log priority information
            if priority is not None:
                priority_status = "HIGH" if priority > 0 else "LOW"
                logger.info(f"📌 Task created with {priority_status} priority (MES Priority: {priority})")
            else:
                logger.info(f"📌 Task created with default priority (no priority specified)")
            
            # Check if RMF task was successful
            # Handle different response formats: {"root": {...}} or {"success": ..., "state": ...}
            if "root" in rmf_response:
                root_response = rmf_response.get('root', {})
            else:
                # Direct format - normalize to root format
                root_response = {
                    "success": rmf_response.get('success', False),
                    "state": rmf_response.get('state', {})
                }
            
            if root_response.get('success', False):
                # Extract booking ID from RMF response for tracking
                booking_id = root_response.get('state', {}).get('booking', {}).get('id', 'unknown')
                
                # Create mission record with unique internal ID
                mission_id = mission_counter
                mission_counter += 1
                
                # Store mission data in our database
                # This data will be used by GetMissions endpoint for status tracking
                missions_db[mission_id] = {
                    'id': mission_id,
                    'external_id': external_id,
                    'name': mission_name,
                    'pickup_place': pickup_place,
                    'dropoff_place': dropoff_place,
                    'rmf_task_id': booking_id,
                    'state': 'Generated',  # Initial state
                    'assigned_machine': 'Unknown',  # Will be updated when robot is assigned
                    'assigned_machine_id': 0,
                    'current_step_index': 0,  # Start with first step
                    'created_at': datetime.now(),
                    # ✅ NEW: Assignment tracking fields
                    'assignment_history': [],  # Track all assignment changes
                    'rmf_assignment_confirmed': False,  # Whether RMF has confirmed assignment
                    'last_assignment_update': None,  # Timestamp of last assignment update
                    'steps': [
                        {
                            'step_type': 'Pickup',
                            'step_status': 'Generated',  # Initial step status
                            'current_target': f"Pickup ({get_id_from_conveyor(pickup_place)})",
                            'current_target_id': get_id_from_conveyor(pickup_place),
                            'wait_target': '',
                            'target_shelf_id': -1,
                            'load_type': 'FP-1',
                            'load_type_id': 1,
                            'reserving_location': False
                        },
                        {
                            'step_type': 'Dropoff',
                            'step_status': 'Generated',  # Initial step status
                            'current_target': f"Dropoff ({get_id_from_conveyor(dropoff_place)})",
                            'current_target_id': get_id_from_conveyor(dropoff_place),
                            'wait_target': '',
                            'target_shelf_id': -1,
                            'load_type': 'FP-1',
                            'load_type_id': 1,
                            'reserving_location': False
                        }
                    ]
                }
                
                # Create success response for WCS
                success_response = MissionResponse(
                    ExternalId=external_id,
                    InternalId=mission_id,
                    Success=True,
                    Description=f"Delivery mission created successfully. RMF Task ID: {booking_id}",
                    ReplyResult="Success"
                )
                
                logger.info(f"Delivery mission created successfully: {external_id} -> RMF Task: {booking_id}")
                return [success_response]
            else:
                # RMF task creation failed
                error_response = MissionResponse(
                    ExternalId=external_id,
                    InternalId=-1,
                    Success=False,
                    Description="RMF delivery task creation failed",
                    ReplyResult="TaskCreationFailed"
                )
                logger.warning(f"RMF delivery task failed: {error_response.Description}")
                return [error_response]
                
        except HTTPException:
            # Re-raise HTTP exceptions (timeout, etc.)
            raise
        except Exception as e:
            # Handle other RMF task creation errors
            error_response = MissionResponse(
                ExternalId=external_id,
                InternalId=-1,
                Success=False,
                Description=f"RMF delivery task creation error: {str(e)}",
                ReplyResult="TaskCreationError"
            )
            logger.error(f"RMF delivery task error: {error_response.Description}")
            return [error_response]
        
    except json.JSONDecodeError as e:
        # Handle invalid JSON in request
        logger.error(f"Invalid JSON format: {e}")
        raise HTTPException(status_code=400, detail="Invalid JSON format")
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes and details
        raise
    except Exception as e:
        # Handle any other unexpected errors with detailed logging
        import traceback
        error_traceback = traceback.format_exc()
        logger.error(f"Unexpected error in mission_create: {e}")
        logger.error(f"Traceback: {error_traceback}")
        raise HTTPException(
            status_code=500, 
            detail=f"Internal server error: {str(e)}. Check server logs for details."
        )

@app.get("/api/GetMissions", response_model=List[MissionData])
async def get_missions(
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Get all missions with current status and progress
    
    This endpoint returns real-time status of all missions including:
    - Mission state (Generated, Executing, Complete, etc.)
    - Robot assignments
    - Step-by-step progress
    - Current target locations
    
    The status is updated automatically every second by the MissionTracker node.
    No manual status updates are needed.
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        List[MissionData]: List of all missions with current status
    """
    
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        logger.warning(" API key missing in GetMissions request")
        raise HTTPException(
            status_code=401, 
            detail="API key required. Please provide Apikey header."
        )
    
    if x_api_key != WCS_API_KEY:
        logger.warning(f" Invalid API key provided in GetMissions: {x_api_key}")
        raise HTTPException(
            status_code=401, 
            detail="Invalid API key. Access denied."
        )
    
    logger.info(f" API key validated successfully for GetMissions request")
    
    # Get mission tracker to ensure it's initialized
    tracker = get_mission_tracker()
    
    # Ensure background monitoring is running
    await ensure_background_monitoring()
    
    # Debug: Show current missions in database
    logger.info("=== CURRENT MISSIONS IN DATABASE ===")
    for mission_id, mission_data in missions_db.items():
        logger.info(f"Mission {mission_id}: {mission_data.get('external_id')} -> RMF Task: {mission_data.get('rmf_task_id')} -> State: {mission_data.get('state')} -> Robot: {mission_data.get('assigned_machine')}")
    logger.info("===================================")
    
    # Mission status is now updated automatically by MissionTracker every second
    # No need to call update_mission_status_from_fleet() manually
    
    # Debug: Show detailed mission and robot states
    debug_mission_states()
    
    # Debug: Show missions after status update
    logger.info("=== MISSIONS AFTER STATUS UPDATE ===")
    for mission_id, mission_data in missions_db.items():
        logger.info(f"Mission {mission_id}: {mission_data.get('external_id')} -> RMF Task: {mission_data.get('rmf_task_id')} -> State: {mission_data.get('state')} -> Robot: {mission_data.get('assigned_machine')}")
    logger.info("===================================")
    
    # Convert internal mission data to WCS response format
    # The MissionTracker has already updated the missions_db automatically
    missions_response = []
    for mission_id, mission_data in missions_db.items():
        # Create StepData objects for each step
        steps_data = []
        for step in mission_data['steps']:
            step_data = StepData(
                StepType=step['step_type'],
                StepStatus=step['step_status'],
                CurrentTarget=step['current_target'],
                CurrentTargetId=step['current_target_id'],
                WaitTarget=step['wait_target'],
                TargetShelfId=step['target_shelf_id'],
                LoadType=step['load_type'],
                LoadTypeId=step['load_type_id'],
                ReservingLocation=step['reserving_location']
            )
            steps_data.append(step_data)
        
        # For delivery missions, determine the final target (dropoff)
        final_target = f"Dropoff ({get_id_from_conveyor(mission_data['dropoff_place'])})"
        final_target_id = get_id_from_conveyor(mission_data['dropoff_place'])
        
        # Create MissionData object for WCS response
        mission_response = MissionData(
            Id=mission_data['id'],
            MissionType="Mission",
            ExternalId=mission_data['external_id'],
            Name=mission_data['name'],
            State=mission_data['state'],
            AssignedMachine=mission_data['assigned_machine'],
            AssignedMachineId=mission_data.get('assigned_machine_id', 1),
            CurrentStepIndex=mission_data['current_step_index'],
            FinalTarget=final_target,
            FinalTargetId=final_target_id,
            Steps=steps_data
        )
        
        missions_response.append(mission_response)
    
    logger.info(f"Returning {len(missions_response)} missions")
    return missions_response

@app.get("/")
async def root():
    """
    Root endpoint - provides basic API information
    
    Returns:
        dict: API information and available endpoints
    """
    return {
        "message": "RMF Patrol Mission API is running", 
        "endpoints": [
            "/api/TestAuth",
            "/api/MissionCreate", 
            "/api/GetMissions", 
            "/api/CheckRMFStatus", 
            "/api/RestartBackgroundTask",
            "/api/DebugStatus",
            "/api/AssignmentSyncStatus"
        ],
        "mission_type": "Patrol",
        "features": [
            "Real-time mission tracking",
            "Professional robot assignment sync with RMF",
            "Assignment history tracking",
            "Automatic discrepancy resolution",
            "HTTP-based task dispatch via API server",
            "Dashboard integration support"
        ],
        "rmf_api_server": {
            "url": RMF_API_SERVER_URL,
            "jwt_configured": bool(RMF_API_JWT_TOKEN)
        }
    }

@app.get("/api/CheckTaskCancellation")
async def check_task_cancellation(
    task_id: str = Query(..., description="Task ID to check"),
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Check if a task can be cancelled and why/why not
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Task cancellation status information
    """
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        raise HTTPException(status_code=401, detail="API key required")
    if x_api_key != WCS_API_KEY:
        raise HTTPException(status_code=401, detail="Invalid API key")
    
    try:
        # Check task status via API server
        url = f"{RMF_API_SERVER_URL}/tasks/{task_id}/state"
        headers = {}
        if RMF_API_JWT_TOKEN:
            headers["Authorization"] = f"Bearer {RMF_API_JWT_TOKEN}"
        
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(url, headers=headers)
            
            if response.status_code == 200:
                task_state = response.json()
                status = task_state.get("status", "unknown")
                
                # Check if task is cancellable
                non_cancellable_statuses = ['canceled', 'killed', 'completed', 'failed']
                is_cancellable = status not in non_cancellable_statuses
                
                return {
                    "task_id": task_id,
                    "status": status,
                    "is_cancellable": is_cancellable,
                    "reason": f"Task status is '{status}'" + (
                        f" - cannot cancel tasks with status: {', '.join(non_cancellable_statuses)}"
                        if not is_cancellable
                        else " - task can be cancelled"
                    ),
                    "task_state": task_state
                }
            else:
                return {
                    "task_id": task_id,
                    "error": f"Failed to get task state: {response.status_code}",
                    "response": response.text
                }
    except Exception as e:
        return {
            "task_id": task_id,
            "error": str(e),
            "message": "Error checking task cancellation status"
        }

@app.post("/api/CancelTask")
async def cancel_task_via_api(
    task_id: str = Query(..., description="Task ID to cancel"),
    x_api_key: Optional[str] = Header(None, alias="Apikey")
):
    """
    Cancel a task via RMF API server
    
    Requires valid API key for authentication via Apikey header
    
    Returns:
        dict: Cancellation result
    """
    # 🔐 API KEY VALIDATION
    if not x_api_key:
        raise HTTPException(status_code=401, detail="API key required")
    if x_api_key != WCS_API_KEY:
        raise HTTPException(status_code=401, detail="Invalid API key")
    
    if not RMF_API_SERVER_URL:
        raise HTTPException(
            status_code=500,
            detail="RMF_API_SERVER_URL environment variable is not set."
        )
    
    try:
        # Cancel task via API server
        url = f"{RMF_API_SERVER_URL}/tasks/cancel_task"
        headers = {"Content-Type": "application/json"}
        if RMF_API_JWT_TOKEN:
            headers["Authorization"] = f"Bearer {RMF_API_JWT_TOKEN}"
        
        cancel_request = {
            "type": "cancel_task_request",
            "task_id": task_id,
            "labels": ["wcs", "api"]
        }
        
        async with httpx.AsyncClient(timeout=30.0) as client:  # Longer timeout for cancel
            response = await client.post(url, json=cancel_request, headers=headers)
            response.raise_for_status()
            response_data = response.json()
            
            logger.info(f"✅ Task {task_id} cancellation requested successfully")
            return {
                "success": True,
                "task_id": task_id,
                "message": "Task cancellation requested",
                "response": response_data
            }
    except httpx.HTTPStatusError as e:
        error_detail = f"API server returned status {e.response.status_code}"
        try:
            error_body = e.response.json()
            error_detail += f": {error_body}"
        except:
            error_detail += f": {e.response.text}"
        
        logger.error(f"❌ Failed to cancel task {task_id}: {error_detail}")
        raise HTTPException(
            status_code=e.response.status_code,
            detail=f"Failed to cancel task: {error_detail}"
        )
    except httpx.RequestError as e:
        error_msg = f"Could not connect to RMF API Server at {RMF_API_SERVER_URL}: {e}"
        logger.error(f"❌ {error_msg}")
        raise HTTPException(
            status_code=500,
            detail=error_msg
        )
    except Exception as e:
        error_msg = f"Unexpected error canceling task: {e}"
        logger.error(f"❌ {error_msg}")
        raise HTTPException(
            status_code=500,
            detail=error_msg
        )

@app.get("/health")
async def health_check():
    """
    Health check endpoint for monitoring
    
    Returns:
        dict: API health status and timestamp
    """
    global background_task, mission_tracker, ros_context_initialized
    
    # Check background task status
    background_status = "unknown"
    if background_task:
        if background_task.done():
            if background_task.cancelled():
                background_status = "cancelled"
            elif background_task.exception():
                background_status = f"failed: {background_task.exception()}"
            else:
                background_status = "completed"
        else:
            background_status = "running"
    
    # Check mission tracker status
    tracker_status = "unknown"
    if mission_tracker:
        try:
            # Check if node is still valid
            if hasattr(mission_tracker, 'get_name'):
                tracker_status = "active"
            else:
                tracker_status = "invalid"
        except Exception as e:
            tracker_status = f"error: {e}"
    
    # Check fleet state data
    fleet_data_status = "unknown"
    if mission_tracker and hasattr(mission_tracker, 'fleet_states'):
        fleet_count = len(mission_tracker.fleet_states)
        if fleet_count > 0:
            fleet_data_status = f"active ({fleet_count} fleets)"
        else:
            fleet_data_status = "no data"
    
    # Check ROS 2 context status
    ros_context_status = "unknown"
    try:
        if is_ros_context_valid():
            ros_context_status = "valid"
        else:
            ros_context_status = "invalid"
    except Exception as e:
        ros_context_status = f"error: {e}"
    
    # Check missions database
    missions_count = len(missions_db)
    active_missions = sum(1 for m in missions_db.values() if m.get('state') in ['Generated', 'Executing', 'Queued'])
    
    return {
        "status": "healthy" if background_status == "running" and ros_context_status == "valid" else "degraded",
        "timestamp": datetime.now().isoformat(),
        "api_type": "RMF Patrol Mission API",
        "authentication": {
            "api_key_required": True,
            "api_key_header": API_KEY_HEADER,
            "api_key_configured": bool(WCS_API_KEY)
        },
        "background_task": background_status,
        "mission_tracker": tracker_status,
        "fleet_data": fleet_data_status,
        "ros_context": {
            "status": ros_context_status,
            "initialized": ros_context_initialized
        },
        "missions": {
            "total": missions_count,
            "active": active_missions
        }
    }

# =============================================================================
# MAIN EXECUTION (for direct script execution)
# =============================================================================

if __name__ == "__main__":
    # Allow running the script directly for testing
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=1234)
