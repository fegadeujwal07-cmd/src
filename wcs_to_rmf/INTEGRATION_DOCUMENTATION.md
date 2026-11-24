# WCS to RMF Integration - Complete Documentation

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Setup and Installation](#setup-and-installation)
4. [Configuration](#configuration)
5. [API Endpoints](#api-endpoints)
6. [Key Features](#key-features)
7. [Waypoint Mapping](#waypoint-mapping)
8. [Task Lifecycle](#task-lifecycle)
9. [Authentication](#authentication)
10. [Troubleshooting](#troubleshooting)
11. [Testing Guide](#testing-guide)

---

## Overview

This integration provides a bridge between a **Warehouse Control System (WCS)** and the **(RMF)**. It enables WCS to dispatch delivery tasks to RMF robots through a REST API, with full bidirectional status synchronization.

### Key Capabilities
- ✅ **Task Dispatch**: WCS can create delivery tasks via REST API
- ✅ **Status Monitoring**: Real-time task status updates from RMF
- ✅ **Robot Assignment**: Automatic robot assignment by RMF
- ✅ **Task Cancellation**: Cancel tasks from RMF Dashboard with status sync back to WCS
- ✅ **Waypoint Mapping**: Translates WCS place names to RMF waypoint names
- ✅ **Business Logic**: Enforces pickup/dropoff on different conveyors
- ✅ **Dashboard Integration**: Tasks appear in RMF Dashboard with friendly names

---

## Architecture

### System Components

```
┌─────────────────┐
│   WCS System    │
│  (Postman/API)  │
└────────┬────────┘
         │ HTTP POST /api/MissionCreate
         ▼
┌─────────────────────────────────────┐
│   WCS Integration Server            │
│   (mission_api.py)                  │
│   - FastAPI Server (Port 1234)      │
│   - Task Translation                │
│   - Status Tracking                 │
└────────┬────────────────────────────┘
         │ HTTP POST /tasks/dispatch_task
         │ (with JWT Authentication)
         ▼
┌─────────────────────────────────────┐
│   RMF API Server                    │
│   (Port 8000)                       │
│   - Task Dispatch Endpoint          │
│   - Database Storage                │
│   - ROS 2 Bridge                    │
└────────┬────────────────────────────┘
         │ ROS 2 Topic: task_api_requests
         ▼
┌─────────────────────────────────────┐
│   RMF Core                          │
│   - Task Dispatcher                 │
│   - Robot Assignment                │
│   - Fleet Management                │
└────────┬────────────────────────────┘
         │ ROS 2 Topic: task_api_responses
         │ ROS 2 Topic: fleet_states
         ▼
┌─────────────────────────────────────┐
│   RMF Dashboard                     │
│   - Task Visualization              │
│   - Task Cancellation               │
│   - Real-time Status                │
└─────────────────────────────────────┘
```

### Communication Flow

1. **Task Creation**: WCS → WCS Integration Server → RMF API Server → RMF Core
2. **Status Updates**: RMF Core → RMF API Server → WCS Integration Server → WCS
3. **Task Cancellation**: RMF Dashboard → RMF API Server → RMF Core → WCS Integration Server

---

## Setup and Installation

### Prerequisites

1. **Python 3.8+**
2. **ROS 2** (Humble or later)
3. **RMF API Server** running on `http://localhost:8000`
4. **RMF Dashboard** (optional, for visualization)

### Installation Steps

1. **Navigate to the project directory:**
   ```bash
   cd /home/apurva/rmf_web/src/wcs_to_rmf
   ```

2. **Install Python dependencies:**
   ```bash
   pip3 install -r requirements.txt
   ```

3. **Install ROS 2 dependencies** (if not already installed):
   ```bash
   # Source ROS 2 environment
   source /opt/ros/humble/setup.bash
   
   # Install RMF messages
   sudo apt-get install ros-humble-rmf-fleet-msgs
   ```

4. **Generate JWT Token for RMF API Server:**
   ```bash
   python3 generate_jwt_token.py
   ```
   
   Copy the generated token for use in configuration.

5. **Set Environment Variables:**
   ```bash
   export RMF_API_SERVER_URL="http://localhost:8000"
   export RMF_API_JWT_TOKEN="<your_generated_token>"
   export WCS_API_KEY="123456789"  # Optional, for WCS authentication
   ```

6. **Start the WCS Integration Server:**
   ```bash
   python3 -m uvicorn wcs_to_rmf.mission_api:app --host 0.0.0.0 --port 1234
   ```

   Or with environment variables inline:
   ```bash
   RMF_API_SERVER_URL="http://localhost:8000" \
   RMF_API_JWT_TOKEN="<your_token>" \
   python3 -m uvicorn wcs_to_rmf.mission_api:app --host 0.0.0.0 --port 1234
   ```

---

## Configuration

### Environment Variables

| Variable | Description | Default | Required |
|----------|-------------|---------|----------|
| `RMF_API_SERVER_URL` | URL of the RMF API Server | `http://localhost:8000` | Yes |
| `RMF_API_JWT_TOKEN` | JWT token for API authentication | `""` | Yes (for authenticated endpoints) |
| `WCS_API_KEY` | API key for WCS authentication | `123456789` | No |

### Waypoint Mapping Configuration

The system uses two mapping dictionaries in `mission_api.py`:

#### 1. WCS to RMF Waypoint Map
Maps WCS place names (e.g., "R11", "R7") to actual RMF waypoint names (e.g., "Conveyor1", "Conveyor2"):

```python
WCS_TO_RMF_WAYPOINT_MAP = {
    "R1": "Conveyor1",
    "R2": "Conveyor2",
    "R3": "Conveyor3",
    "R7": "Conveyor2",
    "R11": "Conveyor1",
    # ... more mappings
}
```

#### 2. Place Name Map
Maps RMF waypoint names to friendly display names for the dashboard:

```python
PLACE_NAME_MAP = {
    "Conveyor1": "Conveyor1",
    "Conveyor2": "Conveyor2",
    "Conveyor3": "Conveyor3",
}
```

**To add new waypoints:**
1. Add the WCS place name → RMF waypoint mapping to `WCS_TO_RMF_WAYPOINT_MAP`
2. Add the RMF waypoint → friendly name mapping to `PLACE_NAME_MAP` (if different)

---

## API Endpoints

### 1. Create Mission (POST /api/MissionCreate)

Creates a new delivery task in RMF from WCS.

**Request:**
```http
POST http://10.10.17.135:1234/api/MissionCreate
Content-Type: application/json
Apikey: 123456789

{
    "ExternalId": "680C4019-9A47-4C3C",
    "Name": "WCS Mission",
    "Options": {
        "Priority": 1
    },
    "Steps": [
        {
            "StepType": "Pickup",
            "Options": {
                "Load": {
                    "RequiredLoadStatus": "LoadAtLocation",
                    "RequiredLoadType": 1
                },
                "SortingRules": ["Priority", "Closest"]
            },
            "AllowedTargets": [
                {"Id": "R11"}
            ],
            "AllowedWaits": null
        },
        {
            "StepType": "Dropoff",
            "Options": {
                "Load": {
                    "RequiredLoadStatus": "LocationHasRoom",
                    "RequiredLoadType": 1
                },
                "SortingRules": null
            },
            "AllowedTargets": [
                {"Id": "R7"}
            ],
            "AllowedWaits": null
        }
    ]
}
```

**Response:**
```json
{
    "ExternalId": "680C4019-9A47-4C3C",
    "InternalId": 100,
    "Success": true,
    "Description": "Mission created successfully",
    "ReplyResult": "OK"
}
```

**Error Response:**
```json
{
    "ExternalId": "680C4019-9A47-4C3C",
    "InternalId": 0,
    "Success": false,
    "Description": "Error message here",
    "ReplyResult": "ERROR"
}
```

### 2. Get Missions (GET /api/GetMissions)

Retrieves the current status of all missions.

**Request:**
```http
GET http://10.10.17.135:1234/api/GetMissions
```

**Response:**
```json
[
    {
        "Id": 100,
        "MissionType": "Mission",
        "ExternalId": "680C4019-9A47-4C3C",
        "Name": "WCS Mission",
        "State": "Executing",
        "AssignedMachine": "amr2",
        "AssignedMachineId": 2,
        "CurrentStepIndex": 1,
        "FinalTarget": "Dropoff (7)",
        "FinalTargetId": 7,
        "Steps": [
            {
                "StepType": "Pickup",
                "StepStatus": "Complete",
                "CurrentTarget": "Pickup (11)",
                "CurrentTargetId": 11,
                "WaitTarget": "",
                "TargetShelfId": -1,
                "LoadType": "FP-1",
                "LoadTypeId": 1,
                "ReservingLocation": false
            },
            {
                "StepType": "Dropoff",
                "StepStatus": "Executing",
                "CurrentTarget": "Dropoff (7)",
                "CurrentTargetId": 7,
                "WaitTarget": "",
                "TargetShelfId": -1,
                "LoadType": "FP-1",
                "LoadTypeId": 1,
                "ReservingLocation": false
            }
        ]
    }
]
```

**Mission States:**
- `"Generated"`: Task created but not yet assigned
- `"Executing"`: Task assigned and in progress
- `"Complete"`: Task completed successfully
- `"Cancelled"`: Task was canceled
- `"Failed"`: Task failed

### 3. Test API Server (GET /api/TestAPIServer)

Tests connectivity to the RMF API Server.

**Request:**
```http
GET http://10.10.17.135:1234/api/TestAPIServer
```

**Response:**
```json
{
    "connected": true,
    "url": "http://localhost:8000",
    "message": "Successfully connected to RMF API Server"
}
```

### 4. Cancel Task (POST /api/CancelTask)

Cancels a task via the RMF API Server.

**Request:**
```http
POST http://10.10.17.135:1234/api/CancelTask
Content-Type: application/json

{
    "task_id": "delivery.dispatch-5"
}
```

**Response:**
```json
{
    "success": true,
    "message": "Task canceled successfully"
}
```

---

## Key Features

### 1. Waypoint Name Translation

The system automatically translates WCS place names to RMF waypoint names:

- **WCS sends**: `"R11"` (pickup), `"R7"` (dropoff)
- **System translates to**: `"Conveyor1"` (pickup), `"Conveyor2"` (dropoff)
- **RMF receives**: Actual waypoint names that exist in the building map
- **Dashboard displays**: Friendly names like `"Conveyor1"` and `"Conveyor2"`

### 2. Business Logic Enforcement

The system enforces that pickup and dropoff cannot occur on the same conveyor:

- If WCS sends pickup and dropoff that map to the same conveyor, the system:
  1. Logs a warning
  2. Validates the waypoint mapping
  3. Returns an error if they are the same (validation happens before dispatch)

### 3. Real-time Status Synchronization

The `MissionTracker` class continuously monitors:
- **Fleet States**: Robot assignments and task progress
- **RMF Task Status**: Queries RMF API Server for task status (`queued`, `active`, `completed`, `canceled`, `failed`)
- **Mission Lifecycle**: Updates mission state based on RMF task status

### 4. Task Cancellation Detection

When a task is canceled from the RMF Dashboard:
1. RMF updates the task status to `"canceled"` or `"killed"`
2. `MissionTracker` queries the RMF API Server every second
3. If a task status is `"canceled"` or `"killed"`, the mission is marked as `"Cancelled"` in the WCS database
4. `GET /api/GetMissions` returns the updated status

### 5. Robot Assignment Tracking

The system tracks robot assignments from RMF:
- When a robot is assigned, the mission's `AssignedMachine` and `AssignedMachineId` are updated
- Robot names are extracted from RMF fleet states (e.g., `"amr2"`, `"FourD_robot_3"`)

---

## Waypoint Mapping

### How It Works

1. **WCS sends place names** like `"R11"` and `"R7"` in the mission request
2. **System translates** using `WCS_TO_RMF_WAYPOINT_MAP`:
   - `"R11"` → `"Conveyor1"`
   - `"R7"` → `"Conveyor2"`
3. **RMF receives** actual waypoint names in the task request:
   ```json
   {
       "description": {
           "pickup": {"place": "Conveyor1", ...},
           "dropoff": {"place": "Conveyor2", ...}
       }
   }
   ```
4. **Dashboard displays** friendly names from labels:
   ```json
   {
       "labels": [
           "pickup=Conveyor1",
           "destination=Conveyor2"
       ]
   }
   ```

### Adding New Waypoints

To add support for new WCS place names:

1. **Edit `WCS_TO_RMF_WAYPOINT_MAP` in `mission_api.py`:**
   ```python
   WCS_TO_RMF_WAYPOINT_MAP = {
       # ... existing mappings ...
       "R30": "Conveyor1",  # New mapping
       "R31": "Conveyor2",  # New mapping
   }
   ```

2. **Restart the server** for changes to take effect

---

## Task Lifecycle

### Task States in RMF

1. **`queued`**: Task created, waiting for robot assignment
2. **`active`**: Task assigned to robot, in progress
3. **`completed`**: Task finished successfully
4. **`canceled`**: Task canceled by user
5. **`killed`**: Task killed by system
6. **`failed`**: Task failed due to error

### Mission States in WCS

1. **`Generated`**: Mission created, waiting for assignment
2. **`Executing`**: Mission assigned and in progress
3. **`Complete`**: Mission completed successfully
4. **`Cancelled`**: Mission canceled
5. **`Failed`**: Mission failed

### State Transition Flow

```
WCS Request
    ↓
Mission Created (Generated)
    ↓
Task Dispatched to RMF (queued)
    ↓
Robot Assigned (Executing / active)
    ↓
Task Progress (Executing / active)
    ↓
Task Complete (Complete / completed)
    OR
Task Canceled (Cancelled / canceled)
    OR
Task Failed (Failed / failed)
```

---

## Authentication

### WCS API Authentication

The WCS Integration Server uses API key authentication:

- **Header**: `Apikey: 123456789`
- **Configuration**: Set via `WCS_API_KEY` environment variable
- **Default**: `"123456789"` (change in production!)

### RMF API Server Authentication

The RMF API Server uses JWT Bearer token authentication:

1. **Generate Token:**
   ```bash
   python3 generate_jwt_token.py
   ```

2. **Token Configuration:**
   - **Secret**: `"rmfisawesome"` (default)
   - **Audience**: `"rmf_api_server"`
   - **Issuer**: `"stub"`
   - **Username**: `"admin"` (default)

3. **Set Environment Variable:**
   ```bash
   export RMF_API_JWT_TOKEN="<generated_token>"
   ```

4. **Token is automatically included** in requests to RMF API Server:
   ```http
   Authorization: Bearer <token>
   ```

---

## Troubleshooting

### Common Issues and Solutions

#### 1. **403 Forbidden - "Not authenticated"**

**Problem**: RMF API Server rejects requests due to missing/invalid JWT token.

**Solution**:
1. Generate a new JWT token: `python3 generate_jwt_token.py`
2. Set the environment variable: `export RMF_API_JWT_TOKEN="<token>"`
3. Restart the server

#### 2. **Tasks Not Appearing in RMF Dashboard**

**Problem**: Tasks created from WCS don't show up in the dashboard.

**Possible Causes**:
- Tasks not saved to RMF API Server database
- Using direct ROS 2 publishing instead of HTTP API

**Solution**:
- Ensure you're using `dispatch_task_via_api_server()` (HTTP) not direct ROS 2 publishing
- Verify `RMF_API_SERVER_URL` is correct
- Check RMF API Server logs for errors

#### 3. **Robot Not Assigned to Task**

**Problem**: Tasks remain in `queued` state, no robot assigned.

**Possible Causes**:
- Invalid waypoint names (RMF can't find the location)
- No available robots in the fleet
- Task priority issues

**Solution**:
- Verify waypoint names in `WCS_TO_RMF_WAYPOINT_MAP` match actual RMF waypoints
- Check RMF fleet state: Are robots available?
- Check RMF logs for assignment errors

#### 4. **Cannot Cancel Task from Dashboard**

**Problem**: Getting 500 error when trying to cancel tasks from WCS.

**Possible Causes**:
- Task not properly registered in RMF (invalid waypoints)
- Task already completed/canceled
- API server connection issues

**Solution**:
- Verify waypoint mapping is correct
- Check task status in RMF Dashboard
- Verify `RMF_API_SERVER_URL` is accessible

#### 5. **Pickup and Dropoff Show Same Conveyor**

**Problem**: Dashboard shows the same conveyor for pickup and destination.

**Solution**:
- Check `WCS_TO_RMF_WAYPOINT_MAP` - ensure different WCS places map to different conveyors
- The system validates this and will error if they're the same
- Update the mapping if needed

#### 6. **ModuleNotFoundError: No module named 'httpx'**

**Problem**: Missing Python dependency.

**Solution**:
```bash
pip3 install -r requirements.txt
```

#### 7. **NameError: name 'Query' is not defined**

**Problem**: Missing FastAPI import.

**Solution**: Already fixed in code - ensure you have the latest version:
```python
from fastapi import FastAPI, Request, HTTPException, Header, Query
```

#### 8. **Cancellation Status Not Reflected in GetMissions**

**Problem**: Task canceled from dashboard, but `GetMissions` still shows old status.

**Solution**:
- The `MissionTracker` checks RMF status every second
- Wait 1-2 seconds after cancellation
- Verify `RMF_API_SERVER_URL` is correct and accessible
- Check server logs for status check errors

---

## Testing Guide

### 1. Test API Server Connectivity

```bash
curl http://10.10.17.135:1234/api/TestAPIServer
```

Expected: `{"connected": true, ...}`

### 2. Create a Test Mission

**Using curl:**
```bash
curl -X POST http://10.10.17.135:1234/api/MissionCreate \
  -H "Content-Type: application/json" \
  -H "Apikey: 123456789" \
  -d '{
    "ExternalId": "TEST-001",
    "Name": "Test Mission",
    "Options": {"Priority": 1},
    "Steps": [
      {
        "StepType": "Pickup",
        "Options": {
          "Load": {
            "RequiredLoadStatus": "LoadAtLocation",
            "RequiredLoadType": 1
          }
        },
        "AllowedTargets": [{"Id": "R11"}]
      },
      {
        "StepType": "Dropoff",
        "Options": {
          "Load": {
            "RequiredLoadStatus": "LocationHasRoom",
            "RequiredLoadType": 1
          }
        },
        "AllowedTargets": [{"Id": "R7"}]
      }
    ]
  }'
```

**Using Postman:**
1. Method: `POST`
2. URL: `http://10.10.17.135:1234/api/MissionCreate`
3. Headers:
   - `Content-Type: application/json`
   - `Apikey: 123456789`
4. Body: Use the JSON from above

### 3. Check Mission Status

```bash
curl http://10.10.17.135:1234/api/GetMissions
```

### 4. Test Task Cancellation

1. Create a task from WCS
2. Open RMF Dashboard
3. Find the task (should show "Conveyor1" → "Conveyor2")
4. Click "Cancel" on the task
5. Wait 1-2 seconds
6. Call `GET /api/GetMissions` - should show `"State": "Cancelled"`

### 5. Verify Waypoint Mapping

1. Create a task with pickup `"R11"` and dropoff `"R7"`
2. Check RMF Dashboard - should show:
   - Pickup: `"Conveyor1"`
   - Destination: `"Conveyor2"`
3. Verify robot assignment occurs
4. Verify task can be canceled

---

## File Structure

```
/home/apurva/rmf_web/src/wcs_to_rmf/
├── mission_api.py          # Main FastAPI server and integration logic
├── generate_jwt_token.py   # JWT token generator for RMF API Server
├── requirements.txt         # Python dependencies
└── INTEGRATION_DOCUMENTATION.md  # This file
```

---

## Key Code Sections

### Task Dispatch Function

Located in `mission_api.py`, the `dispatch_task_via_api_server()` function:
- Translates WCS place names to RMF waypoints
- Validates pickup/dropoff are different
- Constructs RMF task request
- Sends HTTP POST to RMF API Server
- Handles authentication and errors

### Mission Tracker Class

The `MissionTracker` class:
- Monitors ROS 2 fleet states
- Queries RMF API Server for task status
- Updates mission state in `missions_db`
- Detects cancellations and failures
- Runs background monitoring loop

### Waypoint Mapping Functions

- `get_rmf_waypoint_name()`: Converts WCS place → RMF waypoint
- `get_friendly_place_name()`: Gets display name for dashboard
- `ensure_different_conveyors()`: Validates business logic

---

## Summary of Changes Made

### Initial State
- WCS integration published directly to ROS 2 topics
- Tasks not visible in RMF Dashboard
- No waypoint name translation
- No status synchronization

### Final State
- ✅ HTTP-based communication via RMF API Server
- ✅ Tasks visible in RMF Dashboard
- ✅ Waypoint name translation (R11/R7 → Conveyor1/Conveyor2)
- ✅ Real-time status synchronization
- ✅ Task cancellation support
- ✅ Business logic enforcement (different conveyors)
- ✅ JWT authentication
- ✅ Comprehensive error handling

---

## Support and Maintenance

### Logs

The server logs important events:
- `INFO`: Task creation, status updates, robot assignments
- `WARNING`: Business logic violations, missing configuration
- `ERROR`: API failures, connection errors

### Monitoring

Key metrics to monitor:
- Task creation success rate
- Robot assignment time
- Task completion rate
- Cancellation detection accuracy

### Updates

To update waypoint mappings or business logic:
1. Edit `mission_api.py`
2. Restart the server
3. Test with a new task

---

## Conclusion

This integration provides a robust, production-ready bridge between WCS and RMF, with full bidirectional communication, status synchronization, and error handling. The system is designed to be maintainable and extensible for future requirements.

For questions or issues, refer to the troubleshooting section or check the server logs for detailed error messages.



