# WCS API Details

**Base URL:** `http://10.10.17.164:1234`  
**Authentication:** Include header `Apikey: 123456789` in all requests

---

## 1. Create Mission

**POST** `/api/MissionCreate`

### Request Headers
```
Content-Type: application/json
Apikey: 123456789
```

### Request Body
```json
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

### Response (Success)
```json
[
    {
        "ExternalId": "680C4019-9A47-4C3C",
        "InternalId": 100,
        "Success": true,
        "Description": "Delivery mission created successfully. RMF Task ID: delivery.dispatch-0",
        "ReplyResult": "Success"
    }
]
```

---

## 2. Get Missions Status

**GET** `/api/GetMissions`

### Request Headers
```
Apikey: 123456789
```

### Response
```json
[
    {
        "Id": 101,
        "MissionType": "Mission",
        "ExternalId": "680C4019-9A47-4C3C",
        "Name": "WCS Mission",
        "State": "Generated",
        "AssignedMachine": "Unknown",
        "AssignedMachineId": 0,
        "CurrentStepIndex": 0,
        "FinalTarget": "Dropoff (7)",
        "FinalTargetId": 7,
        "Steps": [
            {
                "StepType": "Pickup",
                "StepStatus": "Generated",
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
                "StepStatus": "Generated",
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

### Mission States
- `Generated` - Mission created, waiting for robot
- `Executing` - Mission in progress
- `Complete` - Mission completed
- `Cancelled` - Mission cancelled
- `Failed` - Mission failed

### Step States
- `Generated` - Step not started
- `Executing` - Step in progress
- `Complete` - Step completed
- `Cancelled` - Step cancelled

### Example: Cancelled Mission
```json
{
    "Id": 100,
    "State": "Cancelled",
    "Steps": [
        {
            "StepType": "Pickup",
            "StepStatus": "Cancelled",
            "CurrentTarget": "Pickup (11)",
            "CurrentTargetId": 11
        },
        {
            "StepType": "Dropoff",
            "StepStatus": "Cancelled",
            "CurrentTarget": "Dropoff (7)",
            "CurrentTargetId": 7
        }
    ]
}
```

---

## Place IDs

WCS sends place identifiers (R1, R2, R7, R11, etc.). These are automatically mapped to RMF waypoints:
- **R1, R4, R10, R11, R23, R26, R29** → Conveyor1
- **R2, R5, R7, R21, R24, R27** → Conveyor2
- **R3, R6, R9, R22, R25, R28** → Conveyor3

**Note:** Pickup and dropoff must be on different conveyors.
