# Mutex Supervisor - Test Procedure

## Prerequisites

1. RMF system running with fleet adapter
2. At least 2 robots in the fleet
3. Navigation graph with mutex zones defined (e.g., `0.yaml` with mutex M1)
4. Mutex supervisor package built and installed

## Step-by-Step Test Procedure

### Step 1: Verify Topics Are Available

Open Terminal 1 and check that required topics exist:

```bash
# Check itinerary_set topic
ros2 topic echo /rmf_traffic/itinerary_set -n 1

# Check fleet_states topic
ros2 topic echo /fleet_states -n 1

# Check robot_pause_requests topic (should be empty initially)
ros2 topic echo /robot_pause_requests
```

Expected output:
- `/rmf_traffic/itinerary_set`: Should show ItinerarySet messages when tasks are assigned
- `/fleet_states`: Should show FleetState messages with robot positions
- `/robot_pause_requests`: Empty initially, will show PauseRequest messages during test

### Step 2: Launch Mutex Supervisor

Open Terminal 2 and launch the mutex supervisor:

```bash
# Replace with actual path to your 0.yaml file
ros2 launch mutex_supervisor mutex_supervisor.launch.xml \
    nav_graph_path:=/path/to/install/execution/share/execution/config/nav_graphs/0.yaml \
    match_tolerance:=0.5 \
    claim_distance:=4.0 \
    exit_reached_distance:=0.7 \
    clearance_distance:=0.9
```

Expected output:
```
[INFO] [mutex_supervisor]: ============================================================
[INFO] [mutex_supervisor]: Mutex Supervisor - Configuration Parameters
[INFO] [mutex_supervisor]: ============================================================
[INFO] [mutex_supervisor]:   nav_graph_path: /path/to/0.yaml
[INFO] [mutex_supervisor]:   match_tolerance: 0.5 m
[INFO] [mutex_supervisor]:   claim_distance: 4.0 m
[INFO] [mutex_supervisor]:   exit_reached_distance: 0.7 m
[INFO] [mutex_supervisor]:   clearance_distance: 0.9 m
[INFO] [mutex_supervisor]: ============================================================
[INFO] [mutex_supervisor]: Loaded 1 mutex zones
[INFO] [mutex_supervisor]:   M1: entry=(9.30, -0.90), exit=(6.60, -0.90)
[INFO] [mutex_supervisor]: Subscribed to /rmf_traffic/itinerary_set
[INFO] [mutex_supervisor]: Subscribed to /fleet_states
[INFO] [mutex_supervisor]: Publisher created for /robot_pause_requests
```

### Step 3: Monitor Mutex Supervisor Logs

Keep Terminal 2 open to watch mutex supervisor logs. You should see:
- Itinerary set messages being received
- Mutex zone detections
- Robot status transitions
- Pause/resume commands

### Step 4: Send Delivery Tasks

Open Terminal 3 and send delivery tasks that cause both robots to traverse mutex M1.

**Example scenario**: Send tasks where:
- Robot 1 goes from its current location → through mutex M1 → to pickup location
- Robot 2 goes from its current location → through mutex M1 → to pickup location

Use your RMF task dispatch API or command-line tools to send these tasks.

### Step 5: Expected Log Sequence

Watch Terminal 2 for the following sequence:

#### Phase 1: Robot 1 Approaches and Claims

```
[INFO] [mutex_supervisor]: 📨 [ITINERARY SET] Received for participant 123, plan 1
[INFO] [mutex_supervisor]: 📋 [ITINERARY SET] Extracted 25 waypoints for participant 123
[INFO] [mutex_supervisor]: ✅ Mutex M1 detected in itinerary: entry at waypoint 5, exit at waypoint 12
[INFO] [mutex_supervisor]: 🔒 [ITINERARY SET] Participant 123 will pass through 1 mutex zone(s): ['M1']
[INFO] [mutex_supervisor]: 🔄 [amr1] Status: NORMAL → APPROACHING mutex M1 (dist_to_entry=3.8m <= 4.0m)
[INFO] [mutex_supervisor]: 🔒 Mutex M1 claimed by amr1
[INFO] [mutex_supervisor]: 🔄 [amr1] Mutex M1 status: APPROACHING → PASSING_MUTEX
```

#### Phase 2: Robot 2 Approaches and Gets Queued

```
[INFO] [mutex_supervisor]: 📨 [ITINERARY SET] Received for participant 124, plan 1
[INFO] [mutex_supervisor]: ✅ Mutex M1 detected in itinerary: entry at waypoint 8, exit at waypoint 15
[INFO] [mutex_supervisor]: 🔄 [amr2] Status: NORMAL → APPROACHING mutex M1 (dist_to_entry=3.5m <= 4.0m)
[INFO] [mutex_supervisor]: ⏳ amr2 queued for mutex M1, position=1
[INFO] [mutex_supervisor]: 🔄 [amr2] Mutex M1 status: APPROACHING → QUEUED_STOPPING
[INFO] [mutex_supervisor]: ⛔ Pause requested for robot amr2 (mutex M1, queued)
[INFO] [mutex_supervisor]: 🔄 [amr2] Mutex M1 status: QUEUED_STOPPING → STOPPED_WAITING
```

#### Phase 3: Robot 1 Clears Exit

```
[INFO] [mutex_supervisor]: ✅ [amr1] Exit reached for mutex M1 (dist_to_exit=0.6m <= 0.7m)
[INFO] [mutex_supervisor]: ✅ [amr1] Exit cleared for mutex M1 (dist_to_exit=0.9m >= 0.9m)
[INFO] [mutex_supervisor]: 🔓 Mutex M1 released by amr1
```

#### Phase 4: Robot 2 Gets Granted and Resumed

```
[INFO] [mutex_supervisor]: ➡️ Mutex M1 granted to next robot amr2
[INFO] [mutex_supervisor]: 🔄 [amr2] Mutex M1 status: STOPPED_WAITING → PASSING_MUTEX
[INFO] [mutex_supervisor]: ▶️ Resume requested for robot amr2 (mutex M1 granted)
```

### Step 6: Verify Pause/Resume Commands

In Terminal 1, monitor `/robot_pause_requests`:

```bash
ros2 topic echo /robot_pause_requests
```

Expected messages:

1. **Pause command for robot2**:
```
fleet_name: "amr_fleet"
robot_name: "amr2"
mode_request_id: 1
type: 0  # TYPE_PAUSE_IMMEDIATELY
at_checkpoint: 0
```

2. **Resume command for robot2**:
```
fleet_name: "amr_fleet"
robot_name: "amr2"
mode_request_id: 2
type: 2  # TYPE_RESUME
at_checkpoint: 0
```

### Step 7: Verify Robot Behavior

Observe the robots:
- Robot 1 should pass through mutex M1 without stopping
- Robot 2 should pause before mutex M1 entry
- Robot 2 should resume and proceed after robot 1 clears the exit

## Troubleshooting

### Issue: No mutex zones detected

**Check**:
- Navigation graph file path is correct
- Mutex zones are properly defined in YAML (vertices tagged with `mutex: M1`)
- `match_tolerance` is appropriate (try increasing if waypoints don't match)

### Issue: Robots not pausing

**Check**:
- `/robot_pause_requests` topic is being published
- Fleet adapter is subscribed to `/robot_pause_requests`
- Robot is actually within `claim_distance` of mutex entry
- Check mutex supervisor logs for status transitions

### Issue: Mapping warnings

**Check**:
- `/fleet_states` messages contain `robot_name` field
- Participant ID mapping is working (may need to refine based on your fleet adapter)

### Issue: Commands not taking effect

**Check**:
- Fleet adapter is listening to `/robot_pause_requests`
- `mode_request_id` is being handled correctly
- Command cooldown isn't preventing commands (check logs for cooldown messages)

## Success Criteria

✅ Mutex zones are detected from navigation graph  
✅ Robot plans are matched to mutex zones correctly  
✅ First robot claims mutex and proceeds  
✅ Second robot is queued and paused  
✅ First robot clears exit and releases mutex  
✅ Second robot is resumed and granted mutex  
✅ No duplicate pause/resume commands (cooldown working)  
✅ State resets correctly when new plans arrive  

