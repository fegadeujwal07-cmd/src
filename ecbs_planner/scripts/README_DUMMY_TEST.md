# ECBS dummy test (path register + evaluate + conflict)

## What this proves

1. Dummy publishes full path + ETA on `/from_zone_manager`
2. ECBS `evaluate_path_server` runs `solve` → stores in `ScheduleCache`
3. Dummy evaluate client calls `/evaluate_path_service` `mode=evaluate` every **10 s** for next **20 m** (req/res)
4. All publishes + requests + responses go to text logs
5. Conflict publisher sends head-on path → ECBS inserts waits (FIFO) into cache

## Build

```bash
cd /home/arm-robo1/FMS_ECBS
source /opt/ros/humble/setup.bash
colcon build --packages-select ecbs_planner
source install/setup.bash
```

## FCFS register gate (continuous, live)

With the fleet + `evaluate_path_server` running, leave this node up:

```bash
source /home/arm-robo1/FMS_ECBS/install/setup.bash
ros2 run ecbs_planner fcfs_register_tester.py \
  --log-dir "$HOME/FMS_ECBS/ecbs_test_logs"
```

- `/from_zone_manager` → who got the task first (FCFS order)
- `/to_zone_manager` → WAIT deps after evaluate
- On conflict: **later** AMR `mode=register` WAIT for **first**; first is GO

Log: `fcfs_register.txt`.

## Run (3 terminals)

**Terminal A — server + path1 + evaluate amr1**

```bash
source /home/arm-robo1/FMS_ECBS/install/setup.bash
ros2 launch ecbs_planner ecbs_dummy_test.launch.xml
```

**Terminal B — (later) conflict path amr2**

```bash
source /home/arm-robo1/FMS_ECBS/install/setup.bash
ros2 run ecbs_planner dummy_conflict_publisher.py \
  --log-dir "$HOME/FMS_ECBS/ecbs_test_logs"
```

**Terminal C — (optional) evaluate amr2 after conflict**

```bash
source /home/arm-robo1/FMS_ECBS/install/setup.bash
ros2 run ecbs_planner dummy_evaluate_client.py \
  --amr-id amr2 --task-id task_B --path-dir west \
  --log-dir "$HOME/FMS_ECBS/ecbs_test_logs"
```

## Logs

Default dir: `~/FMS_ECBS/ecbs_test_logs/`

| File | Content |
|------|---------|
| `published_paths.txt` | Every `/from_zone_manager` publish (path1 + conflict) |
| `evaluate_req_res.txt` | Every EvaluatePath request + response |

## What happens after conflict

```text
amr1 already in ScheduleCache
amr2 head-on path arrives on /from_zone_manager
  → EcbsSolver::solve(amr2, cache)
  → same cells + overlapping ETA → conflict
  → MUTUAL: both amr1 and amr2 get wait + waiting_for
  → both upserted into ScheduleCache
  → both published on /to_zone_manager
evaluate(amr1) or evaluate(amr2) → status=WAIT with deps from cache
```
