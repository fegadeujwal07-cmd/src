# execution_full_control → C++ (complete)

**Runtime:** C++ only — `execution_full_control` executable  
**Sources:**
- `include/execution/execution_full_control.hpp`
- `src/execution_full_control.cpp` — Sections 03–04 call `MQTTFleetClient` (same as Python import)
- `include/execution/mqtt_client.hpp` — port of `execution/mqtt_client.py`
- `src/mqtt_client.cpp`

## Progress

| Section | Status |
|---------|--------|
| **00–16** `execution_full_control.py` | **✅** |
| **S0** `mqtt_client.py` → `mqtt_client.hpp` / `mqtt_client.cpp` | **✅** (incl. `operatingMode`) |
| `operating_mode` on `/fleet_states` + delivery wait logs | **✅** |
| Launch → C++ `execution_full_control` | **✅** |

## Run

```bash
sudo apt install libmosquitto-dev   # once, if needed
colcon build --packages-select execution monitor
source install/setup.bash
ros2 launch execution execution_full_control.launch.py
# or
ros2 run execution execution_full_control
```
