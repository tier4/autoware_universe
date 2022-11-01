# accel_brake_map_calibrator

The role of this node is to automatically calibrate `accel_map.csv` / `brake_map.csv` used in the `raw_vehicle_cmd_converter` node.

The base map, which is lexus's one by default, is updated iteratively with the loaded driving data.

## How to calibrate

### Launch Calibrator

After launching Autoware, run the `accel_brake_map_calibrator` by the following command and then perform autonomous driving. Note: You can collect data with manual driving if it is possible to use the same vehicle interface as during autonomous driving (e.g. using a joystick).

```
ros2 launch accel_brake_map_calibrator acceleration_map_calibrator.launch.xml rviz:=true
```
