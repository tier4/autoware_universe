# Synthetic Bag Fixtures

Generate sqlite3 and MCAP fixtures from a sourced ROS 2 workspace:

```bash
python3 ../create_synthetic_bag.py --storage-id sqlite3 --output /tmp/ddsim_sqlite
python3 ../create_synthetic_bag.py --storage-id mcap --output /tmp/ddsim_mcap
```

The generated bags contain:

- `/localization/kinematic_state` (`nav_msgs/msg/Odometry`)
- `/control/command/control_cmd` (`autoware_control_msgs/msg/Control`)

They are intentionally generated rather than committed as binary files.
