^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_static_freespace_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-01-30)
------------------
* feat: create static_freespace_planner based on existing freespace_planner package
* feat: load static route/waypoint CSVs from <static_map_path>/static_path and generate a trajectory per sequence
* feat: match route by start/goal poses and publish debug markers and route name
* feat: publish stop trajectory when inactive or on route matching failure, and publish diagnostics for matching errors
* feat: publish completion state and advance sequences when the vehicle stops and arrives at the end of the current sequence
----------
* Contributors: Katsuaki Kubo
