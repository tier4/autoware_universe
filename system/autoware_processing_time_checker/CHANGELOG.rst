^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_processing_time_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): add processing time pub. (`#9065 <https://github.com/autowarefoundation/autoware.universe/issues/9065>`_)
  * feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): Add: processing_time_pub
  * fix: pre-commit
  * feat(costmap_generator): fix: No output when not Active.
  * fix: clang-format
  * Re: fix: clang-format
  ---------
* feat(processing_time_checker): add a new package (`#7957 <https://github.com/autowarefoundation/autoware.universe/issues/7957>`_)
  * feat(processing_time_checker): add a new package
  * fix
  * fix
  * update README and schema.json
  * fix
  * fix
  * fix
  ---------
* Contributors: Kazunori-Nakajima, Takayuki Murooka, Yutaka Kondo

Forthcoming
-----------
* fix(autoware_processing_time_checker): fix typo (`#9504 <https://github.com/tier4/autoware.universe/issues/9504>`_)
* feat(autoware_processing_time_checker): add a trigger to choice whether to output metrics to log folder (`#9479 <https://github.com/tier4/autoware.universe/issues/9479>`_)
  * add output_metrics option.
  * move param set from config to launch file.
  * fix bug.
  ---------
* feat(processing_time_checker): update processing time list (`#9350 <https://github.com/tier4/autoware.universe/issues/9350>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/tier4/autoware.universe/issues/9304>`_)
* feat(tier4_metric_msgs): apply tier4_metric_msgs for scenario_simulator_v2_adapter, control_evaluator, planning_evaluator, autonomous_emergency_braking, obstacle_cruise_planner, motion_velocity_planner, processing_time_checker (`#9180 <https://github.com/tier4/autoware.universe/issues/9180>`_)
  * first commit
  * fix building errs.
  * change diagnostic messages to metric messages for publishing decision.
  * fix bug about motion_velocity_planner
  * change the diagnostic msg to metric msg in autoware_obstacle_cruise_planner.
  * tmp save for planning_evaluator
  * change the topic to which metrics published to.
  * fix typo.
  * remove unnesessary publishing of metrics.
  * mke planning_evaluator publish msg of MetricArray instead of Diags.
  * update aeb with metric type for decision.
  * fix some bug
  * remove autoware_evaluator_utils package.
  * remove diagnostic_msgs dependency of planning_evaluator
  * use metric_msgs for autoware_processing_time_checker.
  * rewrite diagnostic_convertor to scenario_simulator_v2_adapter, supporting metric_msgs.
  * pre-commit and fix typo
  * publish metrics even if there is no metric in the MetricArray.
  * modify the metric name of processing_time.
  * update unit test for test_planning/control_evaluator
  * manual pre-commit
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/tier4/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/tier4/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): add processing time pub. (`#9065 <https://github.com/tier4/autoware.universe/issues/9065>`_)
  * feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): Add: processing_time_pub
  * fix: pre-commit
  * feat(costmap_generator): fix: No output when not Active.
  * fix: clang-format
  * Re: fix: clang-format
  ---------
* feat(processing_time_checker): add a new package (`#7957 <https://github.com/tier4/autoware.universe/issues/7957>`_)
  * feat(processing_time_checker): add a new package
  * fix
  * fix
  * update README and schema.json
  * fix
  * fix
  * fix
  ---------
* Contributors: Esteve Fernandez, Kazunori-Nakajima, Kem (TiankuiXian), Ryuta Kambe, Takayuki Murooka, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
