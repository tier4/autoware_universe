^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_detection_area_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* test(detection_area): refactor and add unit tests (`#9087 <https://github.com/autowarefoundation/autoware.universe/issues/9087>`_)
* fix(behavior_velocity_planner): fix cppcheck warnings of virtualCallInConstructor (`#8376 <https://github.com/autowarefoundation/autoware.universe/issues/8376>`_)
  Co-authored-by: Ryuta Kambe <ryuta.kambe@tier4.jp>
* fix(autoware_behavior_velocity_detection_area_module): fix uninitMemberVar (`#8332 <https://github.com/autowarefoundation/autoware.universe/issues/8332>`_)
  fix:uninitMemberVar
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware.universe/issues/7640>`_)
* fix(autoware_behavior_velocity_planner_common): remove lane_id check from arc_lane_util (`#7710 <https://github.com/autowarefoundation/autoware.universe/issues/7710>`_)
  * fix(arc_lane_util): remove lane_id check from arc_lane_util
  * modify test_arc_lane_util.cpp
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(behavior_velocity_planner): move packages (`#7526 <https://github.com/autowarefoundation/autoware.universe/issues/7526>`_)
* Contributors: Fumiya Watanabe, Kosuke Takeuchi, Maxime CLEMENT, Takayuki Murooka, Yukinari Hisaki, Yutaka Kondo, kobayu858, taisa1

Forthcoming
-----------
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/tier4/autoware.universe/issues/9570>`_)
* feat(behavior_velocity_planner)!: remove stop_reason (`#9452 <https://github.com/tier4/autoware.universe/issues/9452>`_)
* feat(behavior_velocity_planner): replace first_stop_path_point_index (`#9296 <https://github.com/tier4/autoware.universe/issues/9296>`_)
  * feat(behavior_velocity_planner): replace first_stop_path_point_index
  * add const
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/scene.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/planner_manager.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/tier4/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/tier4/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/tier4/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* feat(detection_area)!: add retruction feature (`#9255 <https://github.com/tier4/autoware.universe/issues/9255>`_)
* test(detection_area): refactor and add unit tests (`#9087 <https://github.com/tier4/autoware.universe/issues/9087>`_)
* fix(behavior_velocity_planner): fix cppcheck warnings of virtualCallInConstructor (`#8376 <https://github.com/tier4/autoware.universe/issues/8376>`_)
  Co-authored-by: Ryuta Kambe <ryuta.kambe@tier4.jp>
* fix(autoware_behavior_velocity_detection_area_module): fix uninitMemberVar (`#8332 <https://github.com/tier4/autoware.universe/issues/8332>`_)
  fix:uninitMemberVar
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/tier4/autoware.universe/issues/7640>`_)
* fix(autoware_behavior_velocity_planner_common): remove lane_id check from arc_lane_util (`#7710 <https://github.com/tier4/autoware.universe/issues/7710>`_)
  * fix(arc_lane_util): remove lane_id check from arc_lane_util
  * modify test_arc_lane_util.cpp
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/tier4/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/tier4/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/tier4/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(behavior_velocity_planner): move packages (`#7526 <https://github.com/tier4/autoware.universe/issues/7526>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, M. Fatih Cırıt, Mamoru Sobue, Maxime CLEMENT, Takayuki Murooka, Yuki TAKAGI, Yukinari Hisaki, Yutaka Kondo, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
