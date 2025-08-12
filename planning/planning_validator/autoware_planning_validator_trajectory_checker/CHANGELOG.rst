^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator_trajectory_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.0 (2025-08-11)
-------------------
* refactor(planning_validator): refactor planning validator configuration and error handling (`#11081 <https://github.com/autowarefoundation/autoware_universe/issues/11081>`_)
  * refactor trajectory check error handling
  * define set_diag_status function for each module locally
  * update documentation
  ---------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(planning_validator): fix conflict between trajectory shift and distance deviation checks (`#10799 <https://github.com/autowarefoundation/autoware_universe/issues/10799>`_)
  * use global is_critical_error flag for trajectory diagnostics update
  * add warning to readme
  * Update planning/planning_validator/autoware_planning_validator/README.md
  ---------
* fix(planning_validator_trajectory_checker): set is_critical_error flag to false at start of validation (`#10912 <https://github.com/autowarefoundation/autoware_universe/issues/10912>`_)
  set is_critical_error flag to false at start of validation
* fix(planning_validator): check the yaw deviation of the initial trajectory (`#10878 <https://github.com/autowarefoundation/autoware_universe/issues/10878>`_)
* Contributors: Maxime CLEMENT, Mete Fatih Cırıt, mkquda

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(planning_validator): add condition to check the yaw deviation (`#10818 <https://github.com/autowarefoundation/autoware_universe/issues/10818>`_)
* fix(planning_validator): fix the lateral distance calculation (`#10801 <https://github.com/autowarefoundation/autoware_universe/issues/10801>`_)
* feat(planning_validator): subscribe additional topic for collision detection (`#10745 <https://github.com/autowarefoundation/autoware_universe/issues/10745>`_)
  * feat(planning_validator): subscribe pointcloud
  * feat(planning_validator): subscribe route and map
  * fix(planning_validator): load glog component
  * fix: unexpected test fail
  ---------
* refactor(planning_validator): implement plugin structure for planning validator node (`#10571 <https://github.com/autowarefoundation/autoware_universe/issues/10571>`_)
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * create planning latency validator plugin module
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * create planning trajectory validator plugin module
  * Update planning/planning_validator/autoware_planning_validator/src/manager.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * minor fix
  * refactor implementation
  * uncomment lines for adding pose markers
  * fix CMakeLists
  * add comment
  * update planning launch xml
  * Update planning/planning_validator/autoware_planning_latency_validator/include/autoware/planning_latency_validator/latency_validator.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/plugin_interface.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/types.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_latency_validator/src/latency_validator.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * apply pre-commit checks
  * rename plugins for consistency
  * rename directories and files to match package names
  * refactor planning validator tests
  * add packages maintainer
  * separate trajectory check parameters
  * add missing package dependencies
  * move trajectory diagnostics test to trajectory checker module
  * remove blank line
  * add launch args for validator modules
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: GitHub Action <action@github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Maxime CLEMENT, Satoshi OTA, TaikiYamada4, mkquda
