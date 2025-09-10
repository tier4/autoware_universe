^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lanelet2_extension
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2024-11-21)
------------------
* fix: update the github link of map_loader (`#31 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/31>`_)
  add autoware prefix to map_loader
* Contributors: Masaki Baba

0.6.1 (2024-09-06)
------------------
* feat(lanelet2_extension): add util/visualization of BusStopArea (`#28 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/28>`_)
* feat(lanelet2_extension): add query for bicycle_lane (`#29 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/29>`_)
  add bicycle_lane query
* Contributors: Mamoru Sobue

0.6.0 (2024-08-28)
------------------
* feat(lanelet2_extension)!: release format_v2 (`#26 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/26>`_)
* docs(autoware_lanelet2_extension): refactor document (`#25 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/25>`_)
  * refactor doc
  * Update autoware_lanelet2_extension/docs/lanelet2_format_extension.md
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* feat(lanelet2_extension): format v2 bicycle lane doc (`#24 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/24>`_)
* feat(lanelet2_extension): update documentation for bus_stop_area to be introduced in format_version2 (`#21 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/21>`_)
* feat(lanelet2_extension): add bus_stop_area implementation (`#22 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/22>`_)
* Contributors: Mamoru Sobue

0.5.0 (2024-07-10)
------------------
* feat(lanelet2_extension)!: introduce API versioning along with format_version (`#18 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/18>`_)
* build: remove redundant move for build on noble (`#12 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/12>`_)
  remove redundant move
* refactor: remove redundant cmake definition (`#13 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/13>`_)
  * remove redundant cmake definition
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: apply pre-commit (`#14 <https://github.com/autowarefoundation/autoware_lanelet2_extension/issues/14>`_)
  apply pre-commit
* Contributors: Daisuke Nishimatsu, Mamoru Sobue

0.4.0 (2024-06-24)
------------------
* Merge pull request `#11 <https://github.com/youtalk/autoware_lanelet2_extension/issues/11>`_ from youtalk/import-update
  feat: import updates from `autoware_common`
* fix link
* feat(lanelet2_extension): overwriteLaneletsCenterline supports "waypoints" (`#252 <https://github.com/youtalk/autoware_lanelet2_extension/issues/252>`_)
  * feat(lanelet2_extension): centerline is converted to waypoints
  * fix lanelet2_extension_python
  * update README
  * fix
  * fix
  * early return
  * fix clang-tidy
  * Update tmp/lanelet2_extension/lib/utilities.cpp
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  * style(pre-commit): autofix
  * fix
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: boost optional build error on rolling environment (`#241 <https://github.com/youtalk/autoware_lanelet2_extension/issues/241>`_)
* perf(lanelet2_extension): use std::unordered_set<>::find instead of std::find (`#244 <https://github.com/youtalk/autoware_lanelet2_extension/issues/244>`_)
  perf(exists): use std::unordered_set<>::find instead of std::find
* Contributors: Maxime CLEMENT, Takayuki Murooka, Yutaka Kondo, ぐるぐる

0.3.0 (2024-05-31)
------------------
* Merge remote-tracking branch 'upstream/main' into rolling
* Merge pull request `#6 <https://github.com/youtalk/autoware_lanelet2_extension/issues/6>`_ from youtalk/autoware-msgs-migration
  feat: `autoware_msgs` migration
* feat(autoware_common): update and replace autoware_auto_msg
* feat(lanelet2_extension): replace autoware_auto_mapping_msg with autoware_map_msg (`#216 <https://github.com/youtalk/autoware_lanelet2_extension/issues/216>`_)
  * feat(lanelet2_extension): replace autoware-auto-mapping-msg to autoware-map-msg
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yutaka Kondo, cyn-liu, liu cui

0.2.0 (2024-05-07)
------------------
* refactor: add `autoware\_` prefix to package names and namespaces (`#3 <https://github.com/youtalk/autoware_lanelet2_extension/issues/3>`_)
  * fix readme
  * fix link
  * fix link
  * revert readme
  * wip
  * update package names
  * update package names
  * fix include
  * rename module
  * rename to autoware\_
  * fix depend
  * rename to autoware\_
  * Revert "wip"
  This reverts commit 8079660c318feace9d841aacd39a8945aa144cf7.
  ---------
* Contributors: Yutaka Kondo
