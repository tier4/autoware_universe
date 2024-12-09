^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/tier4/autoware.universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/tier4/autoware.universe/issues/9569>`_)
* fix(traffic_light_roi_visualizer): show unknown results correctly (`#9467 <https://github.com/tier4/autoware.universe/issues/9467>`_)
  fix: show unknown results correctly
* chore(autoware_traffic_light_visualization): include opencv as system (`#9331 <https://github.com/tier4/autoware.universe/issues/9331>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/tier4/autoware.universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/tier4/autoware.universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/tier4/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/tier4/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, M. Fatih Cırıt, Masato Saeki, Ryohsuke Mitsudome, Tao Zhong, Yukinari Hisaki, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware.universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Masato Saeki, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(docs): fix documentation for traffic light visualization (`#8303 <https://github.com/autowarefoundation/autoware.universe/issues/8303>`_)
  fix docs traffic light visualization
* fix(autoware_traffic_light_visualization): fix to visualize correct color and shapes (`#8428 <https://github.com/autowarefoundation/autoware.universe/issues/8428>`_)
  fix(autoware_traffic_light_visualization): fix vialization to draw correct shapes
  Co-authored-by: Yi-Hsiang Fang (Vivid) <146902905+vividf@users.noreply.github.com>
* fix(traffic_light_visualization): fix funcArgNamesDifferent (`#8156 <https://github.com/autowarefoundation/autoware.universe/issues/8156>`_)
  fix:funcArgNamesDifferent
* fix(traffic_light_visualizer): remove cerr temporarily to avoid flooding logs (`#8294 <https://github.com/autowarefoundation/autoware.universe/issues/8294>`_)
  * fix(traffic_light_visualizer): remove cerr temporarily to avoid flooding logs
  * fix precommit
  * fix
  ---------
* fix(autoware_traffic_light_visualization): fix passedByValue (`#8241 <https://github.com/autowarefoundation/autoware.universe/issues/8241>`_)
  fix:passedByValue
* feat(traffic_light_roi_visualizer): add an option to use normal publisher instead of image tranport in traffic light roi visualizer (`#8157 <https://github.com/autowarefoundation/autoware.universe/issues/8157>`_)
  * apply new parameter schemes, set default parameters
  add an option to use normal publisher instead of image tranport in traffic light roi visualizer
  * small fix on default value
  ---------
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware.universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Kotaro Uetake, Taekjin LEE, Yutaka Kondo, Yuxuan Liu, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
