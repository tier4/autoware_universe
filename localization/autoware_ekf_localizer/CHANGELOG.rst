^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ekf_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(localization_util)!: prefix package and namespace with autoware (`#8922 <https://github.com/autowarefoundation/autoware.universe/issues/8922>`_)
  add autoware prefix to localization_util
* refactor(ekf_localizer)!: prefix package and namespace with autoware (`#8888 <https://github.com/autowarefoundation/autoware.universe/issues/8888>`_)
  * import lanelet2_map_preprocessor
  * move headers to include/autoware/efk_localier
  ---------
* Contributors: Masaki Baba, Yutaka Kondo

Forthcoming
-----------
* fix(cpplint): include what you use - localization (`#9567 <https://github.com/tier4/autoware.universe/issues/9567>`_)
* fix(autoware_ekf_localizer): publish `processing_time_ms` (`#9443 <https://github.com/tier4/autoware.universe/issues/9443>`_)
  Fixed to publish processing_time_ms
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/tier4/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/tier4/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/tier4/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_ekf_localizer): remove `timer_tf\_` (`#9244 <https://github.com/tier4/autoware.universe/issues/9244>`_)
  Removed timer_tf\_
* refactor(localization_util)!: prefix package and namespace with autoware (`#8922 <https://github.com/tier4/autoware.universe/issues/8922>`_)
  add autoware prefix to localization_util
* refactor(ekf_localizer)!: prefix package and namespace with autoware (`#8888 <https://github.com/tier4/autoware.universe/issues/8888>`_)
  * import lanelet2_map_preprocessor
  * move headers to include/autoware/efk_localier
  ---------
* Contributors: Esteve Fernandez, M. Fatih Cırıt, Masaki Baba, SakodaShintaro, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
