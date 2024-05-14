^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package play_motion_builder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'fix/nan-value-bug' into 'master'
  Fix/nan value bug
  See merge request apps/play_motion_builder!4
* remove unused import
* move joint_state pub from test_motion_model.cpp to test_motion_model.test
* add listener to joint_states message pub and warn if nullptr
* remove dup code, add some comment and avoid breaking old behavior
* add joint_state pub to test_motion_model
* update test file with appropriate values
* fix delay issue when changing group
* fix nan values for group but still too slow
* fix add joint nan value
* Contributors: davidfernandez, thomasung

1.1.0 (2023-11-15)
------------------
* Merge branch 'prevent_groups_with_extras' into 'master'
  Groups with joints in the extras list are ignored
  See merge request apps/play_motion_builder!2
* Groups with joints in the extras list are ignored
* Contributors: davidfernandez

1.0.3 (2021-08-05)
------------------
* Merge branch 'unordered_groups' into 'master'
  Allow groups to be loaded in any order
  See merge request apps/play_motion_builder!1
* Allow groups to be loaded in any order
* Contributors: davidfernandez, victor

1.0.2 (2020-10-22)
------------------
* Fix dependencies
* Contributors: davidfernandez

1.0.1 (2020-10-21)
------------------
* REname projecto to comply with naming scheme
* Contributors: davidfernandez

1.0.0 (2020-10-15)
------------------
* Documentation
* Initial commit
* Contributors: davidfernandez
