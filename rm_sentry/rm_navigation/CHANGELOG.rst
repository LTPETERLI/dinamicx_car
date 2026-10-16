^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2023-07-26)
------------------
* Merge branch 'dev'
  # Conflicts:
  #	rm_fsm/test/spare_sentry.yaml
* Merge pull request `#85 <https://github.com/rm-controls/rm_sentry/issues/85>`_ from Edwinlinks/test-dev
  Finished the develop of pid local planner.
* Add env value for loading params of planner.
* Change the value of pid local planner.
* Change the name of launch files.
* Change params name.
* Modify config files of pid planner.
* Merge branch 'double_barrel' into double_barrel
* Modify error of pid local planner.
* Modify the wrong name.
* Add recovery for teb.
* Update controller frenquency for pid planner.
* Change navigation's structure, add pid_local_planner.
* Merge pull request `#81 <https://github.com/rm-controls/rm_sentry/issues/81>`_ from Edwinlinks/test-dev
  Add function for sentry to run out.
* Modify config and launch files.
* Merge pull request `#74 <https://github.com/rm-controls/rm_sentry/issues/74>`_ from Aung-xiao/combine_transform
  Combine goal_point transform function ,and modify a param of planner
* fixed some error of point transform,and modify the weight of obstacle
* add test_rmuc.png
* batter params for planning
* move better
* Merge pull request `#69 <https://github.com/rm-controls/rm_sentry/issues/69>`_ from Aung-xiao/test
  Modify params for better planning
* Merge pull request `#68 <https://github.com/rm-controls/rm_sentry/issues/68>`_ from Aung-xiao/dev
  Modify something of  shoot and gyro
* modify some params
* Merge pull request `#64 <https://github.com/rm-controls/rm_sentry/issues/64>`_ from Aung-xiao/navigation
  Optimize params for climb and navigation
* optimize params for climb and navigation
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/include/rm_fsm/state/common/auto.h
* Merge pull request `#57 <https://github.com/rm-controls/rm_sentry/issues/57>`_ from Aung-xiao/fast_lio_pr
  Optimize fast_lio params
* Merge pull request `#58 <https://github.com/rm-controls/rm_sentry/issues/58>`_ from Aung-xiao/climb
  Modify some params for climbing
* modify param of planner.yaml,delet useless octomap_mapping,rename segmentation
* modify some params for climbing
* optimize fast_lio params
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/src/state_machine.cpp
  #	rm_fsm/test/sentry.yaml
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/include/rm_fsm/common/subscriber.h
  #	rm_fsm/include/rm_fsm/state/cruise.h
  #	rm_fsm/test/sentry.yaml
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/test/sentry.yaml
* Contributors: Aung-xiao, Edwinlinks, NaHCO3bc, ye-luo-xi-tui, yezi

0.1.3 (2023-04-26)
------------------
* Merge branch 'dev'
* Merge pull request `#53 <https://github.com/rm-controls/rm_sentry/issues/53>`_ from Edwinlinks/yuan_dev
  Add time out for supply.
* Merge remote-tracking branch 'origin/yuan_dev' into yuan_dev
  # Conflicts:
  #	rm_fsm/test/sentry.yaml
* Add Statistical outlier removal and adjust planner param.
* Merge branch 'dev' into yuan_dev
* Merge pull request `#52 <https://github.com/rm-controls/rm_sentry/issues/52>`_ from Aung-xiao/dev(sentry.yaml)
  Modify goals and pcl min_z
* modify goals and pcl min_z
* Merge pull request `#49 <https://github.com/rm-controls/rm_sentry/issues/49>`_ from Aung-xiao/dev(planner_yaml)
  Modify min_obstacle_dist
* modify some param of planner
* Merge remote-tracking branch 'origin/navi-dev' into navi-dev
* Merge branch 'dev' into navi-dev
* Merge pull request `#42 <https://github.com/rm-controls/rm_sentry/issues/42>`_ from Edwinlinks/navi-dev
  Modify planner config.
* Modify planner config.
* Merge pull request `#41 <https://github.com/rm-controls/rm_sentry/issues/41>`_ from Aung-xiao/robot_status
  Modify hp topic and condition of hitted
* modify hp topic (not test)
* Merge pull request `#38 <https://github.com/rm-controls/rm_sentry/issues/38>`_ from Edwinlinks/navi-dev
  Add supply state for sentry
* Merge pull request `#39 <https://github.com/rm-controls/rm_sentry/issues/39>`_ from Aung-xiao/dev_pr
  Add rmul map
* add rmul map
* Add remap for fast lio.
* Merge pull request `#36 <https://github.com/rm-controls/rm_sentry/issues/36>`_ from Edwinlinks/navi-dev
  Modify config files after competion.
* modify config files of global planner.
* Modify config files after competion.
* Merge remote-tracking branch 'origin/defense_state' into defense_state
* Merge branch 'dev' into defense_state
* Merge pull request `#31 <https://github.com/rm-controls/rm_sentry/issues/31>`_ from Edwinlinks/time-out
  Modify some parameters of navigation.
* Modify some parameters of navigation.
* Merge pull request `#25 <https://github.com/rm-controls/rm_sentry/issues/25>`_ from Edwinlinks/time-out
  Update navigation parameters.
* Update navigation parameters.
* Contributors: Aung-xiao, Edwinlinks, peiyuan, ye-luo-xi-tui, yezi

0.1.2 (2023-03-22)
------------------
* Merge pull request `#16 <https://github.com/rm-controls/rm_sentry/issues/16>`_ from Edwinlinks/navigation-branch
  Modify navigation config.
* Delete unused plugin.
* Merge branch 'dev' into add_change_gyro
* Modify config of self filter,and reduce max sample.
* Update navigation config.
* Merge branch 'master' into dev
* Merge pull request `#10 <https://github.com/rm-controls/rm_sentry/issues/10>`_ from Edwinlinks/navigation-branch
  Update the first version of navigation.
* Update the first version of navigation.
* Contributors: Edwinlinks, NaHCO3bc, ye-luo-xi-tui, yezi
