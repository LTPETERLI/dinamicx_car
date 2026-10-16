^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_fsm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2023-07-26)
------------------
* Merge branch 'dev'
  # Conflicts:
  #	rm_fsm/test/spare_sentry.yaml
* Merge pull request `#85 <https://github.com/rm-controls/rm_sentry/issues/85>`_ from Edwinlinks/test-dev
  Finished the develop of pid local planner.
* Merge pull request `#89 <https://github.com/rm-controls/rm_sentry/issues/89>`_ from rm-controls/double_barrel
  Double barrel
* Update spare_sentry.yaml
* Merge pull request `#88 <https://github.com/rm-controls/rm_sentry/issues/88>`_ from NaHCO3bc/double_barrel
  Add params about duration in switch barrel
* Add params about duration in switch barrel
* Merge pull request `#87 <https://github.com/rm-controls/rm_sentry/issues/87>`_ from NaHCO3bc/double_barrel
  Update params of threshold
* Merge pull request `#86 <https://github.com/rm-controls/rm_sentry/issues/86>`_ from NaHCO3bc/update_params
  Add spare_sentry params
* Add spare_sentry params
* Update threshold.
* Merge pull request `#84 <https://github.com/rm-controls/rm_sentry/issues/84>`_ from NaHCO3bc/double_barrel
  Modify the function name and some params.
* Update params.
* Update params for desired position to remove motor gap
* Update params.
* Modify the function name of initialization.
* Add param of H key.
* Merge pull request `#56 <https://github.com/rm-controls/rm_sentry/issues/56>`_ from NaHCO3bc/double_barrel
  Add double_barrel_switcher for the double barrel
* Merge branch 'double_barrel' into double_barrel
* Merge pull request `#81 <https://github.com/rm-controls/rm_sentry/issues/81>`_ from Edwinlinks/test-dev
  Add function for sentry to run out.
* Add function for sentry to force run out.
* Modify config and launch files.
* Merge pull request `#80 <https://github.com/rm-controls/rm_sentry/issues/80>`_ from Edwinlinks/test-dev
  Update fsm's point and params.
* Update fsm's point and params.
* Merge pull request `#79 <https://github.com/rm-controls/rm_sentry/issues/79>`_ from Aung-xiao/debug_mode
  Add debug mode
* add debug mode
* Merge pull request `#78 <https://github.com/rm-controls/rm_sentry/issues/78>`_ from Aung-xiao/new_pr
  Modify for new rm_control ; cancel hit_gyro when outpost_hp >0.0; modify escape_state condition
* modify for new rm_control ; cancel hit_gyro when outpost_hp >0.0
* Merge pull request `#77 <https://github.com/rm-controls/rm_sentry/issues/77>`_ from Edwinlinks/conduct-dev
  Modify the action when sentry has reached the first point.
* Delete setGyro.
* Modify the action when sentry has reached the first point.
* Merge pull request `#76 <https://github.com/rm-controls/rm_sentry/issues/76>`_ from Edwinlinks/conduct-dev
  Modify the action when sentry has reached goal.
* Modify the action when sentry reached goal.
* Merge pull request `#75 <https://github.com/rm-controls/rm_sentry/issues/75>`_ from Edwinlinks/shoot-testing
  Add urgent hp for making sentry escape.
* Merge pull request `#74 <https://github.com/rm-controls/rm_sentry/issues/74>`_ from Aung-xiao/combine_transform
  Combine goal_point transform function ,and modify a param of planner
* Add urgent hp for making sentry escape.
* test pass
* Merge pull request `#73 <https://github.com/rm-controls/rm_sentry/issues/73>`_ from Edwinlinks/shoot-testing
  Modify cruise state when referee is not online.
* Change values to auto class.
* Change type from SUCCESS to ACTIVE.
* Change the way of defense when referee is not online.
* Modify the way of judge time out, and the params of fsm.
* Add setReached when entry defense.
* Modify cruise state when referee is not online.
* not test
* fixed some error of point transform,and modify the weight of obstacle
* Merge pull request `#71 <https://github.com/rm-controls/rm_sentry/issues/71>`_ from Aung-xiao/send_path
  Sending path to minimap
* modify params name and delete useless param
* modify some params name
* modify some params name
* combine start_pose and delta_pose transform
* use minimapToWorld() from conduct state
* rename the function and move to state_machine.cpp
* modify some params of sentry.yaml
* Merge pull request `#70 <https://github.com/rm-controls/rm_sentry/issues/70>`_ from Edwinlinks/gryo_test
  Modify Defense state.
* CHange PoseStampedType to PoseStampedIterator.
* Modify name of iterator.
* Modify defense state;s value and format.
* at least can use (test pass)
* can send map to referee
* Adding set false of client_map_update\_.
* Modify Defense state.
* modify some error and optimize the num of delta_xy (not tested in server)
* add map_sentry_data (not test)
* batter params for planning
* Merge pull request `#69 <https://github.com/rm-controls/rm_sentry/issues/69>`_ from Aung-xiao/test
  Modify params for better planning
* longer time for waiting game start and planning
* modify some params of gyro; cancel gyro before game start
* Merge pull request `#68 <https://github.com/rm-controls/rm_sentry/issues/68>`_ from Aung-xiao/dev
  Modify something of  shoot and gyro
* modify some params
* Merge pull request `#67 <https://github.com/rm-controls/rm_sentry/issues/67>`_ from Edwinlinks/fsm-conduct-re
  Add planning when outpost active.
* Change max_yaw_vel
* Modify patrols list.
* Delete outpost_risk_hp from state.
* Add planning when outpost active.
* Merge pull request `#66 <https://github.com/rm-controls/rm_sentry/issues/66>`_ from Edwinlinks/fsm-conduct-re
  Add if_gyro for defense.
* Add if_gyro for defense.
* Merge pull request `#65 <https://github.com/rm-controls/rm_sentry/issues/65>`_ from Aung-xiao/hit_delay
  Add track delay
* move entry_track\_ from subscriber.h to combat.h
* rename the track params
* add track_duration in yaml
* add track delay
* Merge pull request `#64 <https://github.com/rm-controls/rm_sentry/issues/64>`_ from Aung-xiao/navigation
  Optimize params for climb and navigation
* optimize params for climb and navigation
* Merge pull request `#63 <https://github.com/rm-controls/rm_sentry/issues/63>`_ from Edwinlinks/fsm-conduct-re
  Add defense state, add transform judements of outpost hp.
* Modify param's name.
* Modify transform of combat state.
* Add cooling_threshold.
* Delete the shooter command sender in union command sender.
* Delete ros info in state_machine.cpp.
* Modify 300 to 500.
* Add defense state, add transform judements of outpost hp.
* Move the doSwitch judgement into command sender from state machine.
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/include/rm_fsm/state/common/auto.h
* Update params.
* Move the logic of switch barrel into sendCommand in double barrel command sender which is moved into union command sender from subscriber.
* Merge pull request `#60 <https://github.com/rm-controls/rm_sentry/issues/60>`_ from Edwinlinks/fsm-conduct-re
  Modify the name of patrol lists.
* Modify the sendCommand by using the sendCommand function in the DoubleBarrelCommandSender class.
* Modify the name of patrol lists.
* Merge pull request `#59 <https://github.com/rm-controls/rm_sentry/issues/59>`_ from Edwinlinks/fsm-conduct-re
  reconstruct fsm
* Add sendChassisCmd of auto in combat.
* Change the logic of cruise and conduct.
* Modify the name of class and the function of send shooter command.
* Change the naming of values in conduct state, delete unused values.
* Change the way of switching point group, delete supply state.
* Merge pull request `#55 <https://github.com/rm-controls/rm_sentry/issues/55>`_ from Aung-xiao/communicate
  Add conduct state
* delete params of key in yaml; delete useless ROS_INFO;and blue's tf transform test pass
* delete useless params
* fixed the error of cruise's condition
* add blue's tf transform (not test)
* at least red's transform is right……
* Move the double barrel switcher into command sender from fsm.
* Modify the type of barrel command sender and move the logic of switching barrel into the switcher class from state class.
* add robot_color switch (not test)
* modify clientMapSendDataCallback()
* add map2referee_map (not test)
* Add to set switch_done\_ value.
* modify comments
* test passed
* modify param of keyboard(not test)
* delete ros_info
* add conduct state (tested passed)
* Solve the bug in initBarrelType and modify the threshold of the difference between cooling limit and cooling heat.
* simply patrol_list (not test)
* Add judgement of switch barrel in the manual and combat state to replace the old version of sendShooterCmd in state machine.
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/src/state_machine.cpp
  #	rm_fsm/test/sentry.yaml
* Modify the function name in state machine.
* Update params.
* Modify the threshold of the difference between cooling limit and cooling heat.
* Add a subscriber to subscribe the status of trigger and move the realization of switching barrel from state machine into double_barrel_switcher class.
* Solve a error about getting param.
* Add a judgement to set switch_barrel\_ false.
* First edition of communicate with sentry
* Modify the sendShooterCommand function in auto state.
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/include/rm_fsm/common/subscriber.h
  #	rm_fsm/include/rm_fsm/state/cruise.h
  #	rm_fsm/test/sentry.yaml
* Merge branch 'dev' into double_barrel
  # Conflicts:
  #	rm_fsm/test/sentry.yaml
* Parameterize the threshold to restart push.
* Delete the useless file.
* Update params.
* Move the is_double_barrel param form union_command_sender to double_barrel_switcher.
* Send shooter command in state machine and manual to replace send shooter command in switcher while switching barrel.
* Modify the logic of restarting push and some details.
* Modify the judgement to switch barrel and a detail.
* Move the do restart push algorithm into DoubleBarrelSwitcher class and modify some function name.
* Update params and add a judgement to restart pushing.
* Simplify the doSwitch function.
* Modify the param's name and uses it into cruise and combat state.
* Add two barrels' params.
* Solve the bug that the barrel_type does not update when the barrel is calibrated and lead to the barrel does not switch.
* Solve the sendCommand conflict and modify the logic of doSwitch.
* Add a transition function and modify the judgement about entering switch barrel.
* Modify the namespace.
* Modify the id order.
* Add a parameter called double_barrel\_ to distinguish between one or two barrels
* Modify the topics' name which are subscribed.
* Merge branch 'dev' into double_barrel
* Modify namespace.
* Add stop and restart push mode function.
* Add switch barrel in state machine.
* Add double_barrel_switcher's update function in callback.
* Add double_barrel_switcher.h
* Contributors: Aung-xiao, BruceLannn, Edwinlinks, NaHCO3bc, ye-luo-xi-tui, yezi

0.1.3 (2023-04-26)
------------------
* Merge branch 'dev'
* Merge pull request `#54 <https://github.com/rm-controls/rm_sentry/issues/54>`_ from Edwinlinks/yuan_dev
  Reconstruct state class, and modify the way of command sending.
* Reconstruct state class, and modify the way of command sending.
* Merge pull request `#53 <https://github.com/rm-controls/rm_sentry/issues/53>`_ from Edwinlinks/yuan_dev
  Add time out for supply.
* Add Statistical outlier removal and adjust planner param.
* Merge branch 'dev' into yuan_dev
* Delete ROS_INFO.
* Add time out for supply.
* Merge pull request `#52 <https://github.com/rm-controls/rm_sentry/issues/52>`_ from Aung-xiao/dev(sentry.yaml)
  Modify goals and pcl min_z
* modify goals and pcl min_z
* delete robot_id
* Merge pull request `#51 <https://github.com/rm-controls/rm_sentry/issues/51>`_ from Aung-xiao/dev(sentry.yaml)
  Delete robot_id
* Merge pull request `#50 <https://github.com/rm-controls/rm_sentry/issues/50>`_ from Aung-xiao/dev(sentry.yaml)
  Modify some params of sentry.yaml
* add safe_speed_limit param
* modify some params of sentry.yaml
* Merge pull request `#48 <https://github.com/rm-controls/rm_sentry/issues/48>`_ from Aung-xiao/dev(cruise)
  Fix the logical of staying at first point.
* fix the logical of staying at first point.
* Merge pull request `#47 <https://github.com/rm-controls/rm_sentry/issues/47>`_ from Edwinlinks/navi-dev
  Add time out for staying at fixed point.
* Add set gryo ,when staying at first point,
* Add time out for staying at fixed point.
* Merge pull request `#46 <https://github.com/rm-controls/rm_sentry/issues/46>`_ from Edwinlinks/navi-dev
  Change set time from entry to transition.
* Change set time from entry to transition.
* Merge pull request `#45 <https://github.com/rm-controls/rm_sentry/issues/45>`_ from Edwinlinks/navi-dev
  Change the range of generating number.
* Merge remote-tracking branch 'origin/navi-dev' into navi-dev
* Change the range of generating number.
* Merge pull request `#44 <https://github.com/rm-controls/rm_sentry/issues/44>`_ from Edwinlinks/navi-dev
  Modify the way of generating number.
* Merge branch 'dev' into navi-dev
* Modify the way of generating number.
* Merge pull request `#43 <https://github.com/rm-controls/rm_sentry/issues/43>`_ from Edwinlinks/navi-dev
  Change cruise way for sentry, add game start judgement.
* Change cruise way for sentry, add game start judgement.
* Merge pull request `#41 <https://github.com/rm-controls/rm_sentry/issues/41>`_ from Aung-xiao/robot_status
  Modify hp topic and condition of hitted
* delete useless time_arg
* add game_start signal
* modify sentry.yaml
* fix error and modify the condition of "hitted"
* modify hp topic (not test)
* Merge pull request `#40 <https://github.com/rm-controls/rm_sentry/issues/40>`_ from Edwinlinks/navi-dev
  Modify supply state transition.
* Change send command function.
* Modify robotHpUpdate judgement.
* Add header for game robot hp.
* Modify supply state transition.
* Merge pull request `#38 <https://github.com/rm-controls/rm_sentry/issues/38>`_ from Edwinlinks/navi-dev
  Add supply state for sentry
* Modify the transition function.
* Add supply state for sentry.
* Merge pull request `#37 <https://github.com/rm-controls/rm_sentry/issues/37>`_ from Edwinlinks/navi-dev
  Change the command source frame of cruise state.
* Change callServer to switchTarget.
* Change the naming of switch target.
* Change the param of sendChassisCommand.
* Change the command source frame of cruise state.
* Merge pull request `#36 <https://github.com/rm-controls/rm_sentry/issues/36>`_ from Edwinlinks/navi-dev
  Modify config files after competion.
* Modify config files after competion.
* Merge pull request `#33 <https://github.com/rm-controls/rm_sentry/issues/33>`_ from NaHCO3bc/move_when_the_game_start
  Let the sentry enter cruise state when the game start.
* Merge pull request `#34 <https://github.com/rm-controls/rm_sentry/issues/34>`_ from Aung-xiao/dev_pr
  Fixed the error of heat limit
* Merge branch 'dev' into move_when_the_game_start
* modify some params of shoot
* fixed the error of heat limit
* Solve the namespace error.
* Delete the test method.
* Add a test method.
* Merge pull request `#32 <https://github.com/rm-controls/rm_sentry/issues/32>`_ from Edwinlinks/fsm-dev
  Add some details and modify ways of sending command.
* Merge branch 'dev' into fsm-dev
* Merge pull request `#29 <https://github.com/rm-controls/rm_sentry/issues/29>`_ from Aung-xiao/defense_state
  Defense state
* modify the name of latest msgs
* modify the name of the latest hurt msgs
* modify last_hurt_time condition
* Add a judgement to enter cruise state from manual state in the beginning of match.
* Change the judgement of cancel goal.
* Add some details and modify ways of sending command.
* delete chassis->sendcommand()
* modify shooter mode in combat state
* combine cruise_gyro\_ and hitted\_
* Merge remote-tracking branch 'origin/defense_state' into defense_state
* modify some irregular core
* Merge branch 'dev' into defense_state
* defense state test passed
* save
* adapt to new rm_control (not tested)
* modify manual
* Merge pull request `#31 <https://github.com/rm-controls/rm_sentry/issues/31>`_ from Edwinlinks/time-out
  Modify some parameters of navigation.
* Modify wrong parameters.
* Modify some parameters of fsm.
* add hitted interval to yaml
* modify .sm and add hitted interval (not test)
* Merge pull request `#28 <https://github.com/rm-controls/rm_sentry/issues/28>`_ from NaHCO3bc/fix_gyro_bug
  Solve the error which happen after the gyro mode was deleted.
* add gyro duration in yaml (not tested yet ,please don't merge)
* move "hit detect" from combat to cruise
* add callService() and modify RobotHurtCallback
* Modify the judgement in setting chassis command.
* Change is_gyro to a local variable.
* add defense state (tested pass)
* Fix in missing setting is_gyro\_ false.
* Solve the error which happen after the gyro mode was deleted.
* modify some error of defense state
* add defense state
* Merge pull request `#26 <https://github.com/rm-controls/rm_sentry/issues/26>`_ from Edwinlinks/time-out
  Add calling service for switch enemy color.
* Add calling service for switch enemy color.
* Merge pull request `#24 <https://github.com/rm-controls/rm_sentry/issues/24>`_ from Edwinlinks/time-out
  Add time out judgement for cruise.
* Add time out judgement for cruise.
* Contributors: Aung-xiao, Edwinlinks, NaHCO3bc, peiyuan, ye-luo-xi-tui, yezi

0.1.2 (2023-03-22)
------------------
* Merge pull request `#23 <https://github.com/rm-controls/rm_sentry/issues/23>`_ from Edwinlinks/modify-fsm
* Add gyro speed variation way and add some parameters for standby gyro.
* Merge pull request `#18 <https://github.com/rm-controls/rm_sentry/issues/18>`_ from Aung-xiao/new_dev
  Add combat state
* replace rm_common/union.h to rm_fsm/union.h
* delete useless .h
* add combat state
* Merge pull request `#14 <https://github.com/rm-controls/rm_sentry/issues/14>`_ from rm-controls/revert-13-dev
  Revert "Add shoot state"
* Revert "Add shoot state"
* Merge pull request `#13 <https://github.com/rm-controls/rm_sentry/issues/13>`_ from Aung-xiao/dev
  Add shoot state
* Merge pull request `#11 <https://github.com/rm-controls/rm_sentry/issues/11>`_ from NaHCO3bc/add_change_gyro
  Add gyro mode in cruise state.
* Delete the way to get gyro_rotate_reduction.
* modify some error of state_machine.cpp
* modify some error of .sm
* Merge branch 'dev' into add_change_gyro
* Modify the value to setting the z axis angular velocity.
* add cruise_cancel_goal()
* modify the name of shoot state and add gyro_speed\_ arg
* add shoot state
* Modify the param name of entering gyro in cruise mode.
* Merge pull request `#12 <https://github.com/rm-controls/rm_sentry/issues/12>`_ from Aung-xiao/dev
  modified the random function
* modified the random function
* Simplify the code in cruise mode.
* Delete the requirement of track data.
* Modify the requirement of entering the gyro mode and the value to setting the z axis angular velocity.
* Code style.
* Modify the param name of entering gyro mode.
* Merge branch 'master' into dev
* Add gyro mode in cruise state.
* ci: fix ci build
* ci: remove unused package
* Merge pull request `#10 <https://github.com/rm-controls/rm_sentry/issues/10>`_ from Edwinlinks/navigation-branch
  Update the first version of navigation.
* Update the first version of navigation.
* Merge pull request `#8 <https://github.com/rm-controls/rm_sentry/issues/8>`_ from rm-controls/master
  Merge master to dev
* Merge remote-tracking branch 'origin/visual_test' into visual_test
* ci: fix ci build
* Merge branch 'dev' into visual_test
* ci: fix smc build again
* chore: bump smc version
* build: add missing dependency
* build: add `smclib`
* build: add `move_base_msgs`
* build: move jre into docker
* build: add java to dependencies
* lint: fix format
* build: fix SMC build
* Contributors: Aung-xiao, Edwinlinks, NaHCO3bc, StarHeart, StarHeartHunt, peiyuan, ye-luo-xi-tui, yezi

0.1.1 (2023-03-01)
------------------
* Merge pull request `#3 <https://github.com/rm-controls/rm_sentry/issues/3>`_ from Edwinlinks/renew-rm-fsm
  Update new version of rm fsm
* Modify naming of state.
* Change pubilshManual() to pubilshFsm().
* Change manual_to_referee_data to fsm_to_referee.
* Modify the gimbal and chassis code of cruise.
* Modify fsm parameter and delete getting robot type.
* Modify random generate way.
* Modify the wrong topic name.
* Delete lower_gimbal_des_error\_.
* Change the way to generate rand numbers.
* Make config.yaml and load.launch under test folder.
* Rename files name with code style.
* Delete used part of launch file.
* Change the order of sensor_msgs.
* Add smc.java package.
* Modify chassis and vel context in config files.
* Modify sentry fsm config files.
* Modify the order of dependence package.
* Modify smc file and add call back function in Subscriber.h, add publish referee data.
* Change mbf to move_base.
* Change the structure of rm_fsm, rename side command sender.
* Add new version of rm_fsm.
* Contributors: Edwinlinks, ye-luo-xi-tui
