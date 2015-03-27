^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.19 (2015-03-27)
-------------------
* added launch for carl grasp collection
* Contributors: Russell Toris

0.0.18 (2015-03-27)
-------------------

0.0.17 (2015-03-24)
-------------------
* limited output on phidgets initialization, added moveit stuff to carl_bringup launch
* Contributors: David Kent

0.0.16 (2015-02-17)
-------------------

0.0.15 (2015-02-10)
-------------------
* Updated base.launch so that CARL can be turned on while charging.
* Contributors: David Kent

0.0.14 (2015-02-06)
-------------------
* Main bringup launch file now launches basic safety (tipping prevention and arm over current warning)
* Contributors: David Kent

0.0.13 (2015-01-21)
-------------------
* New camera calibrations
* Contributors: David Kent

0.0.12 (2015-01-19)
-------------------
* Orientation filter ignores accelerometer measurements when anything more than the gravity vector is detected
* Contributors: David Kent

0.0.11 (2014-12-18)
-------------------

0.0.10 (2014-12-02)
-------------------
* deleted everything
* Parameter, topic, and launch file cleanup for consistency
* changes to make teleop safety able to toggle on and off with launch paramater
* Contributors: Brian Hetherman, David Kent, Peter

0.0.9 (2014-10-22)
------------------

0.0.8 (2014-10-03)
------------------

0.0.7 (2014-09-22)
------------------
* carl_interactive_manipulation added
* launches carl_interactive_manipulation instead of the jaco-only interactive markers
* Contributors: Russell Toris, dekent

0.0.6 (2014-09-19)
------------------
* interactive markers added to rviz display
* Contributors: Russell Toris

0.0.5 (2014-09-10)
------------------

0.0.4 (2014-09-02)
------------------
* view added to bringup
* Updated rgb camera calibration for the ASUS
* Contributors: David Kent, Russell Toris

0.0.3 (2014-08-25)
------------------
* added calibration files for the asus
* Contributors: David Kent

0.0.2 (2014-08-18)
------------------
* no longer publish openni TF tree
* Contributors: Russell Toris

0.0.1 (2014-08-15)
------------------
* revert changelogs
* changelog updated
* jaco arm interactive manipulation now launches on carl startup
* carl_dynamixel package cleanup
* cleanup of carl_description
* launch cleanup
* minor cleanup
* launch file launches teleop on startup
* updated run dependencies needed for launch files
* updated launch files
* increased rate of tf updating from joint_states
* joint_state_publisher now updates based on the jaco arm's published joint states
* Switched local planner
* Visual odometry disabled by default.
* Refactoring
* Parameter changes. Autonomous navigation significantly improved.
* Parameter changes.
* Using openni2 launch instead of camera node
* Using openni2 for asus
* Added visual odometry and efk node to launch.
* Parameter modifications.
* Removed asus for now
* Fixed frame for laser scan data
* Fixed included launch file path
* Fixed missing EOF new lines
* Created launch files for robot model, sensors, segway, and minimal bringup.
* Created carl_bringup package.
* Contributors: =, Russell Toris, Steven Kordell, dekent, spkordell
