^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.16 (2015-02-17)
-------------------
* Changed teleop segment call to segment_auto
* Contributors: David Kent

0.0.15 (2015-02-10)
-------------------

0.0.14 (2015-02-06)
-------------------
* home/retract action input adjustment
* teleop adjustment for estop and home/retract with planning
* Set arm estop calls on the joystick controller
* Removed wait on home arm server so that the node can be started while running only CARL's basic functionality
* Switched home/retract actions to use motion planning, added home/retract/segment calls from joystick teleop
* Contributors: David Kent

0.0.13 (2015-01-21)
-------------------

0.0.12 (2015-01-19)
-------------------

0.0.11 (2014-12-18)
-------------------
* Fixed bug in IM where menu hover could cause pickup commands, added IM frontend and launch, updated joystick and keyboard teleop to use angular commands for finger commands so that fingers can be controlled even when the arm is in/near singularities, and updated metapackage manifest
* Contributors: David Kent

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
* Revert "Changes to base teleop to increase safety"
  This reverts commit ff0796d01abd7b8640db59a5e7789940692b9b4c.
* Changes to base teleop to increase safety
* Contributors: Brian Hetherman, Russell Toris

0.0.7 (2014-09-22)
------------------

0.0.6 (2014-09-19)
------------------

0.0.5 (2014-09-10)
------------------

0.0.4 (2014-09-02)
------------------

0.0.3 (2014-08-25)
------------------
* updates to teleop due to the bug in switching arm control modes
* Contributors: dekent

0.0.2 (2014-08-18)
------------------
* fixed missing build dep
* Contributors: Russell Toris

0.0.1 (2014-08-15)
------------------
* revert changelogs
* changelog updated
* bugfix for camera teleop controls
* velocity commands and teleop controls for the second camera
* adjustments to asus joint servo speed
* Servo velocity control and teleop initial commit
* possible fix for arm stopping during teleop mode switches
* carl_teleop cleanup
* carl_description updated to use jaco_description instead of jaco_model, more carl_teleop cleanup
* updated jaco_msgs to wpi_jaco_msgs, misc. cleanup
* added keyboard teleop for the JACO arm to CARL keyboard teleop
* Added arm control to CARL teleop, and analog controller functionality to the base teleop
* carl teleop cleanup
* Using relative namespace for teleop node and loading default parameters for covarience.
* Reversed angular commands
* Added ability to cancel navigation planning with the controller
* Joy teleop only publishes cmd_vel when deadman switch is pressed.
* cleanup of carl teleop nodes
* cleanup of carl joy teleop
* Added launch file for joystick teleop.
* Added joy to build dependencies
* Removed unnecessary function
* Now using right joystick for angular control
* Added fixes is response to code review: authorship credit, doxygen, and formatting.
* Added deadman swtich and boost button.
* Added keyboard teleop
* Joy Teleop Works
* Created cpp file for joystick teleop
* Created package for carl teleoperation.
* Contributors: Russell Toris, Steven Kordell, dekent
