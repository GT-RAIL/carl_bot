^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
