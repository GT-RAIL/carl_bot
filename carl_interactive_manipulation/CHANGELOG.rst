^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_interactive_manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.34 (2016-02-23)
-------------------
* Update package.xml
* Updates for new rail_manipulation_msgs
* Contributors: David Kent

0.0.33 (2015-08-10)
-------------------

0.0.32 (2015-05-08)
-------------------
* Merge branch 'develop' of github.com:WPI-RAIL/carl_bot into develop
* Added a parameter for carl interactive manipulation to create object markers from segmented objects instead of recognized objects
* Contributors: David Kent

0.0.31 (2015-05-07)
-------------------

0.0.30 (2015-05-04)
-------------------
* Limited move while retracted feedback to publish at most every 5 seconds
* Contributors: David Kent

0.0.29 (2015-04-29)
-------------------
* Lowered error level on retracted arm movement feedback
* Added feedback on attempted arm move while arm is retracted
* Contributors: David Kent

0.0.28 (2015-04-27)
-------------------
* Switched remove functionality to remove objects from the recognized list instead of the segmented list
* Updated feedback message
* Contributors: David Kent

0.0.27 (2015-04-22)
-------------------
* Debug statement cleanup
* Connected IM with grasping and scene object attach/detach
* Contributors: David Kent

0.0.26 (2015-04-17)
-------------------

0.0.25 (2015-04-10)
-------------------

0.0.24 (2015-04-06)
-------------------

0.0.23 (2015-04-03)
-------------------
* removed pickup from IM client
* Contributors: Russell Toris

0.0.22 (2015-04-03)
-------------------

0.0.21 (2015-03-31)
-------------------
* removed recognition
* Contributors: Russell Toris

0.0.20 (2015-03-31)
-------------------
* renamed topic
* Contributors: Russell Toris

0.0.19 (2015-03-27)
-------------------

0.0.18 (2015-03-27)
-------------------
* Fixed a text placement bug on recognized markers
* Switched recognition functionality in interactive manipulation to use the new shared recognition actions
* Merge branch 'develop' of github.com:WPI-RAIL/carl_bot into develop
* Switched to the new ready/retract action provided by carl_moveit_common_actions
* Contributors: David Kent

0.0.17 (2015-03-24)
-------------------
* Added new recognition calls to carl_interactive_manipulation (note that pickup is still in progress and will not currently work)
* updted message
* Updated to reflect moving some messages from rail_segmentation to rail_manipulation_messages
* Switched im to use new rail_manipulation_msgs
* Contributors: David Kent, Russell Toris

0.0.16 (2015-02-17)
-------------------
* Update package.xml
* Merge pull request #25 from PeterMitrano/develop
  moved carl_parking to carl_interactive markers
* removed timeout and added intsallation
* Adjusted finger current thresholds
* Put a small threshold on executing an arm interactive marker recovery behavior to prevent it from activating on accidental clicks
* Adjustments for finger safety threshold.
* Lengthened distance of arm recovery behavior
* Documentation
* moved ros spin
* conformed to conventions
* Safety override for interactive marker control
* mend
* improved oop-ness
* removed unnessecary dependancy
* parking spots correctly appear
* improved oop structure. no statics
* oop works, but is really bad oop
* working on initializing static member
* made ParkingSpots class
* moved carl_parking to carl_interactive markers
* Contributors: David Kent, Peter, Russell Toris

0.0.15 (2015-02-10)
-------------------

0.0.14 (2015-02-06)
-------------------
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

0.0.9 (2014-10-22)
------------------
* Updated visualized segmented/recognized objects to work with the web visualizer
* Added dependency on message generation for rail_pick_and_place_msgs
* Interactive markers for objects now denote whether they have been recognized, and allow calls to pickup services
* Contributors: David Kent

0.0.8 (2014-10-03)
------------------
* adjusted point cloud visualization for segmented objects to be in keeping with rail_segmentation update
* adjusted retract position
* Revert "adjusted retract position"
  This reverts commit 01aa246c9bd241e9ad7b948c159059e407d7ceda.
* adjusted retract position
* Contributors: Russell Toris, dekent

0.0.7 (2014-09-22)
------------------
* carl_interactive_manipulation added
* Contributors: Russell Toris

0.0.6 (2014-09-19)
------------------

0.0.5 (2014-09-10)
------------------

0.0.4 (2014-09-02)
------------------

0.0.3 (2014-08-25)
------------------

0.0.2 (2014-08-18)
------------------

0.0.1 (2014-08-15)
------------------
