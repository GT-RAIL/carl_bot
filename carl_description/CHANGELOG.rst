^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.26 (2015-04-17)
-------------------

0.0.25 (2015-04-10)
-------------------

0.0.24 (2015-04-06)
-------------------

0.0.23 (2015-04-03)
-------------------

0.0.22 (2015-04-03)
-------------------

0.0.21 (2015-03-31)
-------------------

0.0.20 (2015-03-31)
-------------------

0.0.19 (2015-03-27)
-------------------

0.0.18 (2015-03-27)
-------------------

0.0.17 (2015-03-24)
-------------------
* Added an end effector frame for the JACO
* Contributors: David Kent

0.0.16 (2015-02-17)
-------------------

0.0.15 (2015-02-10)
-------------------

0.0.14 (2015-02-06)
-------------------

0.0.13 (2015-01-21)
-------------------
* New camera calibrations
* Contributors: David Kent

0.0.12 (2015-01-19)
-------------------
* Orientation filter ignores accelerometer measurements when anything more than the gravity vector is detected
* Added joints to CARL urdf to allow for orientation adjustments from IMU data, implemented a static orientation correction from accelerometer data
* Contributors: David Kent

0.0.11 (2014-12-18)
-------------------
* travis fix
* fixed minifiy
* minify script
* no more pre-built file
* Updated transform between asus mount and asus; added carl_tools for miscellaneous packages, currently including calibration for the asus camera transform.
* Contributors: David Kent, Russell Toris

0.0.10 (2014-12-02)
-------------------
* reverted rviz
* fixed rear struts. also rebuild urdf
* rebuild urdf
* fixed front cover and back strut
* removed stls
* reverted to original rviz
* removed stls
* ran xacro
* removed screenshot
* fixed castor plate. added screenshot
* finished collision models. creative camera,mount,topplate,jacomount,camera lift
* caster
* servo arm, caster plate
* servo
* base, rear struts<
  >
* wheels, front cover, camera, hokuyo
* used cylinder for wheel collision model
* Contributors: Peter

0.0.9 (2014-10-22)
------------------

0.0.8 (2014-10-03)
------------------

0.0.7 (2014-09-22)
------------------

0.0.6 (2014-09-19)
------------------
* new URDF updated
* updated back cover collision model to extend upwards, allowing motion planning to treat the back struts as a solid object for increased safety
* Contributors: Russell Toris, dekent

0.0.5 (2014-09-10)
------------------
* added missing dep for carl_description
* Contributors: Russell Toris

0.0.4 (2014-09-02)
------------------
* start position of arm fixed
* fixed rotation of arm
* JACO arm added back
* rebuild of URDF
* moved to minified DAE
* re-wrote URDF to fix collision problems
* new collision test
* Contributors: Russell Toris

0.0.3 (2014-08-25)
------------------
* updated URDF
* added calibration files for the asus
* Contributors: David Kent, Russell Toris

0.0.2 (2014-08-18)
------------------
* rebuild CARL URDF
* urdf adjustments for point cloud accuracy
* recompiled URDF
* no longer publish openni TF tree
* testing camera URDF
* updated asus URDF
* Contributors: David Kent, Russell Toris

0.0.1 (2014-08-15)
------------------
* recompiled URDF
* revert changelogs
* changelog updated
* Updated Collision Model
* Added creative camera
* Added meshes for creative camera
* added front servo to joint state publisher
* Rotated right_rear_strut 180 degrees
* redid front cover
* redid front cover
* covers redone
* retextured back cover
* retextured back cover
* new asus model
* updated URDF
* revert asus
* asus test
* fixed size of metal texture
* minified new rear strut
* Added red color to rear_strut model itself
* minified new table plate
* New tiny plate
* moved originals back (materials reference broken)
* base plate test
* Set fixed frame to base_footprint
* Re-exported asus collada
* updated CARL urdf
* fixed install cmake bug
* minifyied materials
* robot URDF files installed
* recompiled URDF
* READMEs and Travis build
* minified XML in Collada modles
* cleanup of carl_description
* launch cleanup
* carl_description updated to use jaco_description instead of jaco_model, more carl_teleop cleanup
* Updated urdf to use jaco_description
* Rviz configuration adjustements
* Updated collision model
* Added new servo mount to model
* Added newly machined components to urdf
* Changed default camera angle
* Fixed issue with some camera transformes beign published multiple times by different nodes.
* Fixed base_footprint tf
* Added transform to base_footprint
* Fixed missing EOF new lines
* Added launch file for viewing the robot model.
* Changed initial pose of caster and repositioned cover.
* Fixed indentation
* Added asus xtion.
* Added new lines to end of files.
* Refactoring
* Refactoring
* Refactoring
* Refactoring.
* Refactoring.
* Refactoring.
* Renamed meshes to follow ROS conventions
* Renamed a couple meshes.
* Added lettering
* Updates to collision model
* Fixed transforms to match origins of new meshes
* Replaced STLs with Collada files.
* Removed unnused meshes
* Added back cover
* Added collision for caster
* Added front plate
* Added side walls
* Simplifications to collision model
* Switched to xacro format. Added jaco arm to model.
* Removed spaces from mesh file names
* Removed spaces from mesh file names
* Removed CAD models
* Fixed origin of STLs and reoriented axis of urdf
* Fixed collision origins
* Fixed origins for visualization.
* Fixed some origin alignment issues.
* Fixed rotation axis for camera tilt link
* Fixed rotation axis for camera tilt link
* Added more links
* Added meshes for wheels and base
* Started urdf
* Added STLs
* Added carl cad
* Added carl_description package.
* Contributors: =, Russell Toris, Steven Kordell, dekent
