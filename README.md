carl_bot [![Build Status](https://api.travis-ci.org/WPI-RAIL/carl_bot.png)](https://travis-ci.org/WPI-RAIL/carl_bot)
========

#### CARL (Crowdsourcing for Autonomous Robot Learning)
For full documentation, see [the ROS wiki](http://ros.org/wiki/carl_bot).

### Contributing

[carl_description](carl_description) includes both minified versions of the 3D Collada models as well as pre-compiled URDF files. To properly contribute, do the following:

 1. Re-minify any modified Collada files
   * `cd /path/to/carl_bot/carl_description/meshes/original`
   * `xmllint --noblanks my_modified_mesh.dae > ../my_modified_mesh.min.dae`
 1. Re-compile the modified URDF
   * `cd /path/to/carl_bot`
   * `rosrun xacro xacro carl_description/robots/carl.urdf.xacro > carl_description/robots/carl.urdf`

### License
carl_bot is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
