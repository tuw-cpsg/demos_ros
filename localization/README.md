Localization
============

Required/Used Packages
----------------------

* [general-ros-modules/pioneer_teleop](https://github.com/tuw-cpsg/general-ros-modules)
* [gmapping](http://wiki.ros.org/gmapping)
* [map-server](http://wiki.ros.org/map_server)
* navigation ([acml](http://wiki.ros.org/amcl))


Create a Map
------------

Start tele-operation of rover, gmapping and rviz.

```bash
roslaunch localization create_map.launch notebook:=<hostname>
rviz -d config/create_map.rviz
```

Drive around until you're satisfied with the map visible in rviz. Then save it
with:

```bash
rosrun map_server map_saver -f <mapname>
```


Localize
--------

Start tele-operation, amcl with default map `cpslab-empty.yaml` and rviz.

```bash
roslaunch localization localize.launch notebook:=<hostname>
rviz -d config/localize.rviz
```

Set initial pose in rviz with the button "2D Pose Estimate".
