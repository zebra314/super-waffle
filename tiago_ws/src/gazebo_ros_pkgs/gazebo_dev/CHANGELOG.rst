^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_dev
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.1 (2023-03-29)
------------------

4.0.0 (2021-07-28)
------------------
* Merge branch 'noetic-fixes' into 'gallium-devel'
  Noetic fixes
  See merge request common/gazebo_ros_pkgs!6
* Updated libgazebo dev version for noetic
* Contributors: Jordan Palacios

3.0.1 (2019-11-05)
------------------

3.0.0 (2019-09-09)
------------------
* added melodic API changes
* Contributors: Sai Kishor Kothakota

2.6.8 (2018-10-25)
------------------

2.6.7 (2018-01-12)
------------------

2.6.6 (2018-01-12)
------------------

2.6.5 (2018-01-11)
------------------
* removed changelogs and unified package versions
* Contributors: Hilario Tome

2.5.14 (2017-12-11)
-------------------
* Generate changelogs
* Contributors: Jose Luis Rivero

2.5.13 (2017-06-24)
-------------------
* Update version to make catkin_generate_release happy
* Update changelogs
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (#571)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------

2.5.10 (2017-03-03)
-------------------

2.5.9 (2017-02-20)
------------------

2.5.8 (2016-12-06)
------------------

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------

2.5.4 (2016-04-27)
------------------

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------

2.5.1 (2015-08-16 02:31)
------------------------

2.5.0 (2015-04-30)
------------------

2.4.9 (2015-08-16 01:30)
------------------------

2.4.8 (2015-03-17)
------------------

2.4.7 (2014-12-15)
------------------

2.4.6 (2014-09-01)
------------------

2.4.5 (2014-08-18)
------------------

2.4.4 (2014-07-18)
------------------

2.4.3 (2014-05-12)
------------------

2.4.2 (2014-03-27)
------------------

2.4.1 (2013-11-13 18:52)
------------------------

2.4.0 (2013-10-14)
------------------

2.3.5 (2014-03-26)
------------------

2.3.4 (2013-11-13 18:05)
------------------------

2.3.3 (2013-10-10)
------------------

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------

2.3.0 (2013-08-12)
------------------

2.2.1 (2013-07-29 18:02)
------------------------

2.2.0 (2013-07-29 13:55)
------------------------

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------

2.1.1 (2013-07-10)
------------------

2.1.0 (2013-06-27)
------------------

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------

2.0.0 (2013-06-18)
------------------
