^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_bag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.13 (2016-03-08)
-------------------
* RQT_BAG: Ensure monotonic clock publishing.
  Due to parallelism issues, a message can be published
  with a simulated timestamp in the past. This lead to
  undesired behaviors when using TF for example.
* Contributors: lsouchet

0.3.12 (2015-07-24)
-------------------
* Added step-by-step playback capability
* Contributors: Aaron Blasdel, sambrose

0.3.11 (2015-04-30)
-------------------
* fix viewer plugin relocation issue (`#306 <https://github.com/ros-visualization/rqt_common_plugins/issues/306>`_)

0.3.10 (2014-10-01)
-------------------
* fix topic type retrieval for multiple bag files (`#279 <https://github.com/ros-visualization/rqt_common_plugins/issues/279>`_)
* fix region_changed signal emission when no start/end stamps are set
* improve right-click menu
* improve popup management (`#280 <https://github.com/ros-visualization/rqt_common_plugins/issues/280>`_)
* implement recording of topic subsets
* sort the list of topics
* update plugin scripts to use full name to avoid future naming collisions

0.3.9 (2014-08-18)
------------------
* fix visibility with dark Qt theme (`#263 <https://github.com/ros-visualization/rqt_common_plugins/issues/263>`_)

0.3.8 (2014-07-15)
------------------

0.3.7 (2014-07-11)
------------------
* fix compatibility with Groovy, use queue_size for Python publishers only when available (`#243 <https://github.com/ros-visualization/rqt_common_plugins/issues/243>`_)
* use thread for loading bag files, emit region changed signal used by plotting plugin (`#239 <https://github.com/ros-visualization/rqt_common_plugins/issues/239>`_)
* export architecture_independent flag in package.xml (`#254 <https://github.com/ros-visualization/rqt_common_plugins/issues/254>`_)

0.3.6 (2014-06-02)
------------------
* fix closing and reopening topic views
* use queue_size for Python publishers

0.3.5 (2014-05-07)
------------------
* fix raw view not showing fields named 'msg' (`#226 <https://github.com/ros-visualization/rqt_common_plugins/issues/226>`_)

0.3.4 (2014-01-28)
------------------
* add option to publish clock tim from bag (`#204 <https://github.com/ros-visualization/rqt_common_plugins/issues/204>`_)

0.3.3 (2014-01-08)
------------------
* add groups for rqt plugins, renamed some plugins (`#167 <https://github.com/ros-visualization/rqt_common_plugins/issues/167>`_)
* fix high cpu load when idle (`#194 <https://github.com/ros-visualization/rqt_common_plugins/issues/194>`_)

0.3.2 (2013-10-14)
------------------

0.3.1 (2013-10-09)
------------------
* update rqt_bag plugin interface to work with qt_gui_core 0.2.18

0.3.0 (2013-08-28)
------------------
* fix rendering of icons on OS X (`ros-visualization/rqt#83 <https://github.com/ros-visualization/rqt/issues/83>`_)
* fix shutdown of plugin (`#31 <https://github.com/ros-visualization/rqt_common_plugins/issues/31>`_)
* fix saving parts of a bag (`#96 <https://github.com/ros-visualization/rqt_common_plugins/issues/96>`_)
* fix long topic names (`#114 <https://github.com/ros-visualization/rqt_common_plugins/issues/114>`_)
* fix zoom behavior (`#76 <https://github.com/ros-visualization/rqt_common_plugins/issues/76>`_)

0.2.17 (2013-07-04)
-------------------

0.2.16 (2013-04-09 13:33)
-------------------------

0.2.15 (2013-04-09 00:02)
-------------------------

0.2.14 (2013-03-14)
-------------------

0.2.13 (2013-03-11 22:14)
-------------------------

0.2.12 (2013-03-11 13:56)
-------------------------

0.2.11 (2013-03-08)
-------------------

0.2.10 (2013-01-22)
-------------------

0.2.9 (2013-01-17)
------------------
* Fix; skips time when resuming playback (`#5 <https://github.com/ros-visualization/rqt_common_plugins/issues/5>`_)
* Fix; timestamp printing issue (`#6 <https://github.com/ros-visualization/rqt_common_plugins/issues/6>`_)

0.2.8 (2013-01-11)
------------------
* expose command line arguments to rqt_bag script
* added fix to set play/pause button correctly when fastforwarding/rewinding, adjusted time headers to 0m00s instead of 0:00m for ease of reading
* support passing bagfiles on the command line (currently behind --args)

0.2.7 (2012-12-24)
------------------

0.2.6 (2012-12-23)
------------------

0.2.5 (2012-12-21 19:11)
------------------------

0.2.4 (2012-12-21 01:13)
------------------------

0.2.3 (2012-12-21 00:24)
------------------------

0.2.2 (2012-12-20 18:29)
------------------------

0.2.1 (2012-12-20 17:47)
------------------------

0.2.0 (2012-12-20 17:39)
------------------------
* first release of this package into Groovy
