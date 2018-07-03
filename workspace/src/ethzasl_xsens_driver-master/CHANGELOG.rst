^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xsens_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2017-04-14)
------------------
* Add no_rotation_duration option
* Fix typo (`#39 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/39>`_)
* Fix gnss pvt parsing (`#37 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/37>`_)
* fix GetOptionFlags (`#34 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/34>`_)
* Contributors: Andersw88, Francis Colas

2.0.1 (2016-08-16)
------------------
* fix TimeReference member name
* Contributors: Francis Colas

2.0.0 (2016-08-02)
------------------
* support of mark iv devices (configuration and ROS node)
* remove gps_common dependency (for jade and kinetic)
* work in 16.04 with pyserial3
* proper message types for temperature, pressure, magnetic field and time
* better timeout management
* various bug fixes
* Contributors: CTU robot, Francis Colas, João Sousa, Konstantinos Chatzilygeroudis, Latitude OBC, Vincent Rousseau, fcolas, jotaemesousa

1.0.3 (2014-05-14)
------------------
* Inclusion of launch file
* Additions and fixes from PAL robotics
* Add local frame conversion for calibrated imu data (acc, gyr, mag)
* Contributors: Enrique Fernandez, Francis Colas, Paul Mathieu, Sam Pfeiffer

1.0.2 (2014-03-04)
------------------
* catkinized
* experimental support of mark 4 IMUs
* fixed scaling in DOP values
* adding publisher for full data as a string
* relative topic names

1.0.1 (2012-08-27)
------------------
* minor improvements
* naming cleanup
* Contributors: Benjamin Hitov, Francis Colas, Nikolaus Demmel, Stéphane Magnenat, fcolas
