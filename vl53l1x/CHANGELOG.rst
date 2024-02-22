^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vl53l1x
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2021-08-17)
------------------
* Ported to ROS2
* CMAKE updated to ament format. Building of lib and i2c moved into new CmakeLists.txt inside /lib
* MeasurementData.msg explicilty uses std_msgs/Header
* i2c_setup now throws runtime_error instead of ROS errors directly for portability.
* Contributors: Mickey Li

1.0.0 (2021-01-28)
------------------
* Change licence to BSD-3-Clause

0.4.1 (2021-01-16)
------------------

0.4.0 (2019-12-05)
------------------
* Publish additional data of the measurements
* Publish measurement data even if range status is not valid
* Add status meanings to measurement data message file
* Implement min_signal and max_sigma parameters
* Add pass_statuses parameter for passing range statuses other than 0
* Print device info on startup
* Contributors: Alexey Rogachevskiy, Arthur Golubtsov, Oleg Kalachev
