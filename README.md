# Imu Filter
Yet another IMU filter. Further [infos](https://mediatum.ub.tum.de/node?id=1452203) (chapter 4.1.1).

This filter does the following:
* `apply_transform`: transform IMU message from one tf frame to another and remove gyroscope offset
* `broadcast_transform`: autodetect tf transformation to make roll and pitch zero (planar assumption) when not moving
* `broadcast_transform`: autodetect gyroscope offset when not moving

## Limitations
Currently the following limitations are present:
* autocalibration only for roll and pitch angle (yaw angle is untouched)
* transformation assumes change in rotation (omega) to be zero



## other Filters
There are also some other filters out there (which I didn't find sufficient for our use case):
* [IMU Tools](https://wiki.ros.org/imu_tools)
* [IMU Pipeline](https://wiki.ros.org/imu_pipeline)
* [IMU Pipeline from dawonn](https://github.com/dawonn/imu_pipeline)

## REP 145
Usefull information: [Conventions for IMU Sensor Drivers](https://github.com/paulbovbel/rep/blob/master/rep-0145.rst)
