^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nerian_sp1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.3 (2017-10-18)
------------------
* Fixed file clash with new nerian_stereo package
* Contributors: Konstantin Schauwecker

1.6.2 (2017-05-30)
------------------
* Allow launch even if calibration file is not found
* Implemented upper limit for point cloud depth (max_depth parameter)
* Contributors: Konstantin Schauwecker

1.6.1 (2017-03-27)
------------------
* Updated libvisiontransfer to version 4.1.2
* Contributors: Konstantin Schauwecker

1.6.0 (2017-02-15)
------------------
* Updated SP1 software to version 4.1.0
* Script and launch file for downloading camera calibration
* Added optional execution delay
* Contributors: Konstantin Schauwecker

1.5.1 (2017-01-19)
------------------
* Added proper error reporting in case of exceptions
* Contributors: Konstantin Schauwecker

1.5.0 (2017-01-17)
------------------
* Switched to new sp1 software release 4.0.0
* Added example code for operation mode configuration to launch script
* Added example scripts for switching SP1 operation mode
* Separate topic for right image and bugfix for right image output
* Contributors: Konstantin Schauwecker

1.4.0 (2016-10-07)
------------------
* Updated to SP1 software release 3.0.0
* Removed automatic installation of spcom
* Handling of point cloud exceptions
* Contributors: Konstantin Schauwecker

1.3.3 (2016-05-17)
------------------
* Updated SP1 software release to version 2.1.6
* Contributors: Konstantin Schauwecker

1.3.2 (2016-05-09)
------------------
* Build fix for ROS kinetic
* Contributors: Konstantin Schauwecker

1.3.1 (2016-05-05)
------------------
* Added missing launch file to ROS package
* Contributors: Konstantin Schauwecker

1.3.0 (2016-03-18)
------------------
* Updated sp1 software release to version 2.1.5
* Support for changing q-matrix (caused by auto re-calibration)
* Contributors: Konstantin Schauwecker

1.2.2 (2016-02-12)
------------------
* Upgraded libvisiontransfer to version 2.1.2
* Contributors: Konstantin Schauwecker

1.2.1 (2016-01-12)
------------------
* Upgraded libvisiontransfer to version 2.1.1
* Contributors: Konstantin Schauwecker

1.2.0 (2015-11-23)
------------------
* Added current release candidate of libvisiontransfer 2.0.0
* Adaptations for libvisiontransfer 2.0.0
* Support transfer of Q matrix
* Contributors: Konstantin Schauwecker

1.1.2 (2015-10-05)
------------------
* Fixed bug that prevented conversion of point cloud message to PCL object
* Contributors: Konstantin Schauwecker

1.1.1 (2015-09-15)
------------------
* Updated to libvisiontransfer 1.0.2
* Installing libvisiontransfer headers
* Contributors: Konstantin Schauwecker

1.1.0 (2015-08-26)
------------------
* Cleaned-up example launch file
* Minor bugfixes
* Updated SP1 software package
* Publishing of camera information
* Optional disparity window
* Performance optimization
* Removed enable parameters
* Fixed ROS coordinate system
* Contributors: Konstantin Schauwecker

1.0.2 (2015-08-25)
------------------
* Minor fixes to build files
* Contributors: Konstantin Schauwecker

1.0.1 (2015-08-25)
------------------
* Initial release
* Contributors: Konstantin Schauwecker, nerian-vision
