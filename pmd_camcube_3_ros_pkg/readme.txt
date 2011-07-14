===> Installation

	===> File Setup
		
		1. libpmdaccess2.so file provided in PMDSDK2 should copied to a directory where the operating system can find it (e.g. /usr/local/lib)
		2. The include directory and plugins directory from PMDSDK2 should be in a directory where the operating system can find it (e.g. /usr/include/) or the path to these directories should be exported to the environment variable : $PMDDIR

	===> Usb device rules
		Please see the file usb-rule-setup.txt for instructions to set a new usb-dev rule for pmd-camcube

	===> Compiling the stack

		Keep the stack in a location know by ROS, i.e it should be in $ROS_PACKAGE_PATH. Then, 

		$ rosmake PMD_CAMCUBE_3

	===> Running the node
		
		$ rosrun PMD_CAMCUBE_3 pmd_camcube_3_node

	===> Visualization in rviz
		
		$ rosrun rviz rviz

	    Set the fixed_frame as "/tf_pmd_camcube"

	===> Add a Pointcloud2 topic to visualize the depthclouds. Three different set of points are published with following topic names:
		
		1. /depth_non_filtered : raw data from the pmd camera
		2. /depth_filtered : after applying statistical outlier detection from pcl
		3. /depth_outliers : the outliers found after applying the filter for statistical noise detection from pcl
		
		The filtered point clouds will be shown only when a filter option (AmplitudeFilter/StatisticalNoiseFilter) is turned on. The outliers will be published only for Statistical Noise Filter. The StatisticalNoiseFilter will make some delay between frames, due to required computaion time for removing outliers.
		
	===> To use the filter and change camera parameters, use dynamic_reconfigure from ros. To use it do
		
		$ rosrun dynamic_reconfigure reconfigure_gui 
		
		Select /pmd_camcube_3 to view the options available for modifications

Have FUN!!
		
