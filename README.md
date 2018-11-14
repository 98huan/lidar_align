# lidar_align

## A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor
The method makes use of the property that pointclouds from lidars appear more 'crisp' when the calibration is correct. It does this as follows:
1) A transformation between the lidar and pose sensor is set.
2) The poses are used in combination with the above transformation to fuse all the lidar points into a single pointcloud.
3) The sum of the distance between each point and its nearest neighbour is found.
This process is repeated in an optimization that attempts to find the transformation that minimizes this distance.

## Installation

To install lidar_align, please install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu), [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

The following additional system dependencies are also required:
```
sudo apt-get install libnlopt-dev
```

## Estimation proceedure
For most systems the node can be run without tuning the parameters. By default two optimizations are performed, a rough global optimzation followed by a local refinement.

The node will load all messages of type `sensor_msgs/PointCloud2` from the given ROS bag for use as the lidar scans to process. The poses can either be given in the same bag file as `geometry_msgs/TransformStamped` messages or in a seperate CSV file that follows the format of [Maplab](https://github.com/ethz-asl/maplab).

## CSV format

| Column | Description |
|--:|:--|
1 | timestamp ns 
2 | vertex index (not used) 
3 | position x
4 | position y
5 | position z 
6 | orientation quaternion w 
7 | orientation quaternion x
8 | orientation quaternion y 
9 | orientation quaternion z 

Note that Maplab has two CSV exporters. This file-format is the same as produced by [exportPosesVelocitiesAndBiasesToCsv](https://github.com/ethz-asl/maplab/blob/master/console-plugins/vi-map-data-import-export-plugin/src/export-vertex-data.cc#L39) but differs from the output of [exportVerticesAndTracksToCsv](https://github.com/ethz-asl/maplab/blob/master/tools/csv-export/src/csv-export.cc#L35)

## Parameters
------

### Scan Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `min_point_distance` |  Minimum range a point can be from the lidar and still be included in the optimization. | 0.0 |
| `max_point_distance` |  Maximum range a point can be from the lidar and still be included in the optimization. | 100.0 |
| `keep_points_ratio` |  Ratio of points to use in the optimization (runtimes increase drastically as this is increased). | 0.01 |
| `min_return_intensity` | The minimum return intensity a point requires to be considered valid. | -1.0 |
| `motion_compensation` |  If the movement of the lidar during a scan should be compensated for. | true |
| `estimate_point_times` | Uses the angle of the points in combination with `lidar_rpm` and `clockwise_lidar` to estimate the time a point was taken at. | false |
| `lidar_rpm` | Spin rate of the lidar in rpm, only used with `estimate_point_times`. | 600 |
| `clockwise_lidar` | True if the lidar spins clockwise, false for anti-clockwise, only used with `estimate_point_times`. | false |

### IO Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `use_n_scans` |  Optimization will only be run on the first n scans of the dataset. | 2147483647 |
| `input_bag_path` |  Path of rosbag containing sensor_msgs::PointCloud2 messages from the lidar. | N/A  |
| `transforms_from_csv` | True to load scans from a csv file, false to load from the rosbag. | false |
| `input_csv_path` |  Path of csv generated by Maplab, giving poses of the system to calibrate to. | N/A |
| `output_pointcloud_path` |  If set, a fused pointcloud will be saved to this path as a ply when the calibartion finishes. | "" |
| `output_calibration_path` |  If set, a text document giving the final transform will be saved to this path when the calibration finishes. | "" |

### Alinger Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `local` |  If False a global optimzation will be performed and the result of this will be used in place of the `inital_guess` parameter. | false |
| `inital_guess` |  Inital guess to the calibration (x, y, z, rotation vector, time offset), only used if running in `local` mode. | [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] |
| `max_baseline` |  Maximum distance between sensors to consider during the global optimization stage. | 1.0 |
| `max_time_offset` |  Maximum time offset between sensor clocks in seconds. | 0.1 |
| `angular_range` | Search range in radians around the `inital_guess` during the local optimization stage. | 0.5 |
| `translational_range` | Search range around the `inital_guess` during the local optimization stage. | 0.5 |
| `max_evals` | Maximum number of function evaluations to run | 2000 |
| `xtol` | Tolerance of final solution | 0.000001 |
| `knn_batch_size` | Number of points to send to each thread when finding nearest points | 1000 |
| `knn_k` | Number of neighbours to consider in error function | 1 |
| `global_knn_max_dist` | Error between points is limited to this value during global optimization. | 1.0 |
| `local_knn_max_dist` | Error between points is limited to this value during local optimization. | 0.1 |
| `time_cal` | True to perform time offset calibration | true |
