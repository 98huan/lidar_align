#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include <pcl/point_types.h>
#include <ros/ros.h>

#include "lidar_align/sensors.h"
#include <gps_common/GPSFix.h>
#include "rigid.h"
#define PCL_NO_PRECOMPILE

namespace lidar_align {

class Loader {
 public:
  struct Config {

//     int use_n_scans = 2000;    //08-05-20-04.bag
//     int use_n_scans = 320;    //08_filter.bag
    //int use_n_scans = 750;    // iplus_bag_S2_filter.bag
    int use_n_scans = 500;    // iplus_bag_S2_filter1.bag
  };

  Loader(const Config& config);

  void parsePointcloudMsg(const sensor_msgs::PointCloud2 msg,
                          LoaderPointcloud* pointcloud);

  bool loadPointcloudFromROSBag(const std::string& bag_path,
                                const Scan::Config& scan_config, Lidar* lidar);

  bool loadTformFromROSBag(const std::string& bag_path, Odom* odom);
  
  bool loadGpsFromROSBag(const std::string& bag_path, Odom* odom);

  bool loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom);

  static Config getConfig(ros::NodeHandle* nh);

 private:
  static bool getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                  Transform* T);

  Config config_;
};
}  // namespace lidar_align

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_align::PointAllFields,
    (float, x, x)(float, y, y)(float, z, z)(int32_t, time_offset_us,
                                            time_offset_us)(
        uint16_t, reflectivity, reflectivity)(uint16_t, intensity,
                                              intensity)(uint8_t, ring, ring))

#endif  // LIDAR_ALIGN_ALIGNER_H_
