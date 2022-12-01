#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "lidar_align/loader.h"
#include "lidar_align/transform.h"

#include <iostream> 
#include <stdio.h>
// #include <opencv2/core/core.hpp>  
// #include <opencv2/highgui/highgui.hpp>  
// #include <opencv2/opencv.hpp>
// using namespace cv;
using namespace std;


// typedef cv::Point2d point2d;
// vector<point2d> gps;

namespace lidar_align {
using namespace std;
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                               const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;
  
  return Eigen::Vector3d(x, y, z);
}

const Rigid3d ComputeLocalFrameFromLatLong(
  const double latitude, const double longitude) {
  
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond 
    rotation =  Eigen::AngleAxisd(DegToRad(latitude - 90.),
                         Eigen::Vector3d::UnitY()) * 
                         Eigen::AngleAxisd(DegToRad(-longitude),
                                           Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond 
    rotation2 =  Eigen::AngleAxisd(DegToRad(latitude - 90.),
                         Eigen::Vector3d::UnitY()) * 
                         Eigen::AngleAxisd(DegToRad(-longitude),
                                           Eigen::Vector3d::UnitZ());
//   dbg(rotation.toRotationMatrix());
//   dbg(rotation.toRotationMatrix().inverse().eulerAngles(2,1,0).transpose());
  return Rigid3d({rotation * -translation, rotation});
}

Loader::Loader(const Config& config) : config_(config) {}

Loader::Config Loader::getConfig(ros::NodeHandle* nh) {
  Loader::Config config;
  nh->getParam("use_LidarScan_number", config.use_n_scans); //从launch文件中读取使用的雷达帧数
  return config;
}

void Loader::parsePointcloudMsg(const sensor_msgs::PointCloud2 msg,
                                LoaderPointcloud* pointcloud) {
  bool has_timing = false;
  bool has_intensity = false;
//   for (const sensor_msgs::PointField& field : msg.fields) {
//     if (field.name == "timestamp") {
//       has_timing = true;
//     } else if (field.name == "intensity") {
//       has_intensity = true;
//     }
//   }
  static int intensity_type = -1;
//   dbg("start parsing..");
  for (const auto& field : msg.fields)
  {
//     dbg(field.name == "intensity");
    if (field.name == "intensity" && intensity_type == -1)
    {
      if(field.datatype == 7)
        intensity_type = 7;
      else
        intensity_type = 2;
      dbg("intensity_type: ", intensity_type);
    }
    break;
  }
  const string lidar_type = "hesai";
  int i = 0;
  double first_point_timestamp = 0;
  if (has_timing) {
    pcl::fromROSMsg(msg, *pointcloud);
    return;
  } else if (lidar_type == "hesai") {
    Pointcloud_TR raw_pointcloud;
    pcl::fromROSMsg(msg, raw_pointcloud);
    
    for (const PointXYZTR& raw_point : raw_pointcloud) {
      PointAllFields point;
      if(i == 0)
        first_point_timestamp = raw_point.timestamp;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;
      point.time_offset_us = (raw_point.timestamp - first_point_timestamp) * 1000000.0;
      point.intensity = 0;
      ++i;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
        continue;
      }

      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  } else {
    pcl::PointCloud<pcl::PointXYZ> raw_pointcloud;
    pcl::fromROSMsg(msg, raw_pointcloud);

    for (const pcl::PointXYZ& raw_point : raw_pointcloud) {
      PointAllFields point;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;
      point.intensity = 0.;
      point.time_offset_us = 0;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }

      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  }
}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path,
                                      const Scan::Config& scan_config,
                                      Lidar* lidar) {
  dbg(bag_path);
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back(std::string("/pointcloud_front"));
  rosbag::View view(bag, rosbag::TopicQuery(types));

  size_t scan_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::cout << " Loading scan: \e[1m" << scan_num++ << "\e[0m from ros bag"
              << '\r' << std::flush;

    LoaderPointcloud pointcloud;
    parsePointcloudMsg(*(m.instantiate<sensor_msgs::PointCloud2>()),
                       &pointcloud);

    lidar->addPointcloud(pointcloud, scan_config);

    if (lidar->getNumberOfScans() >= config_.use_n_scans) {
      break;
    }
  }
  if (lidar->getTotalPoints() == 0) {
    ROS_ERROR_STREAM(
        "No points were loaded, verify that the bag contains populated "
        "messages of type sensor_msgs/PointCloud2");
    return false;
  }

  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back(std::string("geometry_msgs/TransformStamped"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  size_t tform_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from ros bag" << '\r' << std::flush;

    geometry_msgs::TransformStamped transform_msg =
        *(m.instantiate<geometry_msgs::TransformStamped>());

    Timestamp stamp = transform_msg.header.stamp.sec * 1000000ll +
                      transform_msg.header.stamp.nsec / 1000ll;

    Transform T(Transform::Translation(transform_msg.transform.translation.x,
                                       transform_msg.transform.translation.y,
                                       transform_msg.transform.translation.z),
                Transform::Rotation(transform_msg.transform.rotation.w,
                                    transform_msg.transform.rotation.x,
                                    transform_msg.transform.rotation.y,
                                    transform_msg.transform.rotation.z));
    odom->addTransformData(stamp, T);
  }

  if (odom->empty()) {
    ROS_ERROR_STREAM("No odom messages found!");
    return false;
  }

  return true;
}

bool Loader::loadGpsFromROSBag(const std::string& bag_path, Odom* odom)
{
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back(std::string("/jzhw/gps/fix"));
  rosbag::View view(bag, rosbag::TopicQuery(types));

  size_t tform_num = 0;
  Rigid3d ecef_to_local_frame_;
  
  int num = 0;
  for (const rosbag::MessageInstance& m : view) {
    gps_common::GPSFix msg2 =
        *(m.instantiate<gps_common::GPSFix>());
    double dip = msg2.dip * M_PI / 180;   //msg2.dip是什么
    Eigen::Quaterniond north2gps(cos(dip / 2), 0, 0, sin(dip / 2));
    Rigid3d northGps2northBase(Eigen::Vector3d(0,0,0), north2gps);
    if(tform_num == 0)
    {
      ecef_to_local_frame_ = northGps2northBase * ComputeLocalFrameFromLatLong(msg2.latitude, msg2.longitude) ;   //ecef是gps的坐标系
      
    }
    Rigid3d fix2 = northGps2northBase * ComputeLocalFrameFromLatLong(msg2.latitude, msg2.longitude) ;
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from ros bag" << '\r' << std::flush;
    
    
    Rigid3d fix_pose1 = ecef_to_local_frame_ * fix2.inverse();
    Timestamp stamp = msg2.header.stamp.sec * 1000000ll +
                      msg2.header.stamp.nsec / 1000ll;
                      

    Transform T(Transform::Translation(fix_pose1.translation().x(),
                                       fix_pose1.translation().y(),
                                       fix_pose1.translation().z()),
                Transform::Rotation(fix_pose1.rotation().w(),
                                    fix_pose1.rotation().x(),
                                    fix_pose1.rotation().y(),
                                    fix_pose1.rotation().z()));
    odom->addTransformData(stamp, T);   
    
//在这里画出gps的轨迹
//     把gps的数据放入容器中
//     gps.push_back(point2d(T.translation().x(), T.translation().y()));
  }

  if (odom->empty()) {
    ROS_ERROR_STREAM("No odom messages found!");
    return false;
  }
//   Mat img(1200, 1000, CV_8UC3, cv::Scalar(255,255,255));    //创建1000×1000大小的像素快，原点在左上角
//   for (int i = 0; i < gps.size(); i++){
//     double gps_current_x = gps[i].x * 20 + 50;   //向右偏移
//     double gps_current_y = gps[i].y * 20 + 500;   //向下偏移
//     dbg(gps[i].x,gps[i].y );
//     circle(img, point2d(gps_current_x, gps_current_y),1, Scalar(255,0,0));    //画布、圆心、半径、颜色
//   }
//   imwrite(std::string(getenv("HOME")) + "/gps_path.png", img);
//   imwrite("/home/gps_path.png", img);
  
  return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) {
  std::ifstream file(csv_path, std::ifstream::in);

  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from csv file" << '\r' << std::flush;

    Timestamp stamp;
    Transform T;

    if (getNextCSVTransform(file, &stamp, &T)) {
      odom->addTransformData(stamp, T);
    }
  }

  return true;
}

// lots of potential failure cases not checked
bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                 Transform* T) {
  std::string line;
  std::getline(str, line);

  // ignore comment lines
  if (line[0] == '#') {
    return false;
  }

  std::stringstream line_stream(line);
  std::string cell;

  std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) {
    data.push_back(cell);
  }

  if (data.size() < 9) {
    return false;
  }

  constexpr size_t TIME = 0;
  constexpr size_t X = 2;
  constexpr size_t Y = 3;
  constexpr size_t Z = 4;
  constexpr size_t RW = 5;
  constexpr size_t RX = 6;
  constexpr size_t RY = 7;
  constexpr size_t RZ = 8;

  *stamp = std::stoll(data[TIME]) / 1000ll;
  *T = Transform(Transform::Translation(std::stod(data[X]), std::stod(data[Y]),
                                        std::stod(data[Z])),
                 Transform::Rotation(std::stod(data[RW]), std::stod(data[RX]),
                                     std::stod(data[RY]), std::stod(data[RZ])));

  return true;
}

}  // namespace lidar_align
