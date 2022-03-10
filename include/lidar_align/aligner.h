#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ros/ros.h>
#include <future>
#include <limits>
#include <nlopt.hpp>
#include <iostream>
#include <string>

#include "lidar_align/sensors.h"

namespace lidar_align {
  std::string initial_pose;
  std::stringstream strStr;

class Aligner {
 public:
  
  struct Config {
//     bool local = false;  //先进行全局优化，再进行局部优化
    bool local = true;    //只进行局部优化
    std::vector<double> inital_guess{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //外参初值在launch文件中改
    double max_time_offset = 0.1;   //default 最大的时间对准偏差
    double angular_range = 0.5;     //default 在初始估计位置的搜索范围（弧度）角度偏移上下限 +-0.5弧度
//     double angular_range = 0.262;   //15度     
    double translation_range = 1.0; //default 在初始估计位置的搜索范围（位置）位置偏移上下限 +-1米
    //double translation_range = 0.5; //在初始估计位置的搜索范围（位置）位置偏移上下限 +-0.5米
    
    double max_evals = 0;     //default 最大的评估次数,在launch文件中改!
    double xtol = 0.0001;       //default x的容差

    int knn_batch_size = 1000;  //default 点的偏移数量 被用于寻找最近邻点时
    //int knn_batch_size = 500;  //点的偏移数量 被用于寻找最近邻点时
    int knn_k = 1;              //default 寻找最近邻点的个数, 程序用了自加1了  因为在一个点云里面找其中一个点的最近邻一个是自己一个才是真的最近邻
    float local_knn_max_dist = 0.1; //default 局部优化的时候 最近邻点的最大距离
   // float local_knn_max_dist = 0.05; //局部优化的时候 最近邻点的最大距离
    float global_knn_max_dist = 1.0;  //default 全局优化的时候 最近邻点的最大距离
    
    bool time_cal = true;       //是否执行时间补充计算  就是在找每个点的位姿时根据时间对里程计插值

    std::string output_pointcloud_path = "";  //点云文件保存路径
    std::string output_calibration_path = ""; //校准信息文件保存路径
  };

  struct OptData {  //optimize data
    Lidar* lidar;
    Odom* odom;
    Aligner* aligner;
    bool time_cal;
  };

  Aligner(const Config& config);

  static Config getConfig(ros::NodeHandle* nh);

  void lidarOdomTransform(Lidar* lidar, Odom* odom);    //主要求解函数,进行优化求解6个位姿参数

 private:
  void optimize(const std::vector<double>& lb, const std::vector<double>& ub,
                OptData* opt_data, std::vector<double>* x);

  std::string generateCalibrationString(const Transform& T,
                                        const double time_offset);

  static float kNNError(
      const pcl::KdTreeFLANN<Point>& kdtree, const Pointcloud& pointcloud,
      const size_t k, const float max_dist, const size_t start_idx = 0,
      const size_t end_idx = std::numeric_limits<size_t>::max());

  float lidarOdomKNNError(const Pointcloud& base_pointcloud,
                          const Pointcloud& combined_pointcloud) const;

  float lidarOdomKNNError(const Lidar& lidar) const;

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data);
  
  Config config_;
};

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
