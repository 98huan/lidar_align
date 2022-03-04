#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ros/ros.h>
#include <future>
#include <limits>
#include <nlopt.hpp>

#include "lidar_align/sensors.h"

namespace lidar_align {

class Aligner {
 public:
  struct Config {
//     bool local = false;  //先进行全局优化，再进行局部优化
    bool local = true;    //只进行局部优化
    
//     std::vector<double> inital_guess{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //初值为0
//     std::vector<double> inital_guess{-0.017, 1.008, 0.0, -0.0, -0.0, 2.345, 0.0};  //lidar和gps机械xyzRPY, z、rx、ry设为0
//     std::vector<double> inital_guess{0.022177, 0.967522, 0.0, -0.0, 0.0, 2.3434, 0.0};
//     std::vector<double> inital_guess{-0.017, 1.008, 0.870, -0.006, -0.008, 2.345, 0.0};  //lidar和gps机械xyzRPY
//     std::vector<double> inital_guess{0, 0, 0.870, 0, 0, 0, 0.0};  //只用lidar和gps机械外參的Z 其余为0
//     std::vector<double> inital_guess{-0.517, 0.508, 0.870, -0.506, -0.508, 1.845, 0.0};  //只用lidar和gps机械外參的Z 其余的-0.5
//     std::vector<double> inital_guess{0.483, 1.508, 0.870, 0.494, 0.492, 2.845, 0.0};  //只用lidar和gps机械外參的Z 其余的+0.5   
//     std::vector<double> inital_guess{-0.017, 1.008, 0.870, -0.006, -0.008, 2.345, 0.0};  //只用lidar和gps机械外參的Z
//     std::vector<double> inital_guess{0.022381, 0.955674, 1.44462, -0.00132223, 0.00397518, 2.34328, 0};  //校准的初始估计   仅用于运行 local 模式   标定文件中lidar和gps的XYZRPY
    
//     std::vector<double> inital_guess{-0.050, -0.355, 1.000, -0.013, 0.006, 0.000, 0};   //demo车机械外参 rosrun tf tf_echo gps_fix lidar_link
//     std::vector<double> inital_guess{-0.050, -0.355, 1.000, -0.013, 0.006, 0.000, 0};   //1
//     std::vector<double> inital_guess{-0.060, -0.365, 1.000, -0.023, -0.004, -0.010, 0};   //2
//     std::vector<double> inital_guess{-0.070, -0.375, 1.000, -0.033, -0.014, -0.020, 0};   //3
//     std::vector<double> inital_guess{-0.080, -0.385, 1.000, -0.043, -0.024, -0.030, 0};   //4
//     std::vector<double> inital_guess{-0.090, -0.395, 1.000, -0.053, -0.034, -0.040, 0};   //5
//     std::vector<double> inital_guess{-0.100, -0.405, 1.000, -0.063, -0.044, -0.050, 0};   //6
//     std::vector<double> inital_guess{-0.110, -0.415, 1.000, -0.073, -0.054, -0.060, 0};   //7
//     std::vector<double> inital_guess{-0.120, -0.425, 1.000, -0.083, -0.064, -0.070, 0};   //8
//     std::vector<double> inital_guess{-0.130, -0.435, 1.000, -0.093, -0.074, -0.080, 0};   //9
//     std::vector<double> inital_guess{-0.140, -0.445, 1.000, -0.103, -0.084, -0.090, 0};   //10
    
//     std::vector<double> inital_guess{-0.040, -0.345, 1.000, -0.003, 0.006, 0.010, 0};   //11
//     std::vector<double> inital_guess{-0.030, -0.335, 1.000, 0.007, 0.016, 0.020, 0};   //12
//     std::vector<double> inital_guess{-0.020, -0.325, 1.000, 0.017, 0.026, 0.030, 0};   //13
//     std::vector<double> inital_guess{-0.010, -0.315, 1.000, 0.027, 0.036, 0.040, 0};   //14
    std::vector<double> inital_guess{-0.00, -0.305, 1.000, 0.037, 0.046, 0.050, 0};   //15
//     std::vector<double> inital_guess{0.010, -0.295, 1.000, 0.047, 0.056, 0.060, 0};   //16
//     std::vector<double> inital_guess{0.020, -0.285, 1.000, 0.057, 0.066, 0.070, 0};   //17
//     std::vector<double> inital_guess{0.030, -0.275, 1.000, 0.067, 0.076, 0.080, 0};   //18
//     std::vector<double> inital_guess{0.040, -0.265, 1.000, 0.077, 0.086, 0.090, 0};   //19
    
    
    
    double max_time_offset = 0.1;   //default 最大的时间对准偏差
    double angular_range = 0.5;     //default 在初始估计位置的搜索范围（弧度）角度偏移上下限 +-0.5弧度
//     double angular_range = 0.262;   //15度     
    double translation_range = 1.0; //default 在初始估计位置的搜索范围（位置）位置偏移上下限 +-1米
    //double translation_range = 0.5; //在初始估计位置的搜索范围（位置）位置偏移上下限 +-0.5米
    
    double max_evals = 300;     //default 最大的评估次数
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
