#include "lidar_align/aligner.h"

namespace lidar_align {

Aligner::Aligner(const Config& config) : config_(config){};

Aligner::Config Aligner::getConfig(ros::NodeHandle* nh) {
  Aligner::Config config;
  nh->param("local", config.local, config.local);   //dbg(config.local);
  
  // 从参数服务器传入初始位姿
  std::stringstream ss;
  nh->getParam("initial_pose", config.initial_pose);
  ss << config.initial_pose;
  for (int k = 0; k < 7; k++)
  {
    ss >> config.initial_guess[k];
  }
  dbg(config.initial_guess); //从launch文件获取外参初值

  nh->param("max_time_offset", config.max_time_offset, config.max_time_offset);   //dbg(config.max_time_offset);
  nh->param("angular_range", config.angular_range, config.angular_range);   //dbg(config.angular_range);
  nh->param("translation_range", config.translation_range, config.translation_range);    //dbg(config.translation_range);
  nh->getParam("max_evals", config.max_evals);    //dbg(config.max_evals);
  nh->param("xtol", config.xtol, config.xtol);    //dbg(config.xtol);
  nh->param("knn_batch_size", config.knn_batch_size, config.knn_batch_size);    //dbg(config.knn_batch_size);
  nh->param("knn_k", config.knn_k, config.knn_k);   //dbg(config.knn_k);
  nh->param("global_knn_max_dist", config.global_knn_max_dist, config.global_knn_max_dist);    //dbg(config.global_knn_max_dist);
  nh->param("local_knn_max_dist", config.local_knn_max_dist, config.local_knn_max_dist);   //dbg(config.local_knn_max_dist);
  nh->param("time_cal", config.time_cal, config.time_cal);    //dbg(config.time_cal);
  nh->param("output_pointcloud_path", config.output_pointcloud_path, config.output_pointcloud_path);
  dbg(config.output_pointcloud_path);
  nh->param("output_calibration_path", config.output_calibration_path, config.output_calibration_path);
  dbg(config.output_calibration_path);
  
  return config;
}

float Aligner::kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                        const Pointcloud& pointcloud, const size_t k,
                        const float max_dist, const size_t start_idx,
                        const size_t end_idx) {
  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  float error = 0;
  for (size_t idx = start_idx; idx < std::min(pointcloud.size(), end_idx);
       ++idx) {
    kdtree.nearestKSearch(pointcloud[idx], k, kdtree_idx, kdtree_dist);
    for (const float& x : kdtree_dist) {
      error += std::min(x, max_dist);
    }
  }
  return error;
}

float Aligner::lidarOdomKNNError(const Pointcloud& base_pointcloud,
                                 const Pointcloud& combined_pointcloud) const {
  // kill optimization if node stopped
  if (!ros::ok()) {
    throw std::runtime_error("ROS node died, exiting");
  }

  // shared_pointer needed by kdtree, no-op destructor to prevent it trying to
  // clean it up after use
  Pointcloud::ConstPtr combined_pointcloud_ptr(&combined_pointcloud,
                                               [](const Pointcloud*) {});

  pcl::KdTreeFLANN<Point> kdtree;

  kdtree.setInputCloud(combined_pointcloud_ptr);

  float max_dist =
      config_.local ? config_.local_knn_max_dist : config_.global_knn_max_dist;

  size_t k = config_.knn_k;
  // if searching own cloud add one to k as a point will always match to itself
  if (&base_pointcloud == &combined_pointcloud) {
    ++k;
  }

//   small amount of threading here to take edge off of bottleneck
//   break knn lookup up into several smaller problems each running in their own
//   thread
  std::vector<std::future<float>> errors;
  for (size_t start_idx = 0; start_idx < base_pointcloud.size();
       start_idx += config_.knn_batch_size) {
    size_t end_idx =
        start_idx + std::min(base_pointcloud.size() - start_idx,
                             static_cast<size_t>(config_.knn_batch_size));
    errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                   kdtree, base_pointcloud, k, max_dist,
                                   start_idx, end_idx));
  }

  // wait for threads to finish and grab results
  float total_error = 0.0f;
  for (std::future<float>& error : errors) {
    total_error += error.get();
  }

  return total_error;
}

float Aligner::lidarOdomKNNError(const Lidar& lidar) const {
  Pointcloud pointcloud;
  lidar.getCombinedPointcloud(&pointcloud);
  return lidarOdomKNNError(pointcloud, pointcloud);
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data) {
  OptData* d = static_cast<OptData*>(f_data);

  if (x.size() > 6) {
    d->lidar->setOdomOdomTransforms(*(d->odom), x[6]);
  }

  Eigen::Matrix<double, 6, 1> vec;    //6x1大小的double类型矩阵
  vec.setZero();    //

  const size_t offset = x.size() == 3 ? 3 : 0;
  for (size_t i = offset; i < 6; ++i) {
    vec[i] = x[i - offset];
  }

  d->lidar->setOdomLidarTransform(Transform::exp(vec.cast<float>()));
  
  //通过kdtree的方法求的每个点的最近邻距离总和
  double error = d->aligner->lidarOdomKNNError(*(d->lidar));

  static int i = 0; //从当前时刻到初始时刻的变换

  std::cout << std::fixed << std::setprecision(2);
  if (x.size() > 3) {
    std::cout << " \e[1mx:\e[0m " << std::setw(6) << vec[0];
    std::cout << " \e[1my:\e[0m " << std::setw(6) << vec[1];
    std::cout << " \e[1mz:\e[0m " << std::setw(6) << vec[2];
  }
  std::cout << " \e[1mrx:\e[0m " << std::setw(6) << vec[3];
  std::cout << " \e[1mry:\e[0m " << std::setw(6) << vec[4];
  std::cout << " \e[1mrz:\e[0m " << std::setw(6) << vec[5];
  if (x.size() > 6) {
    std::cout << " \e[1mtime:\e[0m " << std::setw(6) << x[6];
  }
  std::cout << " \e[1mError:\e[0m " << std::setw(10) << error;
  std::cout << " \e[1mIteration:\e[0m " << i++ << '\r' << std::flush;

  return error;
}

void Aligner::optimize(const std::vector<double>& lb,
                       const std::vector<double>& ub, OptData* opt_data,
                       std::vector<double>* x) {
  nlopt::opt opt;
  // dbg(config_.local);
  if (config_.local) {//看是执行全局优化还是局部优化,用的算法不同,x的个数在前面配置的也不同.全局优化为3个,只求角度;局部优化为7个
    opt = nlopt::opt(nlopt::LN_BOBYQA, x->size());  //无导数局部最优算法BOBYQA
  } else {
    opt = nlopt::opt(nlopt::GN_DIRECT_L, x->size());//无导数全局优化算法DIRECT_L, x的个数在前面配置的是三个
  }

  //设置下限
  opt.set_lower_bounds(lb);
  //设置上限
  opt.set_upper_bounds(ub);

  opt.set_maxeval(config_.max_evals); //设置最大估计次数
  opt.set_xtol_abs(config_.xtol);     //设置x容差停止值

  opt.set_min_objective(LidarOdomMinimizer, opt_data);  //设置目标函数 LidarOdomMinimizer函数名 opt_data外部数据

  double minf;
  std::vector<double> grad;
  // dbg(config_.local);
  nlopt::result result = opt.optimize(*x, minf);
  LidarOdomMinimizer(*x, grad, opt_data);
  // dbg(config_.local);
}

std::string Aligner::generateCalibrationString(const Transform& T,
                                               const double time_offset) {
  Transform::Vector6 T_log = T.log();
  std::stringstream ss;

  ss << "Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor "
        "Frame to  the Lidar Frame:"
     << std::endl
     << "[";
  ss << T_log[0] << ", ";
  ss << T_log[1] << ", ";
  ss << T_log[2] << ", ";
  ss << T_log[3] << ", ";
  ss << T_log[4] << ", ";
  ss << T_log[5] << "]" << std::endl << std::endl;

  ss << "Active Transformation Matrix from the Pose Sensor Frame to  the "
        "Lidar Frame:"
     << std::endl;
  ss << T.matrix() << std::endl << std::endl;

  ss << "Active Translation Vector (x,y,z) from the Pose Sensor Frame to  "
        "the Lidar Frame:"
     << std::endl
     << "[";
  ss << T.translation().x() << ", ";
  ss << T.translation().y() << ", ";
  ss << T.translation().z() << "]" << std::endl << std::endl;

  ss << "Active Hamiltonen Quaternion (w,x,y,z) the Pose Sensor Frame to  "
        "the Lidar Frame:"
     << std::endl
     << "[";
  ss << T.rotation().w() << ", ";
  ss << T.rotation().x() << ", ";
  ss << T.rotation().y() << ", ";
  ss << T.rotation().z() << "]" << std::endl << std::endl;

  if (config_.time_cal) {
    ss << "Time offset that must be added to lidar timestamps in seconds:"
       << std::endl
       << time_offset << std::endl
       << std::endl;
  }

  ss << "ROS Static TF Publisher: <node pkg=\"tf\" "
        "type=\"static_transform_publisher\" "
        "name=\"pose_lidar_broadcaster\" args=\"";
  ss << T.translation().x() << " ";
  ss << T.translation().y() << " ";
  ss << T.translation().z() << " ";
  ss << T.rotation().x() << " ";
  ss << T.rotation().y() << " ";
  ss << T.rotation().z() << " ";
  ss << T.rotation().w() << " POSE_FRAME LIDAR_FRAME 100\" />" << std::endl;

  return ss.str();
}

void Aligner::lidarOdomTransform(Lidar* lidar, Odom* odom) {
  OptData opt_data;
  opt_data.lidar = lidar;
  opt_data.odom = odom;
  opt_data.aligner = this;
  opt_data.time_cal = config_.time_cal;

  /*根据是否使用时间补偿,来确定优化参数的个数*/
  size_t num_params = 6;  //初始化优化参数的个数
  if (config_.time_cal) {
    ++num_params;   //使用时间补偿计算,优化参数个数为7
  }

  std::vector<double> x(num_params, 0.0);

  /*根据配置参数（是否执行全局优化),执行初始角度的赋值,
   *如果执行全局优化则先进行一个估计,得到角度;否则 直接附设置的初始值*/
  if (!config_.local) { //默认执行全局优化,并用于初始估计
    ROS_INFO("Performing Global Optimization...                             ");

    std::vector<double> lb = {-M_PI, -M_PI, -M_PI}; //设置下限
    std::vector<double> ub = {M_PI, M_PI, M_PI};    //设置上限

    std::vector<double> global_x(3, 0.0);   //设置执行全局优化的 x参数向量设置为3 个,全局优化初值为0.0
    global_x[0] = -0.006;  //雷达和gps机械RPY
    global_x[1] = -0.008;
    global_x[2] = 2.345;
//     global_x[0] = ;   //雷达和gps标定文件中的RPY
//     global_x[1] = ;
//     global_x[2] = ;
    
    optimize(lb, ub, &opt_data, &global_x); //执行优化（全局优化）只求角度的优化结果
    config_.local = true;   //将全局优化的flag关闭, 下次则执行局部优化

    //赋值 角度优化的结果（默认情况：角度赋值全局优化的结果，平移量为0）
    x[0] = -0.017; //雷达和gps机械xyz偏移
    x[1] = 1.008;
    x[2] = 0.870;

//     x[0] = ;//雷达和gps标定文件中的xyz偏移
//     x[1] = ;
//     x[2] = ;


    x[3] = -0.006;//雷达和gps机械RPY
    x[4] = -0.008;
    x[5] = 2.345;

//     x[3] = global_x[0];//全局优化得到的RPY
//     x[4] = global_x[1];
//     x[5] = global_x[2];
    
//     x[3] = ;//雷达和gps标定文件中的RPY
//     x[4] = ;
//     x[5] = ;

  } else {//local为true的话 则为默认的初始值
    x = config_.initial_guess; //初始值全为0 局部优化则初始认为lidar和imu的坐标轴一致对齐,其它情况最好按实际情况粗略配置初始值 
  }

  ROS_INFO("Performing Local Optimization...                                ");
  
  //设置参数向量的下限
  std::vector<double> lb = {
      -config_.translation_range, -config_.translation_range,
      -config_.translation_range, -config_.angular_range,
      -config_.angular_range,     -config_.angular_range};
  //设置参数向量的上限    
  std::vector<double> ub = {
      config_.translation_range, config_.translation_range,
      config_.translation_range, config_.angular_range,
      config_.angular_range,     config_.angular_range};
  //将全局优化得到的角度的初始估计值叠加到上下限中，也就是说初始估计 roll 为0.1弧度，则优化的范围是 0.1 +- 0.5 弧度
  // dbg(lb);
  // dbg(ub);
  for (size_t i = 0; i < 6; ++i) {
    lb[i] += x[i];  //叠加下限
    ub[i] += x[i];  //叠加上限
  }
  //将时间补偿的 优化 范围 加 到 上下限中
  if (config_.time_cal) {
    ub.push_back(config_.max_time_offset);
    lb.push_back(-config_.max_time_offset);
  }
  // dbg(lb);
  // dbg(ub);
  // dbg(x);
  /*执行局部优化*/
  optimize(lb, ub, &opt_data, &x);

  if (!config_.output_pointcloud_path.empty()) {
    ROS_INFO(
        "Saving Aligned Pointcloud...                                     ");
    lidar->saveCombinedPointcloud(config_.output_pointcloud_path);
  }

  const std::string output_calibration =
      generateCalibrationString(lidar->getOdomLidarTransform(), x.back());
  if (!config_.output_calibration_path.empty()) {
    ROS_INFO("Saving Calibration File...                                ");

    std::ofstream file;
    file.open(config_.output_calibration_path, std::ofstream::out);
    file << output_calibration;
    file.close();
  }
  ROS_INFO("\e[1mFinal Calibration:\e[0m                                ");
  std::cout << output_calibration;
}

}  // namespace lidar_align
