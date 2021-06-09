/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <ui/calib_helper.h>
#include <core/scan_undistortion.h>
#include <utils/tic_toc.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <fstream>
#include <sstream>

#include <tf_conversions/tf_eigen.h>

namespace licalib
{
CalibrHelper::CalibrHelper(ros::NodeHandle& nh)
  : calib_step_(Start)
  , iteration_step_(0)
  , opt_time_offset_(false)
  , plane_lambda_(0.6)
  , ndt_resolution_(0.5)
  , associated_radius_(0.05)
{
  std::string topic_lidar;
  double bag_start, bag_durr;
  double scan4map;
  double knot_distance;
  double time_offset_padding;

  nh.param<std::string>("path_bag", bag_path_, "V1_01_easy.bag");
  nh.param<std::string>("topic_imu", topic_imu_, "/imu0");
  nh.param<std::string>("topic_lidar", topic_lidar, "/velodyne_packets");
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  nh.param<double>("scan4map", scan4map, 15);
  nh.param<double>("ndtResolution", ndt_resolution_, 0.5);
  nh.param<double>("time_offset_padding", time_offset_padding, 0.015);
  nh.param<double>("knot_distance", knot_distance, 0.02);
  std::cout << "knot_distance: " << knot_distance << std::endl;

  //// debug
  // bag_path_ = "/home/liuzhiyang/data/test_data/LI-calib/changzhoufactory_2021-05-18-10-39-52.bag";
  // topic_imu_ = "/INS_D/imu_data";
  // topic_lidar = "/rslidar1_points";
  // bag_start = 1;
  // bag_durr = 300;
  // scan4map = 15;
  // ndt_resolution_ = 1.0;
  // time_offset_padding = 0.015;
  // knot_distance = 0.002;

  if (!createCacheFolder(bag_path_))
  {
    calib_step_ = Error;
  }

  {
    std::string lidar_model = "VLP_16";
    nh.param<std::string>("LidarModel", lidar_model, "VLP_16");
    IO::LidarModelType lidar_model_type = IO::LidarModelType::VLP_16;
    if (lidar_model == "VLP_16")
    {
      lidar_model_type = IO::LidarModelType::VLP_16;
    }
    else
    {
      calib_step_ = Error;
      ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
    }
    /// read dataset
    std::cout << "\nLoad dataset from " << bag_path_ << std::endl;
    IO::LioDataset lio_dataset_temp(lidar_model_type);
    lio_dataset_temp.read(bag_path_, topic_imu_, topic_lidar, bag_start, bag_durr);
    dataset_reader_ = lio_dataset_temp.get_data();
    dataset_reader_->adjustDataset();
  }
  map_time_ = dataset_reader_->get_start_time();
  scan4map_time_ = map_time_ + scan4map;
  double end_time = dataset_reader_->get_end_time();
  traj_manager_ = std::make_shared<TrajectoryManager>(map_time_, end_time, knot_distance, time_offset_padding);

  scan_undistortion_ = std::make_shared<ScanUndistortion>(traj_manager_, dataset_reader_);

  lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);

  rotation_initializer_ = std::make_shared<InertialInitializer>();

  surfel_association_ = std::make_shared<SurfelAssociation>(associated_radius_, plane_lambda_);
}

bool CalibrHelper::createCacheFolder(const std::string& bag_path)
{
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag")
  {
    return false;
  }
  cache_path_ = p.parent_path().string() + "/" + p.stem().string();
  boost::filesystem::create_directory(cache_path_);
  return true;
}

std::vector<double> q2eular(const Eigen::Quaterniond& q)
{
  tf::Quaternion q_tf;
  tf::quaternionEigenToTF(q, q_tf);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  std::vector<double> eular{ roll, pitch, yaw };
  return eular;
}

void CalibrHelper::Initialization()
{
  if (Start != calib_step_)
  {
    ROS_WARN("[Initialization] Need status: Start.");
    return;
  }
  // imu数据处理： 拟合so3轨迹(旋转部分)
  // std::string file_name = "/home/liuzhiyang/Desktop/data/imu.txt";
  // std::ofstream file_writer(file_name, std::ios_base::out | std::ios_base::trunc);
  // file_writer << std::fixed << std::setprecision(8);
  for (const auto& imu_data : dataset_reader_->get_imu_data())
  {
    traj_manager_->feedIMUData(imu_data);
    // if (file_writer.is_open())
    // {
    //   file_writer << imu_data.accel(0) << " " << imu_data.accel(1) << " " << imu_data.accel(2) << " "\
    //               << imu_data.gyro(0) << " " << imu_data.gyro(1) << " " << imu_data.gyro(2) << std::endl;
    // }
  }
  // file_writer.close();
  // std::cout << "write file done!" << std::endl;
  traj_manager_->initialSO3TrajWithGyro();

  // debug: save traj result
  std::string file_name = "/home/liuzhiyang/Desktop/data/imu_att.txt";
  std::ofstream file_writer(file_name, std::ios_base::out | std::ios_base::trunc);
  file_writer << std::fixed << std::setprecision(5);
  size_t imu_size = dataset_reader_->get_imu_data().size();
  std::cout << "imu data is : " << imu_size << std::endl;
  bool first_flag = true;
  Eigen::Quaterniond first_att;
  for (const auto& imu_data : dataset_reader_->get_imu_data())
  {
    double imu_time = imu_data.timestamp;
    Eigen::Quaterniond data_att;
    if (first_flag)
    {
      first_att = imu_data.attitude;
      data_att = Eigen::Quaterniond::Identity();
      first_flag = false;
    }
    else
    {
      data_att = first_att.conjugate() * imu_data.attitude;
    }
    auto data_euler = q2eular(data_att);
    Eigen::Quaterniond traj_att;
    if (!traj_manager_->evaluateIMUAtt(imu_time, traj_att))
    {
      std::cout << "fail to find imu att" << std::endl;
      return;
    }
    auto traj_euler = q2eular(traj_att);

    if (file_writer.is_open())
    {
      file_writer << data_euler[0] * 180 / M_PI << " " << data_euler[1] * 180 / M_PI << " "
                  << data_euler[2] * 180 / M_PI << " " << traj_euler[0] * 180 / M_PI << " "
                  << traj_euler[1] * 180 / M_PI << " " << traj_euler[2] * 180 / M_PI << std::endl;
    }
  }
  file_writer.close();
  std::cout << "write file done!" << std::endl;

  // // lidar数据处理：ndt匹配
  // for(const TPointCloud& raw_scan: dataset_reader_->get_scan_data()) {
  //   VPointCloud::Ptr cloud(new VPointCloud);
  //   TPointCloud2VPointCloud(raw_scan.makeShared(), cloud);
  //   double scan_timestamp = pcl_conversions::fromPCL(raw_scan.header.stamp).toSec();
  //   lidar_odom_->feedScan(scan_timestamp, cloud);

  //   if (lidar_odom_->get_odom_data().size() < 30
  //       || (lidar_odom_->get_odom_data().size() % 10 != 0))
  //     continue;
  //   //  外参的旋转部分估计：手眼标定
  //   if (rotation_initializer_->EstimateRotation(traj_manager_,
  //                                               lidar_odom_->get_odom_data())) {
  //     Eigen::Quaterniond qItoLidar = rotation_initializer_->getQ_ItoS();
  //     traj_manager_->getCalibParamManager()->set_q_LtoI(qItoLidar.conjugate());

  //     Eigen::Vector3d euler_ItoL = qItoLidar.toRotationMatrix().eulerAngles(0,1,2);
  //     std::cout << "[Initialization] Done. Euler_ItoL initial degree: "
  //               << (euler_ItoL*180.0/M_PI).transpose() << std::endl;
  //     calib_step_ = InitializationDone;
  //     break;
  //   }
  // }

  Eigen::Matrix3d matrixItoLidar;
  matrixItoLidar << -5.4147422169222970e-01, 8.4052587576459337e-01, 1.7942112834261007e-02, -8.4045115224542399e-01,
      -5.4171649953227685e-01, 1.3604955855701921e-02, 2.1154855994141502e-02, -7.7127365221487335e-03,
      9.9974646074052564e-01;
  Eigen::Quaterniond qItoLidar(matrixItoLidar);
  traj_manager_->getCalibParamManager()->set_q_LtoI(qItoLidar.conjugate());
  Eigen::Vector3d euler_ItoL = qItoLidar.toRotationMatrix().eulerAngles(0, 1, 2);
  std::cout << "[Initialization] Done. Euler_ItoL initial degree: " << (euler_ItoL * 180.0 / M_PI).transpose()
            << std::endl;
  calib_step_ = InitializationDone;

  if (calib_step_ != InitializationDone)
    ROS_WARN("[Initialization] fails.");
}

void CalibrHelper::DataAssociation()
{
  std::cout << "[Association] start ...." << std::endl;
  TicToc timer;
  timer.tic();

  /// set surfel pap
  if (InitializationDone == calib_step_)
  {
    // 去除运动畸变+ndt匹配
    Mapping();
    // 将每帧点云数据变换到第一帧下
    scan_undistortion_->undistortScanInMap(lidar_odom_->get_odom_data());

    // 计算每个cell中的平面
    surfel_association_->setSurfelMap(lidar_odom_->getNDTPtr(), map_time_);
  }
  else if (BatchOptimizationDone == calib_step_ || RefineDone == calib_step_)
  {
    scan_undistortion_->undistortScanInMap();

    plane_lambda_ = 0.7;
    surfel_association_->setPlaneLambda(plane_lambda_);
    auto ndt_omp = LiDAROdometry::ndtInit(ndt_resolution_);
    ndt_omp->setInputTarget(scan_undistortion_->get_map_cloud());
    surfel_association_->setSurfelMap(ndt_omp, map_time_);
  }
  else
  {
    ROS_WARN("[DataAssociation] Please follow the step.");
    return;
  }

  /// get association
  for (auto const& scan_raw : dataset_reader_->get_scan_data())
  {
    auto iter = scan_undistortion_->get_scan_data_in_map().find(scan_raw.header.stamp);
    if (iter == scan_undistortion_->get_scan_data_in_map().end())
    {
      continue;
    }
    surfel_association_->getAssociation(iter->second, scan_raw.makeShared(), 2);
  }
  surfel_association_->averageTimeDownSmaple();
  std::cout << "Surfel point number: " << surfel_association_->get_surfel_points().size() << std::endl;
  std::cout << GREEN << "[Association] " << timer.toc() << " ms" << RESET << std::endl;

  if (surfel_association_->get_surfel_points().size() > 10)
  {
    calib_step_ = DataAssociationDone;
  }
  else
  {
    ROS_WARN("[DataAssociation] fails.");
  }
}

void CalibrHelper::BatchOptimization()
{
  if (DataAssociationDone != calib_step_)
  {
    ROS_WARN("[BatchOptimization] Need status: DataAssociationDone.");
    return;
  }
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  TicToc timer;
  timer.tic();
  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);

  calib_step_ = BatchOptimizationDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");
  std::cout << GREEN << "[BatchOptimization] " << timer.toc() << " ms" << RESET << std::endl;
}

void CalibrHelper::Refinement()
{
  if (BatchOptimizationDone > calib_step_)
  {
    ROS_WARN("[Refinement] Need status: BatchOptimizationDone.");
    return;
  }
  iteration_step_++;
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  DataAssociation();
  if (DataAssociationDone != calib_step_)
  {
    ROS_WARN("[Refinement] Need status: DataAssociationDone.");
    return;
  }
  TicToc timer;
  timer.tic();

  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);
  calib_step_ = RefineDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");

  std::cout << GREEN << "[Refinement] " << timer.toc() << " ms" << RESET << std::endl;
}

void CalibrHelper::Mapping(bool relocalization)
{
  bool update_map = true;
  if (relocalization)
  {
    lidar_odom_->clearOdomData();
    update_map = false;
  }
  else
  {
    // 根据IMU的轨迹和初始外参，将点云数据变换到帧头
    scan_undistortion_->undistortScan();
    lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);
  }

  // 对lidar帧再次进行ndt匹配
  double last_scan_t = 0;
  for (const auto& scan_raw : dataset_reader_->get_scan_data())
  {
    double scan_t = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
    if (scan_t > scan4map_time_)
      update_map = false;
    auto iter = scan_undistortion_->get_scan_data().find(scan_raw.header.stamp);
    if (iter != scan_undistortion_->get_scan_data().end())
    {
      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_L2toL1 = Eigen::Quaterniond::Identity();
      if (last_scan_t > 0 && traj_manager_->evaluateLidarRelativeRotation(last_scan_t, scan_t, q_L2toL1))
      {
        pose_predict.block<3, 3>(0, 0) = q_L2toL1.toRotationMatrix();
      }
      lidar_odom_->feedScan(scan_t, iter->second, pose_predict, update_map);
      last_scan_t = scan_t;
    }
  }
}

void CalibrHelper::saveCalibResult(const std::string& calib_result_file) const
{
  if (!boost::filesystem::exists(calib_result_file))
  {
    std::ofstream outfile;
    outfile.open(calib_result_file, std::ios::app);
    outfile << "bag_path"
            << ","
            << "imu_topic"
            << ","
            << "map_time"
            << ","
            << "iteration_step"
            << ","
            << "p_IinL.x"
            << ","
            << "p_IinL.y"
            << ","
            << "p_IinL.z"
            << ","
            << "q_ItoL.x"
            << ","
            << "q_ItoL.y"
            << ","
            << "q_ItoL"
            << ","
            << "q_ItoL.w"
            << ","
            << "time_offset"
            << ","
            << "gravity.x"
            << ","
            << "gravity.y"
            << ","
            << "gravity.z"
            << ","
            << "gyro_bias.x"
            << ","
            << "gyro_bias.y"
            << ","
            << "gyro_bias.z"
            << ","
            << "acce_bias.z"
            << ","
            << "acce_bias.y"
            << ","
            << "acce_bias.z"
            << "\n";
    outfile.close();
  }

  std::stringstream ss;
  ss << bag_path_;
  ss << "," << topic_imu_;
  ss << "," << map_time_;
  ss << "," << iteration_step_;
  std::string info;
  ss >> info;

  traj_manager_->getCalibParamManager()->save_result(calib_result_file, info);
}

void CalibrHelper::saveMap() const
{
  if (calib_step_ <= Start)
    return;
  std::string NDT_target_map_path = cache_path_ + "/NDT_target_map.pcd";
  lidar_odom_->saveTargetMap(NDT_target_map_path);

  std::string surfel_map_path = cache_path_ + "/surfel_map.pcd";
  surfel_association_->saveSurfelsMap(surfel_map_path);

  if (RefineDone == calib_step_)
  {
    std::string refined_map_path = cache_path_ + "/refined_map.pcd";
    std::cout << "Save refined map to " << refined_map_path << "; size: " << scan_undistortion_->get_map_cloud()->size()
              << std::endl;
    pcl::io::savePCDFileASCII(refined_map_path, *scan_undistortion_->get_map_cloud());
  }
}
}
