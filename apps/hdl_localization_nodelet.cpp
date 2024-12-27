#include <mutex>
#include <memory>
#include <iostream>
#include <deque>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <hdl_localization/ScanMatchingStatus.h>
#include "hdl_localization/logger.h"
#include "hdl_localization/pose_estimator.hpp"

namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet() : tf_buffer(), tf_listener(tf_buffer) {}
  virtual ~HdlLocalizationNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    load_param_from_ros();

    initialize_params();

    if (params.use_imu) {
      LOG_INFO("Enable imu-based prediction.");
      imu_sub = mt_nh.subscribe("/gpsimu_driver/imu_data", 256, &HdlLocalizationNodelet::imu_callback, this);
    }
    globalmap_sub = nh.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    points_sub = mt_nh.subscribe("/velodyne_points", 5, &HdlLocalizationNodelet::points_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);
    status_pub = nh.advertise<ScanMatchingStatus>("/status", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
  }

private:
  struct LocParams {
    std::string robot_odom_frame_id = "robot_odom";
    std::string odom_child_frame_id = "base_link";
    bool use_imu = true;
    bool invert_acc = false;
    bool invert_gyro = false;
    std::string reg_method = "NDT_OMP";
    std::string ndt_neighbor_search_method = "DIRECT7";
    double ndt_neighbor_search_radius = 2.0;
    double ndt_resolution = 1.0;
    double downsample_resolution = 0.1;
    bool specify_init_pose = true;
    double init_pos_x = 0.0;
    double init_pos_y = 0.0;
    double init_pos_z = 0.0;
    double init_ori_w = 1.0;
    double init_ori_x = 0.0;
    double init_ori_y = 0.0;
    double init_ori_z = 0.0;
    double cool_time_duration = 0.5;
    bool enable_robot_odometry_prediction = false;
    double status_max_correspondence_dist = 0.5;
    double status_max_valid_point_dist = 25.0;
  };

  void load_param_from_ros() {
    params.robot_odom_frame_id = private_nh.param<std::string>("hdl_loc/robot_odom_frame_id", "robot_odom");
    params.odom_child_frame_id = private_nh.param<std::string>("hdl_loc/odom_child_frame_id", "base_link");
    params.use_imu = private_nh.param<bool>("hdl_loc/use_imu", true);
    params.invert_acc = private_nh.param<bool>("hdl_loc/invert_acc", false);
    params.invert_gyro = private_nh.param<bool>("hdl_loc/invert_gyro", false);
    params.reg_method = private_nh.param<std::string>("hdl_loc/reg_method", "NDT_OMP");
    params.ndt_neighbor_search_method = private_nh.param<std::string>("hdl_loc/ndt_neighbor_search_method", "DIRECT7");
    params.ndt_neighbor_search_radius = private_nh.param<double>("hdl_loc/ndt_neighbor_search_radius", 2.0);
    params.ndt_resolution = private_nh.param<double>("hdl_loc/ndt_resolution", 1.0);
    params.downsample_resolution = private_nh.param<double>("hdl_loc/downsample_resolution", 0.1);
    params.specify_init_pose = private_nh.param<bool>("hdl_loc/specify_init_pose", true);
    params.init_pos_x = private_nh.param<double>("hdl_loc/init_pos_x", 0.0);
    params.init_pos_y = private_nh.param<double>("hdl_loc/init_pos_y", 0.0);
    params.init_pos_z = private_nh.param<double>("hdl_loc/init_pos_z", 0.0);
    params.init_ori_w = private_nh.param<double>("hdl_loc/init_ori_w", 1.0);
    params.init_ori_x = private_nh.param<double>("hdl_loc/init_ori_x", 0.0);
    params.init_ori_y = private_nh.param<double>("hdl_loc/init_ori_y", 0.0);
    params.init_ori_z = private_nh.param<double>("hdl_loc/init_ori_z", 0.0);
    params.cool_time_duration = private_nh.param<double>("hdl_loc/cool_time_duration", 0.5);
    params.enable_robot_odometry_prediction = private_nh.param<bool>("hdl_loc/enable_robot_odometry_prediction", false);
    params.status_max_correspondence_dist = private_nh.param<double>("hdl_loc/status_max_correspondence_dist", 0.5);
    params.status_max_valid_point_dist = private_nh.param<double>("hdl_loc/status_max_valid_point_dist", 25.0);
  }

  void initialize_params() {
    // intialize scan matching method
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(params.downsample_resolution, params.downsample_resolution, params.downsample_resolution);
    downsample_filter = voxelgrid;

    LOG_INFO("Create registration method for localization.");
    registration = create_registration();

    // initialize pose estimator
    if (params.specify_init_pose) {
      LOG_INFO("Initialize pose estimator with specified parameters.");
      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        Eigen::Vector3f(params.init_pos_x, params.init_pos_y, params.init_pos_z),
        Eigen::Quaternionf(params.init_ori_w, params.init_ori_x, params.init_ori_y, params.init_ori_z),
        params.cool_time_duration));
    }
  }

  pcl::Registration<PointT, PointT>::Ptr create_registration() const {
    if (params.reg_method == "NDT_OMP") {
      LOG_INFO("NDT_OMP is selected.");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(params.ndt_resolution);
      if (params.ndt_neighbor_search_method == "DIRECT1") {
        LOG_INFO("Search method DIRECT1 is selected.");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (params.ndt_neighbor_search_method == "DIRECT7") {
        LOG_INFO("Search method DIRECT7 is selected.");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (params.ndt_neighbor_search_method == "KDTREE") {
          LOG_INFO("search_method KDTREE is selected.");
        } else {
          LOG_WARN("invalid search method was given.");
          LOG_WARN("default method is selected (KDTREE).");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }

    LOG_ERROR("unknown registration method: %s!", params.reg_method.c_str());
    return nullptr;
  }

  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    if (!globalmap) {
      LOG_WARN("Globalmap has not been received!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
      LOG_ERROR("Cloud is empty, at: %f.!", stamp.toSec());
      return;
    }

    // todo::当使用IMU时，通过tf发布外参，可将点云变换到IMU系
    // transform pointcloud into odom_child_frame_id
    std::string tfError;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (this->tf_buffer.canTransform(params.odom_child_frame_id, pcl_cloud->header.frame_id, stamp, ros::Duration(0.1), &tfError)) {
      if (!pcl_ros::transformPointCloud(params.odom_child_frame_id, *pcl_cloud, *cloud, this->tf_buffer)) {
        LOG_ERROR("Point cloud cannot be transformed into target frame!");
        return;
      }
    } else {
      LOG_ERROR(tfError.c_str());
      return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if (!pose_estimator) {
      LOG_ERROR("Waiting for initial pose input!");
      return;
    }

    // predict
    if (!params.use_imu) {
      pose_estimator->predict(stamp.toSec());
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      while (!imu_data.empty() && imu_data.front()->header.stamp <= stamp) {
        const auto& gyro = imu_data.front()->angular_velocity;
        const auto& acc = imu_data.front()->linear_acceleration;
        Eigen::Vector3f acc_vec = Eigen::Vector3f(acc.x, acc.y, acc.z);
        Eigen::Vector3f gyro_vec = Eigen::Vector3f(gyro.x, gyro.y, gyro.z);
        double acc_sign = params.invert_acc ? -1.0 : 1.0;
        double gyro_sign = params.invert_gyro ? -1.0 : 1.0;
        pose_estimator->predict(imu_data.front()->header.stamp.toSec(), acc_sign * acc_vec, gyro_sign * gyro_vec);
        imu_data.pop_front();
      }
    }

    // odometry-based prediction
    double last_correction_time = pose_estimator->last_correction_time();
    if (params.enable_robot_odometry_prediction && (last_correction_time != 0.0)) {
      geometry_msgs::TransformStamped odom_delta;
      ros::Time last_stamp(last_correction_time);  // 将 double 转为 ros::Time 用于 TF 查询

      if (tf_buffer.canTransform(params.odom_child_frame_id, last_stamp, params.odom_child_frame_id, stamp, params.robot_odom_frame_id, ros::Duration(0.1))) {
        odom_delta = tf_buffer.lookupTransform(params.odom_child_frame_id, last_stamp, params.odom_child_frame_id, stamp, params.robot_odom_frame_id, ros::Duration(0));
      } else if (tf_buffer.canTransform(params.odom_child_frame_id, last_stamp, params.odom_child_frame_id, ros::Time(0), params.robot_odom_frame_id, ros::Duration(0))) {
        odom_delta = tf_buffer.lookupTransform(params.odom_child_frame_id, last_stamp, params.odom_child_frame_id, ros::Time(0), params.robot_odom_frame_id, ros::Duration(0));
      }

      if (odom_delta.header.stamp.isZero()) {
        LOG_WARN("Failed to look up transform between %s and %s.", cloud->header.frame_id.c_str(), params.robot_odom_frame_id.c_str());
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // correct
    auto aligned = pose_estimator->correct(stamp.toSec(), filtered);

    if (aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }

    if (status_pub.getNumSubscribers()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    LOG_INFO("Global map received.");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    LOG_INFO("Initial pose received.");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(new hdl_localization::PoseEstimator(registration, Eigen::Vector3f(p.x, p.y, p.z), Eigen::Quaternionf(q.w, q.x, q.y, q.z), params.cool_time_duration));
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter) {
      LOG_WARN("No downsample_filter, will not downsample!");
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    if (tf_buffer.canTransform(params.robot_odom_frame_id, params.odom_child_frame_id, ros::Time(0))) {
      geometry_msgs::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = params.odom_child_frame_id;
      map_wrt_frame.child_frame_id = "map";

      geometry_msgs::TransformStamped frame_wrt_odom = tf_buffer.lookupTransform(params.robot_odom_frame_id, params.odom_child_frame_id, ros::Time(0), ros::Duration(0.1));
      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.transform = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = params.robot_odom_frame_id;

      tf_broadcaster.sendTransform(odom_trans);
    } else {
      geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = params.odom_child_frame_id;
      tf_broadcaster.sendTransform(odom_trans);
    }

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    tf::poseEigenToMsg(Eigen::Isometry3d(pose.cast<double>()), odom.pose.pose);
    odom.child_frame_id = params.odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }

  /**
   * @brief publish scan matching status information
   */
  void publish_scan_matching_status(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = 0.0;

    const double max_correspondence_dist = params.status_max_correspondence_dist;
    const double max_valid_point_dist = params.status_max_valid_point_dist;

    int num_inliers = 0;
    int num_valid_points = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      if (pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }
      num_valid_points++;

      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }

    status.matching_error /= num_inliers;
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    std::vector<double> errors(6, 0.0);

    if (pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = params.use_imu ? "imu" : "motion_model";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub.publish(status);
  }

private:
  LocParams params;

  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher status_pub;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_data;

  pcl::PointCloud<PointT>::ConstPtr last_scan;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  // 输入点云降采样滤波器
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;
};
}  // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
