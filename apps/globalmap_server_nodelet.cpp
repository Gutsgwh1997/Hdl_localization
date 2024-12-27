#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {}
  virtual ~GlobalmapServerNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    readParamsFromROS();

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    map_update_sub = nh.subscribe("/map_request/pcd", 10, &GlobalmapServerNodelet::map_update_callback, this);

    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::pub_once_cb, this, true, true);
  }

private:
  struct Config {
    std::string globalmap_pcd;
    std::string globalmap_frame_id;
    bool convert_utm_to_local;
    double downsample_resolution;
  };

  void readParamsFromROS() {
    config_.globalmap_pcd = private_nh.param<std::string>("global_map_server/globalmap_pcd", "");
    config_.globalmap_frame_id = private_nh.param<std::string>("global_map_server/globalmap_frame_id", "map");
    config_.convert_utm_to_local = private_nh.param<bool>("global_map_server/convert_utm_to_local", true);
    config_.downsample_resolution = private_nh.param<double>("global_map_server/downsample_resolution", 0.1);
  }

  void initialize_params() {
    // read globalmap from a pcd file
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(config_.globalmap_pcd, *globalmap);
    globalmap->header.frame_id = config_.globalmap_frame_id;
    ROS_INFO("Reand %ld pts from %s.", globalmap->size(), config_.globalmap_pcd.c_str());

    std::ifstream utm_file(config_.globalmap_pcd + ".utm");
    if (utm_file.is_open() && config_.convert_utm_to_local) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for (auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      ROS_INFO("Global map offset by UTM reference coordinates (x = %f, y = %f) and altitude (z = %f)", utm_easting, utm_northing, altitude);
    }

    // downsample globalmap
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(config_.downsample_resolution, config_.downsample_resolution, config_.downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    ROS_INFO("After voxel filter, %ld pts remained.", globalmap->size());
  }

  void pub_once_cb(const ros::WallTimerEvent& event) { globalmap_pub.publish(globalmap); }

  void map_update_callback(const std_msgs::String& msg) {
    ROS_INFO("Received map request, map path : %s", msg.data.c_str());
    config_.globalmap_pcd = msg.data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(config_.globalmap_pcd, *globalmap);
    globalmap->header.frame_id = config_.globalmap_frame_id;

    // downsample globalmap
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(config_.downsample_resolution, config_.downsample_resolution, config_.downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    globalmap_pub.publish(globalmap);
  }

private:
  // Config
  Config config_;
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Subscriber map_update_sub;

  ros::WallTimer globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;
};

}  // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
