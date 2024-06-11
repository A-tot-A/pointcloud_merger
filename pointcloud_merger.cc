#include "message_filters/synchronizer.h"
#include <functional>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (std::uint16_t, ring, ring)(float, time, time)
)

ros::Publisher merged_pointscloud_pub;
Eigen::Matrix4d extrins;
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_points0, const sensor_msgs::PointCloud2::ConstPtr& msg_points1) {
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr point0_ptr(new pcl::PointCloud<VelodynePointXYZIRT>());
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr point1_ptr(new pcl::PointCloud<VelodynePointXYZIRT>());
  
  pcl::fromROSMsg(*msg_points0, *point0_ptr);
  pcl::fromROSMsg(*msg_points1, *point1_ptr);

  float time_diff = point1_ptr->points[0].time - point0_ptr->points[0].time;
  for (int i = 0; i < point1_ptr->points.size(); i++) {
    point1_ptr->points[i].time += time_diff;
  }

  pcl::PointCloud<VelodynePointXYZIRT>::Ptr points_transform(new pcl::PointCloud<VelodynePointXYZIRT>());
  pcl::transformPointCloud(*point1_ptr, *points_transform, extrins);
  std::cout << points_transform->size() << ' ' << point0_ptr->size() << std::endl;
  *point0_ptr += *points_transform;
  std::cout << point0_ptr->size() << std::endl;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*point0_ptr, msg);
  msg.header = msg_points0->header;
  merged_pointscloud_pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_merger");
  ros::NodeHandle nh("~");

  std::vector<double> vec_extrinR, vec_extrinT;
  nh.getParam("extrinsic_R", vec_extrinR);
  nh.getParam("extrinsic_T", vec_extrinT);
  if (vec_extrinR.size() != 9u || vec_extrinT.size() != 3u) {
      ROS_ERROR("Please specify pointcloud transform matrix");
      exit(1);
  }
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          extrins(i, j) = vec_extrinR[i*3 + j];
      }
  }
  for (int i = 0; i < 3; i++) {
    extrins(i, 3) = vec_extrinT[i];
  }
  std::cout << "extrins: " << std::endl << extrins << std::endl;

  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub0(nh, "/velodyne_points0", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub1(nh, "/velodyne_points1", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> approximate_sync_policies;
  message_filters::Synchronizer<approximate_sync_policies> sync(approximate_sync_policies(10), points_sub0, points_sub1);
  sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2));

  merged_pointscloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_velodyne_points", 1);
  ros::spin();

  return 0;
}