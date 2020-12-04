#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

int main (int, char**){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("../../hand_gestures/hand_0/image_0000.pcd", *cloud);

  std::cout << "Cloud height: " << cloud->height << std::endl;
  std::cout << "Cloud width: " << cloud->width << std::endl;
  std::cout << "Cloud is dense: " << cloud->is_dense << std::endl;

  std::cout << "Cloud sensor_origin_: " << cloud->sensor_origin_ << std::endl;
  std::cout << "Cloud sensor_orientation_ x: " << cloud->sensor_orientation_.x() << std::endl;
  std::cout << "Cloud sensor_orientation_ y: " << cloud->sensor_orientation_.y() << std::endl;
  std::cout << "Cloud sensor_orientation_ z: " << cloud->sensor_orientation_.z() << std::endl;
  std::cout << "Cloud sensor_orientation_ w: " << cloud->sensor_orientation_.w() << std::endl;

  std::cout << "Cloud is_organized: " << cloud->isOrganized() << std::endl;
  return (0);
} 
