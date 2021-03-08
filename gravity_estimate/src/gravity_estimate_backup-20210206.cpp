#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//SAC segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//filteres
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#define DO_VOXELGRID_FILTER

#define LEAF_SIZE 0.025
#define DISTANCE_TH 0.01

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZ> pcl_out;
  sensor_msgs::PointCloud2 output;

  //convert
  pcl::PCLPointCloud2 convert_temp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>);

  pcl_conversions::toPCL(*input, convert_temp);
  pcl::fromPCLPointCloud2(convert_temp, *pcl_in);
  //pcl::fromPCLPointCloud2((pcl::PCLPointCloud2&)input, *pcl_in);

  //voxel grid filter
#ifdef DO_VOXELGRID_FILTER
  pcl::PointCloud<pcl::PointXYZ>::Ptr vg_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vgf;
  
  vgf.setInputCloud(pcl_in);
  vgf.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  vgf.filter(*vg_filtered);
#endif

  //SAC segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indice (new pcl::PointIndices);

#ifdef DO_VOXELGRID_FILTER
  seg.setInputCloud(vg_filtered);
#else
  seg.setInputCloud(pcl_in);
#endif
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(DISTANCE_TH);

  seg.segment(*indice, *coef);

  //plane extraction
  pcl::ExtractIndices<pcl::PointXYZ> extract;

#ifdef DO_VOXELGRID_FILTER
  extract.setInputCloud(vg_filtered);
#else
  extract.setInputCloud(pcl_in);
#endif
  extract.setIndices(indice);
  extract.setNegative(false);

  extract.filter(pcl_out);

  // Publish the data
  pcl::toPCLPointCloud2(pcl_out, (pcl::PCLPointCloud2&)output);
  pub.publish (output);
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "gravity_estimate");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("gravity_output", 1);

  // Spin
  ros::spin ();
}

