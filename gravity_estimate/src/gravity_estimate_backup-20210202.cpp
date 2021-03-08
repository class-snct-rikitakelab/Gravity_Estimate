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
//plane extraction
#include <pcl/filters/extract_indices.h>

#define DISTANCE_TH 0.01

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
  //pcl::PointCloud<pcl::PointXYZ> pcl_in;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 output;
  //SAC segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indice (new pcl::PointIndices);
  //plane extraction
  pcl::ExtractIndices<pcl::PCLPointCloud2> extract;

////////////////////////////////////////////////////

  //convert
  pcl::PCLPointCloud2 pcl_temp;
  pcl_conversions::toPCL(*input, pcl_temp);
  pcl::fromPCLPointCloud2(pcl_temp, *pcl_in);

  //SAC segmentation
  seg.setInputCloud(pcl_in);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(DISTANCE_TH);

  seg.segment(*indice, *coef);

  //plane extraction
  extract.setInputCloud((pcl::PCLPointCloud2ConstPtr&)input);
  extract.setIndices(indice);
  extract.setNegative(false);

  extract.filter((pcl::PCLPointCloud2&)output);

  // Publish the data
  pub.publish (output);
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);

  // Spin
  ros::spin ();
}

