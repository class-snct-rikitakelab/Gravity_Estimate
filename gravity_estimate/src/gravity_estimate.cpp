//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
//SAC segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//normal estimation
#include <pcl/features/normal_3d.h>
//debug
#include <pcl/visualization/pcl_visualizer.h>

#define LEAF_SIZE   0.03  //Voxle Grid Filter
#define DISTANCE_TH 0.035 //Plane segmentation coef
#define NORM_RADIUS 1.0   //normal estimation searching radius

//ros publisher
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;

//pcl visualizer
pcl::visualization::PCLVisualizer viewer{"pcl_normals"};


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZ> pcl_out; //output from pcl
  sensor_msgs::PointCloud2 output; //output for ros publisher

  ///////////////
  //convert 
  pcl::PCLPointCloud2 convert_temp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>);

  pcl_conversions::toPCL(*input, convert_temp);
  pcl::fromPCLPointCloud2(convert_temp, *pcl_in);
  //pcl::fromPCLPointCloud2((pcl::PCLPointCloud2&)input, *pcl_in);

  ///////////////
  //voxel grid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr vg_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vgf;
  
  vgf.setInputCloud(pcl_in);
  vgf.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);

  vgf.filter(*vg_filtered);

  sensor_msgs::PointCloud2 vgf_out;
  pcl::toPCLPointCloud2(*vg_filtered, (pcl::PCLPointCloud2&)vgf_out);
  pub2.publish (vgf_out);

  ///////////////
  //SAC segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indice (new pcl::PointIndices);

  seg.setInputCloud(vg_filtered);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(DISTANCE_TH);

  seg.segment(*indice, *coef);

  ///////////////
  //plane extraction
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

  extract.setInputCloud(vg_filtered);
  extract.setIndices(indice);
  extract.setNegative(false);

  extract.filter(*plane);

  sensor_msgs::PointCloud2 plane_out;
  pcl::toPCLPointCloud2(*plane, (pcl::PCLPointCloud2&)plane_out);
  pub3.publish (plane_out);

  ///////////////
  //normal estimation
  pcl::PointCloud<pcl::PointNormal>::Ptr 
    norm_in (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr norm_out(new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm;
  pcl::search::KdTree<pcl::PointNormal>::Ptr 
    kd_tree (new pcl::search::KdTree<pcl::PointNormal> ());

  pcl::copyPointCloud(*plane, *norm_in);
  norm.setInputCloud(norm_in);
  norm.setSearchMethod(kd_tree);
  norm.setRadiusSearch(NORM_RADIUS);

  norm.compute(*norm_out);

  ///////////////
  // Publish the data
  pcl::toPCLPointCloud2(*norm_out, (pcl::PCLPointCloud2&)output);
  pub.publish (output);

  ///////////////
  // PCL Visualizer
  viewer.removeAllPointClouds();

  viewer.addPointCloud(plane, "plane");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "plane");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane");

  viewer.addPointCloudNormals<pcl::PointNormal>(norm_out, 10, 0.5, "normal");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normal");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normal");

  viewer.spinOnce();
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "gravity_estimate");
  ros::NodeHandle nh;

  viewer.setBackgroundColor(1,1,1);
  viewer.addCoordinateSystem(0.5, "axis");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub  = nh.advertise<sensor_msgs::PointCloud2> ("gravity/normal", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("gravity/vgf", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("gravity/plane", 1);

  // Spin
  ros::spin ();
}

