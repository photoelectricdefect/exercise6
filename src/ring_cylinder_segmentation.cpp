#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/MarkerArray.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

// cylinder candidates
ros::Publisher publish_cylinder_candidates, publish_ring_candidates, publish_cylinder_pointcloud;
visualization_msgs::MarkerArray cylinder_candidates_array, ring_candidates_array;

tf2_ros::Buffer tf2_buffer;
int marker_id = 0;
typedef pcl::PointXYZ PointT;

void
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
	std::cerr << "CIGO CIGO CIGO" << std::endl;

  ros::Time time_rec_0,time_rec_1, time_test;
  time_rec_0 = ros::Time::now();
  time_rec_1 = ros::Time::now();

  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  Eigen::Vector4f centroid_cylinder, centroid_ring;

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients), coefficients_ring (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices), inliers_ring (new pcl::PointIndices);

  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_ring (new pcl::PointCloud<PointT> ());

  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);

  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);


  int iter = 0, nr_points = (int) cloud_filtered->size();
  int size_cloud_filtered=cloud_filtered->size();

  while (iter < 5 && size_cloud_filtered > 0) {
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.009);

    if(iter<=0) {
      seg.setInputCloud (cloud_filtered);
      seg.setInputNormals (cloud_normals);
    }
    else {
      seg.setInputCloud (cloud_filtered2);
      seg.setInputNormals (cloud_normals2);
    }

    seg.segment (*inliers_plane, *coefficients_plane);

    // if(inliers_plane->indices.size() <= 0) {
    //   size_cloud_filtered=0;
    //   break;
    // }

    extract.setNegative (true);

    if(iter<=0) {
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers_plane);
    }
    else {
      extract.setInputCloud (cloud_filtered2);
      extract.setIndices (inliers_plane);
    }

    extract.filter (*cloud_filtered2);

    extract_normals.setNegative (true);

    if(iter<=0) {
      extract_normals.setInputCloud (cloud_normals);
      extract_normals.setIndices (inliers_plane);
    }
    else {
      extract_normals.setInputCloud (cloud_normals2);
      extract_normals.setIndices (inliers_plane);
    }

    extract_normals.filter (*cloud_normals2);

    size_cloud_filtered=cloud_filtered2->size();
    iter++;
  }

  if(size_cloud_filtered <= 0) {
    std::cerr << "Can't find the cylindrical component." << std::endl;
    return;
  }

  // // Create the segmentation object for the planar model and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.03);
  // seg.setInputCloud (cloud_filtered);
  // seg.setInputNormals (cloud_normals);
  // seg.segment (*inliers_plane, *coefficients_plane);

  // //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // extract.setNegative (true);
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers_plane);
  // extract.filter (*cloud_filtered2);

  // extract_normals.setNegative (true);
  // extract_normals.setInputCloud (cloud_normals);
  // extract_normals.setIndices (inliers_plane);
  // extract_normals.filter (*cloud_normals2);

  ////////////////////////////////////
  ////////////////////////////////////

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.01);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  // seg.setRadiusLimits (0.06, 0.2);
  seg.setRadiusLimits (0.06, 0.2);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  double cyl_radius=coefficients_cylinder->values[6];
  double angle_y=pcl::getAngle3D(Eigen::Vector3f(coefficients_cylinder->values[3],coefficients_cylinder->values[4],coefficients_cylinder->values[5]),Eigen::Vector3f(0,1,0),true);

  // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  extract.setNegative (false);
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.filter (*cloud_cylinder);

  // extract.setNegative (true);
  // extract.setInputCloud (cloud_filtered2);
  // extract.setIndices (inliers_cylinder);
  // extract.filter (*cloud_filtered3);

  // extract_normals.setNegative (true);
  // extract_normals.setInputCloud (cloud_normals2);
  // extract_normals.setIndices (inliers_cylinder);
  // extract_normals.filter (*cloud_normals3);

  ////////////////////////////////////
  ////////////////////////////////////

  // // Create the segmentation object for ring segmentation and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMaxIterations (10000);
  // seg.setDistanceThreshold (0.05);
  // // seg.setRadiusLimits (0.06, 0.2);
  // seg.setInputCloud (cloud_filtered3);
  // seg.setInputNormals (cloud_normals3);
  // seg.segment (*inliers_ring, *coefficients_ring);

  // // std::cerr << "Ring coefficients: " << *coefficients_ring << std::endl;

  // extract.setNegative (false);
  // extract.setInputCloud (cloud_filtered3);
  // extract.setIndices (inliers_ring);
  // extract.filter (*cloud_ring);

  ////////////////////////////////////
  ////////////////////////////////////

  if (cloud_cylinder->points.empty () || (angle_y>5&&angle_y<175) || cyl_radius>0.15 || cyl_radius<0.08)
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else {
    pcl::compute3DCentroid (*cloud_cylinder, centroid_cylinder);

    geometry_msgs::PointStamped point_camera_cylinder;
    geometry_msgs::PointStamped point_map_cylinder;
    visualization_msgs::Marker marker_cylinder;
    geometry_msgs::TransformStamped tss_cylinder;

    point_camera_cylinder.header.frame_id = "camera_rgb_optical_frame";
    point_camera_cylinder.header.stamp = ros::Time::now();
    point_map_cylinder.header.frame_id = "map";
    point_map_cylinder.header.stamp = ros::Time::now();

		point_camera_cylinder.point.x = centroid_cylinder[0];
		point_camera_cylinder.point.y = centroid_cylinder[1];
		point_camera_cylinder.point.z = centroid_cylinder[2];


		// point_camera_cylinder.point.x = coefficients_cylinder->values[0];
		// point_camera_cylinder.point.y = coefficients_cylinder->values[1];
		// point_camera_cylinder.point.z = coefficients_cylinder->values[2];


	  try{
  	    tss_cylinder = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec_0);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform warning: %s\n", ex.what());
    }

    tf2::doTransform(point_camera_cylinder, point_map_cylinder, tss_cylinder);

    marker_cylinder.header.frame_id = "map";
    marker_cylinder.header.stamp = ros::Time::now();
    marker_cylinder.ns = "cylinder";
    marker_cylinder.id = marker_id++;
    marker_cylinder.type = visualization_msgs::Marker::CYLINDER;
    marker_cylinder.action = visualization_msgs::Marker::ADD;

    // if (std::isnan(point_map_cylinder.point.x) || (point_map_cylinder.point.z > 0.5)) {
    //   return;
    // }

    if (std::isnan(point_map_cylinder.point.x)) {
      return;
    }

    marker_cylinder.pose.position.x = point_map_cylinder.point.x;
    marker_cylinder.pose.position.y = point_map_cylinder.point.y;
    marker_cylinder.pose.position.z = point_map_cylinder.point.z;
    marker_cylinder.pose.orientation.x = coefficients_cylinder->values[3];
    marker_cylinder.pose.orientation.y = coefficients_cylinder->values[4];
    marker_cylinder.pose.orientation.z = coefficients_cylinder->values[5];
    marker_cylinder.pose.orientation.w = 0;
    marker_cylinder.scale.x = coefficients_cylinder->values[6];
    marker_cylinder.scale.y = coefficients_cylinder->values[6];
    marker_cylinder.scale.z = 0.1;
    marker_cylinder.color.r=0.0f;
    marker_cylinder.color.g=1.0f;
    marker_cylinder.color.b=0.0f;
    marker_cylinder.color.a=1.0f;
    marker_cylinder.lifetime = ros::Duration();

    cylinder_candidates_array.markers.push_back(marker_cylinder);
    publish_cylinder_candidates.publish(cylinder_candidates_array);

    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud);
    publish_cylinder_pointcloud.publish (outcloud);

    // std::cerr << "Cylinder RADIUS: " << cyl_radius << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
  }
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "cylinder_segment");
  ros::NodeHandle nh;
  std::cerr << "temnopolti hrvat" << std::endl;


  tf2_ros::TransformListener tf2_listener(tf2_buffer);
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  publish_cylinder_candidates = nh.advertise<visualization_msgs::MarkerArray>("cylinder_markers", 10);
  publish_ring_candidates = nh.advertise<visualization_msgs::MarkerArray>("ring_markers", 10);

  publish_cylinder_pointcloud = nh.advertise<pcl::PCLPointCloud2> ("cloud_cylinder", 1);

  ros::spin ();
}
