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
#include <algorithm>    // std::min

// cylinder candidates
ros::Publisher publish_cylinder_candidates, publish_ring_candidates, publish_cylinder_pointcloud, publish_ring_pointcloud;
visualization_msgs::MarkerArray cylinder_candidates_array, ring_candidates_array;

tf2_ros::Buffer tf2_buffer;
int marker_id = 0;

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int yellow_lower_r = 60, yellow_lower_g = 60, yellow_lower_b = 30;
int yellow_upper_r = 120, yellow_upper_g = 120, yellow_upper_b = 80;
int green_lower_r = 20, green_lower_g = 70, green_lower_b = 20;
int green_upper_r = 60, green_upper_g = 150, green_upper_b = 60;
int blue_lower_r = 30, blue_lower_g = 50, blue_lower_b = 70;
int blue_upper_r = 70, blue_upper_g = 90, blue_upper_b = 120;
int red_lower_r = 100, red_lower_g = 30, red_lower_b = 30;
int red_upper_r = 160, red_upper_g = 100, red_upper_b = 100;
bool red_detected = false, yellow_detected = false, green_detected = false, blue_detected = false;

void
cloud_cb_arm (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  std::cerr << "CIGO____1 CIGO_____1 CIGO_____1" << std::endl;

  ros::Time time_rec_0, time_rec_1, time_test;
  time_rec_0 = ros::Time::now();
  time_rec_1 = ros::Time::now();

  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  Eigen::Vector4f centroid_ring;

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
  //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5, 0.1);
  pass.filter(*cloud_filtered);

  //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  ////////////////////////////////////
  ////////////////////////////////////

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);

  // seg.setNormalDistanceWeight (0.1);
  // seg.setMaxIterations (10000);
  // seg.setDistanceThreshold (0.05);

  seg.setNormalDistanceWeight (0.01);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.06, 0.2);

  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  seg.segment (*inliers_ring, *coefficients_ring);

  // // std::cerr << "Ring coefficients: " << *coefficients_ring << std::endl;

  extract.setNegative (false);
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_ring);
  extract.filter (*cloud_ring);

  ////////////////////////////////////
  ////////////////////////////////////

  if(cloud_ring->points.empty ()) {
    std::cerr << "Can't find the ring component." << std::endl;
  }
  else {
    pcl::compute3DCentroid (*cloud_ring, centroid_ring);

    geometry_msgs::PointStamped point_camera_ring;
    geometry_msgs::PointStamped point_map_ring;
    visualization_msgs::Marker marker_ring;
    geometry_msgs::TransformStamped tss_ring;

    point_camera_ring.header.frame_id = "camera_rgb_optical_frame";
    point_camera_ring.header.stamp = ros::Time::now();
    point_map_ring.header.frame_id = "map";
    point_map_ring.header.stamp = ros::Time::now();
		point_camera_ring.point.x = centroid_ring[0];
		point_camera_ring.point.y = centroid_ring[1];
		point_camera_ring.point.z = centroid_ring[2];

	  try{
      tss_ring = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec_1);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform warning: %s\n", ex.what());
    }

    tf2::doTransform(point_camera_ring, point_map_ring, tss_ring);

    marker_ring.header.frame_id = "map";
    marker_ring.header.stamp = ros::Time::now();
    marker_ring.ns = "ring";
    marker_ring.id = marker_id++;
    marker_ring.type = visualization_msgs::Marker::ARROW;
    marker_ring.action = visualization_msgs::Marker::ADD;

    if (std::isnan(point_map_ring.point.x)) {
      return;
    }

    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2 (*cloud_ring, outcloud);
    publish_ring_pointcloud.publish (outcloud);

    marker_ring.pose.position.x = point_map_ring.point.x;
    marker_ring.pose.position.y = point_map_ring.point.y;
    marker_ring.pose.position.z = point_map_ring.point.z;
    marker_ring.pose.orientation.x = coefficients_ring->values[3];
    marker_ring.pose.orientation.y = coefficients_ring->values[4];
    marker_ring.pose.orientation.z = coefficients_ring->values[5];
    marker_ring.pose.orientation.w = 0;
    marker_ring.scale.x = coefficients_ring->values[6];
    marker_ring.scale.y = coefficients_ring->values[6];
    marker_ring.scale.z = 0.1;

    marker_ring.color.r=1.0f;
    marker_ring.color.g=0.0f;
    marker_ring.color.b=0.0f;
    marker_ring.color.a=1.0f;
    marker_ring.lifetime = ros::Duration();

    ring_candidates_array.markers.push_back(marker_ring);
    publish_ring_candidates.publish(ring_candidates_array);
  
    std::cerr << "Ring FOUND" << std::endl;
    std::cerr << "Ring FOUND" << std::endl;
    std::cerr << "Ring FOUND" << std::endl;
    std::cerr << "Ring FOUND" << std::endl;
    std::cerr << "Ring FOUND" << std::endl;
  }
}

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
  //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

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

  ////////////////////////////////////
  ////////////////////////////////////

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.01);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
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
  // seg.setInputCloud (cloud_filtered2);
  // seg.setInputNormals (cloud_normals2);
  // seg.segment (*inliers_ring, *coefficients_ring);

  // // // std::cerr << "Ring coefficients: " << *coefficients_ring << std::endl;

  // extract.setNegative (false);
  // extract.setInputCloud (cloud_filtered2);
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

    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud);
    publish_cylinder_pointcloud.publish (outcloud);

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

    marker_cylinder.color.a=1.0f;
    marker_cylinder.lifetime = ros::Duration();


    // Initialize accumulated RGB values to zero
    float r_total = 0.0, g_total = 0.0, b_total = 0.0;
    int num_points = cloud_cylinder->points.size();

    // std::cerr << "COLOR IS: " << cloud_cylinder->points[1] << std::endl;
    // Loop through each point and accumulate RGB values
    for (int i = 0; i < num_points; i++) {
      r_total += cloud_cylinder->points[i].r;
      g_total += cloud_cylinder->points[i].g;
      b_total += cloud_cylinder->points[i].b;
    }

    // Divide accumulated RGB values by the number of points to get the average
    float r_avg = r_total / num_points;
    float g_avg = g_total / num_points;
    float b_avg = b_total / num_points;




    Eigen::Vector3f c_avg(r_avg,g_avg,b_avg);
    Eigen::Vector3f R(255,0,0);
    Eigen::Vector3f G(0,255,0);
    Eigen::Vector3f B(0,0,255);
    Eigen::Vector3f Y(255,255,0);
    Eigen::Vector3f gray(128,128,128);

    double d_R=(c_avg-R).norm();
    double d_G=(c_avg-G).norm();
    double d_B=(c_avg-B).norm();
    double d_Y=(c_avg-Y).norm();
    double d_gray=(c_avg-gray).norm();

    double c_min=std::min(1e6,d_R);
    c_min=std::min(c_min,d_G);
    c_min=std::min(c_min,d_B);
    c_min=std::min(c_min,d_Y);
    c_min=std::min(c_min,d_gray);

    std::cerr << "AVERAGE COLOR IS: " << r_avg << ", " << g_avg << ", " << b_avg << std::endl;

    // if (r_avg >= yellow_lower_r && r_avg <= yellow_upper_r &&
    //     g_avg >= yellow_lower_g && g_avg <= yellow_upper_g &&
    //     b_avg >= yellow_lower_b && b_avg <= yellow_upper_b) {
    if(c_min==d_Y) {
        std::cout << "Yellow detected!" << std::endl;
        if (!yellow_detected) {
          marker_cylinder.color.r = 1.0f;
          marker_cylinder.color.g = 1.0f;
          marker_cylinder.color.b = 0.0f;
          cylinder_candidates_array.markers.push_back(marker_cylinder);
          publish_cylinder_candidates.publish(cylinder_candidates_array);
          yellow_detected = true;
          std::string cmd = "aplay /home/gal/ROS/src/exercise6/yellow_cylinder.wav";
          std::system(cmd.c_str());
        }
    } 
    // else if (r_avg >= green_lower_r && r_avg <= green_upper_r &&
    //            g_avg >= green_lower_g && g_avg <= green_upper_g &&
    //            b_avg >= green_lower_b && b_avg <= green_upper_b) {
    else if(c_min==d_G) {
        std::cout << "Green detected!" << std::endl;
        if (!green_detected) {
          marker_cylinder.color.r = 0.0f;
          marker_cylinder.color.g = 1.0f;
          marker_cylinder.color.b = 0.0f;
          cylinder_candidates_array.markers.push_back(marker_cylinder);
          publish_cylinder_candidates.publish(cylinder_candidates_array);
          green_detected = true;
          std::string cmd = "aplay /home/gal/ROS/src/exercise6/greencylinder.wav";
          std::system(cmd.c_str());
        }
    } 
    // else if (r_avg >= blue_lower_r && r_avg <= blue_upper_r &&
    //            g_avg >= blue_lower_g && g_avg <= blue_upper_g &&
    //            b_avg >= blue_lower_b && b_avg <= blue_upper_b) {
    else if(c_min==d_B) {
        std::cout << "Blue detected!" << std::endl;
        if (!blue_detected) {
          marker_cylinder.color.r = 0.0f;
          marker_cylinder.color.g = 0.0f;
          marker_cylinder.color.b = 1.0f;
          cylinder_candidates_array.markers.push_back(marker_cylinder);
          publish_cylinder_candidates.publish(cylinder_candidates_array);
          blue_detected = true;
          std::string cmd = "aplay /home/gal/ROS/src/exercise6/blue_cylinder.wav";
          std::system(cmd.c_str());
        }
    } 
    // else if (r_avg >= red_lower_r && r_avg <= red_upper_r &&
    //            g_avg >= red_lower_g && g_avg <= red_upper_g &&
    //            b_avg >= red_lower_b && b_avg <= red_upper_b) {
    else if(c_min==d_R) {
        std::cout << "Red detected!" << std::endl;
        if (!red_detected) {
          marker_cylinder.color.r = 1.0f;
          marker_cylinder.color.g = 0.0f;
          marker_cylinder.color.b = 0.0f;
          cylinder_candidates_array.markers.push_back(marker_cylinder);
          publish_cylinder_candidates.publish(cylinder_candidates_array);
          red_detected = true;
          std::string cmd = "aplay /home/gal/ROS/src/exercise6/red_cylinder.wav";
          std::system(cmd.c_str());
        }
    } 
    // else {
    else if(c_min==d_gray) {    
        std::cout << "No color detected." << std::endl;
    }

    // Create an RGB color object from the average values
    //pcl::RGB avg_color(r_avg, g_avg, b_avg);

    // std::cerr << "Cylinder RADIUS: " << cyl_radius << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
    std::cerr << "Cylinder FOUND" << std::endl;
  }

  // if(cloud_ring->points.empty ()) {
  //   std::cerr << "Can't find the ring component." << std::endl;
  // }
  // else {
  //   pcl::compute3DCentroid (*cloud_ring, centroid_ring);

  //   geometry_msgs::PointStamped point_camera_ring;
  //   geometry_msgs::PointStamped point_map_ring;
  //   visualization_msgs::Marker marker_ring;
  //   geometry_msgs::TransformStamped tss_ring;

  //   point_camera_ring.header.frame_id = "camera_rgb_optical_frame";
  //   point_camera_ring.header.stamp = ros::Time::now();
  //   point_map_ring.header.frame_id = "map";
  //   point_map_ring.header.stamp = ros::Time::now();
	// 	point_camera_ring.point.x = centroid_ring[0];
	// 	point_camera_ring.point.y = centroid_ring[1];
	// 	point_camera_ring.point.z = centroid_ring[2];

	//   try{
  //     tss_ring = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec_1);
  //   }
  //   catch (tf2::TransformException &ex)
  //   {
  //     ROS_WARN("Transform warning: %s\n", ex.what());
  //   }

  //   tf2::doTransform(point_camera_ring, point_map_ring, tss_ring);

  //   marker_ring.header.frame_id = "map";
  //   marker_ring.header.stamp = ros::Time::now();
  //   marker_ring.ns = "ring";
  //   marker_ring.id = marker_id++;
  //   marker_ring.type = visualization_msgs::Marker::ARROW;
  //   marker_ring.action = visualization_msgs::Marker::ADD;

  //   if (std::isnan(point_map_ring.point.x)) {
  //     return;
  //   }

  //   pcl::PCLPointCloud2 outcloud;
  //   pcl::toPCLPointCloud2 (*cloud_ring, outcloud);
  //   publish_ring_pointcloud.publish (outcloud);

  //   marker_ring.pose.position.x = point_map_ring.point.x;
  //   marker_ring.pose.position.y = point_map_ring.point.y;
  //   marker_ring.pose.position.z = point_map_ring.point.z;
  //   marker_ring.pose.orientation.x = coefficients_ring->values[4];
  //   marker_ring.pose.orientation.y = coefficients_ring->values[5];
  //   marker_ring.pose.orientation.z = coefficients_ring->values[6];
  //   marker_ring.pose.orientation.w = 0;
  //   marker_ring.scale.x = coefficients_ring->values[3];
  //   marker_ring.scale.y = coefficients_ring->values[3];
  //   marker_ring.scale.z = 0.1;





  //   // if (std::isnan(point_map_ring.point.x) || (point_map_ring.point.z > 0.5)) {
  //   //   return;
  //   // }

  //   // marker_ring.pose.position.x = point_map_ring.point.x;
  //   // marker_ring.pose.position.y = point_map_ring.point.y;
  //   // marker_ring.pose.position.z = point_map_ring.point.z;
  //   // marker_ring.pose.orientation.x = 0.0;
  //   // marker_ring.pose.orientation.y = 0.0;
  //   // marker_ring.pose.orientation.z = 0.0;
  //   // marker_ring.pose.orientation.w = 1.0;
  //   // marker_ring.scale.x = 0.1;
  //   // marker_ring.scale.y = 0.1;
  //   // marker_ring.scale.z = 0.1;


  //   marker_ring.color.r=1.0f;
  //   marker_ring.color.g=0.0f;
  //   marker_ring.color.b=0.0f;
  //   marker_ring.color.a=1.0f;
  //   marker_ring.lifetime = ros::Duration();

  //   ring_candidates_array.markers.push_back(marker_ring);
  //   publish_ring_candidates.publish(ring_candidates_array);
  
  //   std::cerr << "Ring FOUND" << std::endl;
  //   std::cerr << "Ring FOUND" << std::endl;
  //   std::cerr << "Ring FOUND" << std::endl;
  //   std::cerr << "Ring FOUND" << std::endl;
  //   std::cerr << "Ring FOUND" << std::endl;
  // }

}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "cylinder_segment");
  ros::NodeHandle nh;
  std::cerr << "temnopolti hrvat" << std::endl;


  tf2_ros::TransformListener tf2_listener(tf2_buffer);
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub_arm = nh.subscribe ("/arm_camera/depth/points", 1, cloud_cb_arm);

  publish_cylinder_candidates = nh.advertise<visualization_msgs::MarkerArray>("cylinder_markers", 10);
  publish_ring_candidates = nh.advertise<visualization_msgs::MarkerArray>("ring_markers", 10);

  publish_cylinder_pointcloud = nh.advertise<pcl::PCLPointCloud2> ("cloud_cylinder", 1);
  publish_ring_pointcloud = nh.advertise<pcl::PCLPointCloud2> ("cloud_ring", 1);

  ros::spin ();
}
