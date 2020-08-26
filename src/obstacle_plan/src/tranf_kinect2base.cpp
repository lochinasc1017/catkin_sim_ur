#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
using namespace std;

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <ros/duration.h>

// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>






ros::Publisher pub_boject;

void 
transf (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // Create a container for the data.
  sensor_msgs::PointCloud2 scene_pcd;

  ros::Time beg_t = ros::Time::now();

  // Do data processing here...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB> tranf_pcd;

  // pcl::PointCloud<pcl::PointXYZRGB>::iterator index = cloud->begin();   // extract the object above table



  // pcl_conversions::toPCL(*input, *cloud);
  pcl::fromROSMsg(*input, *cloud);

  // ROS_INFO_STREAM("thats ok");

  //coordinate transformation to base_link
  // float M_tf[4][4] = {{-1, 0, 0, 0}, {0, 0.7071, -0.7071, 1.5}, {0, -0.7071, -0.7071, 0.6}, {0, 0, 0, 1}};  // base_link to kincet_depth_link 旋转矩阵
  float M_tf[4][4] = {{-1, 0, 0, 0}, {0, 1, 0, 0.75}, {0, 0, -1, 1.5}, {0, 0, 0, 1}};  // base_link to kincet_depth_link 旋转矩阵

  int po_num = (cloud -> height) * (cloud -> width), ti;
  float po[4];

  // cout<<M_tf[2][3]<<endl;

  std::vector<int> indexs;
  
  for (ti=0; ti<po_num; ti++)
  {

    //init the new point
    float pn[4] = {0, 0, 0, 0};

    //point_data Old
    po[0]=cloud->points[ti].x;
    po[1]=cloud->points[ti].y;
    po[2]=cloud->points[ti].z;
    po[3]=1;

    // matrix multiply
    for (int r=0; r<4; r++)
    {
        for (int c=0; c<4; c++)
        {
            pn[r] += M_tf[r][c] * po[c];
        }
    }

    // New point_data
    cloud->points[ti].x = pn[0];
    cloud->points[ti].y = pn[1];
    cloud->points[ti].z = pn[2];

    if (pn[2] > -0.168)
    {
      indexs.push_back(ti);
    }

  }
  pcl::copyPointCloud(*cloud, indexs, *cloud_out);


  ROS_INFO_STREAM("Scene points reference transform to base_link have done!"); 
  ROS_INFO_STREAM(po_num);  
  // pcl::io::savePCDFile("/home/robot/Documents/scene_points.pcd", *cloud);
    
  pcl::copyPointCloud(*cloud_out, tranf_pcd);


  // the processing time
  ros::Time end_t = ros::Time::now();
  ros::Duration t = end_t - beg_t;
  ROS_INFO_STREAM("The pcd transform time is :" << t.toSec() ); 

  // PointCloudT to ROS msg pointcloud2
  pcl::toROSMsg(tranf_pcd, scene_pcd);

  scene_pcd.header.frame_id = "base_link";

  ros::Rate loop_rate(0.017);
  while (ros::ok())
  {
    // Publish the data.
    pub_boject.publish (scene_pcd);
    ROS_INFO_STREAM("The scene pcd topic publish success!");

    ros::spinOnce();
    loop_rate.sleep();
  }
       

  // Publish the data.
  pub_boject.publish (scene_pcd);

  ROS_INFO_STREAM("The scene pcd topic publish success!");
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "transform_kinect_to_base_link");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect/depth/points", 3, transf);

  // Create a ROS publisher for the output point cloud
  pub_boject = nh.advertise<sensor_msgs::PointCloud2> ("/scene/points", 3);
  
  
  // ros::Rate loop_rate(0.007);
  // the topic rate
  ros::spin();
  
}