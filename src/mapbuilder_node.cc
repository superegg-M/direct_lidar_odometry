#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

int i=1;

// 存储平移和旋转信息
geometry_msgs::PoseStamped current_pose;

// 存储点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// 存储完整的地图
pcl::PointCloud<pcl::PointXYZI>::Ptr complete_map(new pcl::PointCloud<pcl::PointXYZI>);

// 创建PCL可视化工具
//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Map Viewer"));
pcl::visualization::CloudViewer viewer("Map Viewer");

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_pose = *msg;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // 将ROS消息转换为PCL点云格式
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *current_cloud);

  // 获取当前位置的变换矩阵
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try
  {
    listener.waitForTransform("robot/odom", "robot/base_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("robot/odom", "robot/base_link", ros::Time(0), transform);
  }
  catch(const std::exception& e)
  {
    pcl::io::savePCDFile("./map.pcd",*complete_map);
    std::cout << "map saved!" << '\n';
  }
  
  

  // 将点云数据转换到全局坐标系中
  pcl_ros::transformPointCloud(*current_cloud, *current_cloud, transform);

  // 将当前点云数据与之前的点云数据合并
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::concatenatePointCloud(*complete_map, *current_cloud, *temp);
  *temp=*complete_map+*current_cloud;
  complete_map = temp;
  i++;
 
  viewer.showCloud(complete_map);

  // // 将点云添加到可视化工具中
  // viewer->addPointCloud<pcl::PointXYZI>(complete_map, std::to_string(i));

  // // 设置点云颜色为绿色
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, std::to_string(i));

  // // 设置点云大小为1.0
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(i));
 
  cout<<"map update!"<<endl;
 
  // 启动可视化工具
  // viewer->spin();


  // 发布完整的地图
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*complete_map, map_msg);

  //map_pub.publish(map_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_builder");
    ros::NodeHandle nh;

    // ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("robot/dlo/odom_node/pose", 10, poseCallback);
    ros::Subscriber point_sub = nh.subscribe<sensor_msgs::PointCloud2>("ouster/points", 10, pointCloudCallback);

    // ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("complete_map", 10);
    
    ros::spin();

    return 0;
}
