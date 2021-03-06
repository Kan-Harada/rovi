#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
//#include <opencv2/core/utility.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rovi/Dialog.h"

ros::NodeHandle *nh;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3[4];

bool change_dir(rovi::Dialog::Request &req, rovi::Dialog::Response &res)
{
  if (chdir(req.hello.c_str()) == 0)
  {
    res.answer = "OK";
    return true;
  }
  else
  {
    res.answer = "Failed";
    return false;
  }
}

bool load_img(rovi::Dialog::Request &req, rovi::Dialog::Response &res)
{
  cv_bridge::CvImage cv_img;
  cv_img.encoding = "mono8";
  cv_img.image = cv::imread(req.hello, CV_LOAD_IMAGE_GRAYSCALE);
  ROS_INFO("tools/imread : %s", req.hello.c_str());
  if (cv_img.image.data != NULL)
  {
    pub1.publish(cv_img.toImageMsg());
    res.answer = "OK";
    return true;
  }
  else
  {
    ROS_ERROR("tools/imread : %s", req.hello.c_str());
    res.answer = "File error";
    return false;
  }
}

bool load_ply(rovi::Dialog::Request &req, rovi::Dialog::Response &res)
{
  cv::Mat pc = cv::ppf_match_3d::loadPLYSimple(req.hello.c_str(), 0);
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = "/camera";
  pts.points.resize(pc.rows);
  pts.channels.resize(1);
  pts.channels[0].name = "intensities";
  pts.channels[0].values.resize(pc.rows);
  for (int i = 0; i < pc.rows; i++)
  {
    pts.points[i].x = pc.at<float>(i, 0);
    pts.points[i].y = pc.at<float>(i, 1);
    pts.points[i].z = pc.at<float>(i, 2);
    pts.channels[0].values[i] = 255;
  }
  pub2.publish(pts);
  return true;
}

bool logger0(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  std_msgs::UInt8 msg;
  msg.data=req.data;
  pub3[0].publish(msg);
  return res.success=true;
}
bool logger1(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  std_msgs::UInt8 msg;
  msg.data=req.data;
  pub3[1].publish(msg);
  return res.success=true;
}
bool logger2(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  std_msgs::UInt8 msg;
  msg.data=req.data;
  pub3[2].publish(msg);
  return res.success=true;
}
bool logger3(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  std_msgs::UInt8 msg;
  msg.data=req.data;
  pub3[4].publish(msg);
  return res.success=true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tools_node");
  ros::NodeHandle n;
  nh = &n;
  pub1 = n.advertise<sensor_msgs::Image>("tools/image", 1);
  pub2 = n.advertise<sensor_msgs::PointCloud>("tools/pcl", 1);
  for(int i=0;i<4;i++){
    char s[20];
    sprintf(s,"tools/log%d",i);
    pub3[i] = n.advertise<std_msgs::UInt8>(s, 1);
  }
  ros::ServiceServer svc0 = n.advertiseService("tools/cd", change_dir);
  ros::ServiceServer svc1 = n.advertiseService("tools/imread", load_img);
  ros::ServiceServer svc2 = n.advertiseService("tools/plread", load_ply);
  ros::ServiceServer svc30 = n.advertiseService("tools/logger0", logger0);
  ros::ServiceServer svc31 = n.advertiseService("tools/logger1", logger1);
  ros::ServiceServer svc32 = n.advertiseService("tools/logger2", logger2);
  ros::ServiceServer svc33 = n.advertiseService("tools/logger3", logger3);
  ros::spin();
  return 0;
}
