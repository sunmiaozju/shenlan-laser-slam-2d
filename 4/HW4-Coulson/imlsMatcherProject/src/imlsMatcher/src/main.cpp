#include "champion_nav_msgs/ChampionNavLaserScan.h"
#include "geometry_msgs/Point32.h"
#include "imls_icp.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <boost/foreach.hpp>
#include <csm/csm_all.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");
//此处bag包的地址需要自行修改
std::string bagfile = "/home/sunm/work/code/shenlan-2d/4/HW4/imlsMatcherProject/src/bag/imls_icp.bag";
int count = 0;
class imlsDebug {
public:
    imlsDebug()
    {
        m_imlsPathPub = m_node.advertise<nav_msgs::Path>("imls_path_pub_", 1, true);
        m_imlsPath.header.stamp = ros::Time::now();
        m_imlsPath.header.frame_id = "odom";
        m_odomPathPub = m_node.advertise<nav_msgs::Path>("odom_path_pub_", 1, true);
        m_odomPath.header.stamp = ros::Time::now();
        m_odomPath.header.frame_id = "odom";

        m_isFirstFrame = true;

        rosbag::Bag bag;
        bag.open(bagfile, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("/sick_scan"));
        topics.push_back(std::string("/odom"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        //按顺序读取bag内激光的消息和里程计的消息
        BOOST_FOREACH (rosbag::MessageInstance const m, view) {
            champion_nav_msgs::ChampionNavLaserScanConstPtr scan = m.instantiate<champion_nav_msgs::ChampionNavLaserScan>();
            if (scan != NULL)
                championLaserScanCallback(scan);

            nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if (odom != NULL)
                odomCallback(odom);

            ros::spinOnce();
            if (!ros::ok())
                break;
        }
        // m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
    }

    //将激光消息转换为激光坐标系下的二维点云
    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
        std::vector<Eigen::Vector2d>& eigen_pts)
    {
        eigen_pts.clear();
        for (int i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if (std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx, ly));
        }
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        if (m_isFirstFrame == true) {
            std::cout << "First Frame" << std::endl;
            m_isFirstFrame = false;
            m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            ConvertChampionLaserScanToEigenPointCloud(msg, m_prevPointCloud);
            return;
        }

        std::vector<Eigen::Vector2d> nowPts;
        ConvertChampionLaserScanToEigenPointCloud(msg, nowPts);

        //调用imls进行icp匹配，并输出结果．
        m_imlsMatcher.setSourcePointCloud(nowPts);
        m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

        Eigen::Matrix3d rPose, rCovariance;
        // cout << " num: " << count++ << endl;
        if (m_imlsMatcher.Match(rPose, rCovariance)) {
            // std::cout << "---2---" << std::endl;
            std::cout << "IMLS Match Successful:" << rPose(0, 2) << "," << rPose(1, 2) << "," << atan2(rPose(1, 0), rPose(0, 0)) * 57.295 << std::endl;
            Eigen::Matrix3d lastPose;
            lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                sin(m_prevLaserPose(2)), cos(m_prevLaserPose(2)), m_prevLaserPose(1),
                0, 0, 1;
            Eigen::Matrix3d nowPose = lastPose * rPose;
            m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1, 0), nowPose(0, 0));
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
        } else {
            std::cout << "IMLS Match Failed!!!!" << std::endl;
        }

        m_prevPointCloud = nowPts;
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        if (m_isFirstFrame == true)
            return;

        pubPath(msg, m_odomPath, m_odomPathPub);
    }

    //发布路径消息
    void pubPath(Eigen::Vector3d& pose, nav_msgs::Path& path, ros::Publisher& mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = pose(0);
        this_pose_stamped.pose.position.y = pose(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    void pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path& path, ros::Publisher& mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
        this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

        this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
        this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
        this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
        this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    bool m_isFirstFrame;
    ros::NodeHandle m_nh;
    IMLSICPMatcher m_imlsMatcher;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;
    nav_msgs::Path m_imlsPath;
    nav_msgs::Path m_odomPath;

    tf::TransformListener m_tfListener;
    ros::NodeHandle m_node;

    ros::Subscriber m_laserscanSub;
    ros::Publisher m_imlsPathPub;
    ros::Publisher m_odomPathPub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}
