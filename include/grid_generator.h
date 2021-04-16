#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>


class GridGenerator
{
    private:
        // map data
        std::string mapFrame_;
        geometry_msgs::PoseStamped mapPose_;
        nav_msgs::OccupancyGrid occMsg_;

        // ros stuff
        ros::NodeHandle nodeHandler_, privateNh_;
        ros::Publisher gridPub_, cloudPrimePub_;
        ros::Subscriber cloudSub_;

    public:
        GridGenerator();

        // main run function if needed
        void run();

        // callback for the point cloud note that ROS sensormsg to pcl pointcloud automatically
        void cloudCallback(const pcl::PCLPointCloud2ConstPtr & cloudMsg);
};