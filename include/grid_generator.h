#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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

        /**
         * @brief Applies a pass through filter to the cloud. Note that the filter discards everything outside of passLow < i < passHigh
         *
         * @param cloudIn input cloud
         * @param cloudOut output cloud 
         * @param leafX x direction leaf size
         * @param leafY y direction leaf size
         * @param leafZ z direction leaf size
         */
        void pruneCloud(const pcl::PCLPointCloud2ConstPtr & cloudIn, pcl::PCLPointCloud2::Ptr &cloudOut, double leafX, double leafY, double leafZ);

        /**
         * @brief Applies a pass through filter to the cloud. Note that the filter discards everything outside of passLow < i < passHigh
         *
         * @param cloudIn input cloud
         * @param cloudOut output cloud 
         * @param fieldName the pass through field which contains the data to be filtered
         * @param passLow low threshold of the passthrough
         * @param passHigh high thresdhold of the passthrough
         */
        void passThroughCloud(const pcl::PCLPointCloud2ConstPtr & cloudIn, pcl::PCLPointCloud2::Ptr &cloudOut, std::string fieldName, double passLow, double passHigh);
};