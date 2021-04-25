#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>


class GridGenerator
{
    private:
        // map data
        std::string mapFrame_, cloudFrame_;
        geometry_msgs::TransformStamped cloudToMap_;
        geometry_msgs::TransformStamped mapToWorld_;
        geometry_msgs::Pose mapPose_;
        int mapH_;
        int mapW_;
        double mapRes_;
        std::vector<int8_t> mapData_;
        nav_msgs::OccupancyGrid mapMsg_, lastMapMsg_;
        bool verbose_;

        // ros stuff
        ros::NodeHandle nodeHandler_, privateNh_;
        ros::Publisher gridPub_, cloudPrimePub_;
        ros::Subscriber cloudSub_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;

    public:
        GridGenerator();

        // main run function if needed
        void run();

        // callback for the point cloud note that ROS sensormsg to pcl pointcloud automatically
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg);

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

        /**
         * @brief Projects point cloud points onto the XY plane so that they can be put into occ grid. The resulting point cloud is also given in the map frame coordinates.
         *
         * @param cloudIn input cloud
         * @param cloudOut output cloud 
         * @param plane height in z coordinates to project to
         */
        void cloudProjection(const pcl::PCLPointCloud2ConstPtr & cloudIn, pcl::PCLPointCloud2::Ptr & cloudOut, double plane);

        /**
         * @brief Constructs the occupancy grid based off of the 2D point cloud
         *
         */
        void constructOccGrid(const pcl::PCLPointCloud2ConstPtr &cloudIn);

        /**
         * @brief Converts matrix i,j indices into an index of an array that can be indexed
         *
         */
        int convertToIndex(int i, int j, int columns); 

        /**
         * @brief Converts matrix i,j indices into an index of an array that can be indexed
         *
         */
        void placePointOnGrid(const pcl::PointXYZ & point);
};