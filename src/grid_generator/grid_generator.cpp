#include "grid_generator.h"

GridGenerator::GridGenerator() : privateNh_("~")
{

    // get parameters to mess with the transform lookup
    privateNh_.param<std::string>("map_frame", mapFrame_, "world");

    // sub to the point cloud, publish the occ grid
    cloudSub_ = nodeHandler_.subscribe<pcl::PCLPointCloud2>("/gazebo_api/lidar", 1, &GridGenerator::cloudCallback, this);
    gridPub_ = nodeHandler_.advertise<nav_msgs::OccupancyGrid>("/dragoon/map", 2);
    cloudPrimePub_ = nodeHandler_.advertise<pcl::PCLPointCloud2>("/dragoon/lidar", 2);
}

void GridGenerator::cloudCallback(const pcl::PCLPointCloud2ConstPtr & cloudMsg){
    // point cloud is in the lidar frame
    std::string cloudFrame = cloudMsg->header.frame_id;
    
}

int main (int argc, char**argv){
    ros::init(argc, argv, "grid_generator_node");
    GridGenerator node;
    while (ros::ok()){
        ros::spinOnce();
    }

    return 0;
}