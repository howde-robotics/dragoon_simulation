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

    // define output container for the processed point cloud data
    pcl::PCLPointCloud2::Ptr outCloud (new pcl::PCLPointCloud2());

    // remove the floor
    passThroughCloud(cloudMsg, outCloud, "z", -0.2, 1.0);
    
    // publish the data
    cloudPrimePub_.publish(*outCloud);
}

void GridGenerator::pruneCloud(const pcl::PCLPointCloud2ConstPtr & cloudIn, pcl::PCLPointCloud2::Ptr &cloudOut, double leafX, double leafY, double leafZ){

    // dfeine a voxelgrid
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
    // set input to cloud
    voxelGrid.setInputCloud(cloudIn);
    // set the leaf size (x, y, z)
    voxelGrid.setLeafSize(leafX, leafY, leafZ);
    // apply the filter to dereferenced cloudVoxel
    voxelGrid.filter(*cloudOut);
}

void GridGenerator::passThroughCloud(const pcl::PCLPointCloud2ConstPtr & cloudIn, pcl::PCLPointCloud2::Ptr &cloudOut, std::string fieldName, double passLow, double passHigh){
    // define a passthrough filter
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // set input cloud
    pass.setInputCloud(cloudIn);
    // filter along z-axis
    pass.setFilterFieldName(fieldName);
    // set z-limits
    pass.setFilterLimits(passLow, passHigh);
    // apply the filter to the output cloud
    pass.filter(*cloudOut);
}

int main (int argc, char**argv){
    ros::init(argc, argv, "grid_generator_node");
    GridGenerator node;
    while (ros::ok()){
        ros::spinOnce();
    }

    return 0;
}