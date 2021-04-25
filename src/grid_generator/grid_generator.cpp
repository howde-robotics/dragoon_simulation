#include "grid_generator.h"

GridGenerator::GridGenerator() : privateNh_("~")
{
    // get parameters to mess with the transform lookup
    privateNh_.param<std::string>("map_frame", mapFrame_, "map");
    privateNh_.param<std::string>("cloud_frame", cloudFrame_, "lidar");
    privateNh_.param<int>("map_height", mapH_, 1000);
    privateNh_.param<int>("map_width", mapW_,1000);
    privateNh_.param<double>("map_res", mapRes_,0.01);
    privateNh_.param<bool>("verbose", verbose_, false);

    mapData_ = std::vector<int8_t>(mapW_*mapH_);

    // sub to the point cloud, publish the occ grid
    cloudSub_ = nodeHandler_.subscribe<sensor_msgs::PointCloud2>("/gazebo_api/lidar", 1, &GridGenerator::cloudCallback, this);
    gridPub_ = nodeHandler_.advertise<nav_msgs::OccupancyGrid>("/dragoon/map", 2);
    cloudPrimePub_ = nodeHandler_.advertise<pcl::PCLPointCloud2>("/dragoon/lidar", 2);

    // initalize all map values to 0
    for (std::vector<int8_t>::iterator it = mapData_.begin(); it != mapData_.end(); ++it){
        *it = 0;
    }
}

void GridGenerator::run(){
    // transform listener is here
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    try{
        cloudToMap_ = tfBuffer.lookupTransform(mapFrame_, cloudFrame_, ros::Time(0), ros::Duration(1.0));
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
    }
}

void GridGenerator::cloudCallback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg){
    // define output container for the processed point cloud data
    pcl::PCLPointCloud2::Ptr outCloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr inCloud (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloudMsg, *inCloud);

    // remove the floor
    passThroughCloud(inCloud, outCloud, "z", -0.2, 1.0);

    cloudProjection(outCloud, outCloud, 0.0);
    
    // publish the data
    cloudPrimePub_.publish(*outCloud);

    constructOccGrid(outCloud);
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

void GridGenerator::cloudProjection(const pcl::PCLPointCloud2ConstPtr & cloudIn, pcl::PCLPointCloud2::Ptr & cloudOut, double plane){
    // iterate over the XYZ points and just set Z to plane height value
    sensor_msgs::PointCloud2 cloudInPrime;
    sensor_msgs::PointCloud2 cloudOutPrime;
    pcl_conversions::fromPCL(*cloudIn, cloudInPrime);

    // projects the points onto the map
    tf2::doTransform(cloudInPrime, cloudOutPrime, cloudToMap_);

    // iterate over the z field in PointCloud2
    sensor_msgs::PointCloud2Iterator<double> iterZ (cloudOutPrime, "z");

    // set the z components to 0
    for (;iterZ != iterZ.end(); ++iterZ){
        *iterZ = plane;
    }

    // convert the adjusted cloud into output cloud
    pcl_conversions::toPCL(cloudOutPrime, *cloudOut);
    
}

void GridGenerator::constructOccGrid(const pcl::PCLPointCloud2ConstPtr &cloudIn){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInPrime(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloudIn, *cloudInPrime);

    // collect the x, y components and use them to adjust occupancy grid cells
    for (auto& point : cloudInPrime->points){
        placePointOnGrid(point);
    }

    // construct map message
    mapPose_.position.x = 0;
    mapPose_.position.y = 0;
    mapMsg_.header.frame_id = mapFrame_;
    mapMsg_.info.origin = mapPose_;
    mapMsg_.info.height = (uint32_t) mapH_;
    mapMsg_.info.width = (uint32_t) mapW_;
    mapMsg_.info.resolution = mapRes_;
    mapMsg_.data = mapData_;

    lastMapMsg_ = mapMsg_;
    gridPub_.publish(mapMsg_);
}

int GridGenerator::convertToIndex(int i, int j, int columns){
    return columns*i + j;
}

void GridGenerator::placePointOnGrid(const pcl::PointXYZ & point){
    // first clip the doubles to 2 decimal places (or whatever resolution allows)
    double x  = std::round(point.x /mapRes_)*mapRes_;
    double y  = std::round(point.y/mapRes_)*mapRes_;
    int i     = x /mapRes_;
    int j     = y /mapRes_;
    int a     = convertToIndex(j, i, mapW_);
    int lowLim = 0;
    int highLim = mapData_.size();

    if (verbose_){
        ROS_INFO_STREAM("x: " << x << " y: " << y << " i: " << i << " j: " << j << " index: " << a);
    }

    // set the appropriate map values to increase by a certain amount
    if (a >= lowLim && a < highLim){
        mapData_[a] += 10;
    }
}


int main (int argc, char**argv){
    ros::init(argc, argv, "grid_generator_node");
    GridGenerator node;
    while (ros::ok()){
        node.run();
        ros::spinOnce();
    }

    return 0;
}