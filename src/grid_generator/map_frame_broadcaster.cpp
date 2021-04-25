#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    // publish a static transform that is the map frame
    ros::init(argc, argv, "map_frame_broadcaster");
    ros::NodeHandle nH;
    static tf2_ros::StaticTransformBroadcaster tfBroadcaster;

    int mapH, mapW;
    double mapRes;

    // get information from the param file
    nH.param<int>("/grid_generator/map_height", mapH, 1000);
    nH.param<int>("/grid_generator/map_width", mapW, 1000);
    nH.param<double>("/grid_generator/map_res", mapRes,0.01);

    geometry_msgs::TransformStamped mapToWorld;
        // map frame is different than world frame so add to the TF tree
    mapToWorld.header.stamp = ros::Time::now();
    mapToWorld.header.frame_id = "world";
    mapToWorld.child_frame_id = "map";
    mapToWorld.transform.translation.x = -(mapW*mapRes)/2.0;
    mapToWorld.transform.translation.y = -(mapH*mapRes)/2.0;
    mapToWorld.transform.translation.z = 0;
    mapToWorld.transform.rotation.x = 0;
    mapToWorld.transform.rotation.y = 0;
    mapToWorld.transform.rotation.z = 0;
    mapToWorld.transform.rotation.w = 1;
    tfBroadcaster.sendTransform(mapToWorld);

    ros::spin();

    return 0;
}
