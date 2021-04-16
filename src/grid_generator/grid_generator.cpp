#include "grid_generator.h"

int main (int argc, char**argv){

    ros::init(argc, argv, "grid_generator_node");
    ros::NodeHandle node("~");

    std::string target_frame;
    std::string source_frame;

    // get parameters to mess with the transform lookup
    node.param<std::string>("target_frame", target_frame, "base_link");
    node.param<std::string>("source_frame", source_frame, "base_link");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10);

    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;

        try{
            transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        }

        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        rate.sleep();
    }

    return 0;
}