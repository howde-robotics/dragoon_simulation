#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MarkerNode {

    ros::Publisher markerPub_;
    ros::NodeHandle nodeHandler_, nodePrivate_;
    geometry_msgs::TransformStamped transform_;
    tf2_ros::Buffer tfBuffer_;
    visualization_msgs::Marker dragoonMarker_;
    std::string markerFrame_;


    public:
        MarkerNode() : nodePrivate_("~")
        {
            // initialize the listener and the publisher
            markerPub_ = nodeHandler_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
            nodePrivate_.param<std::string>("marker_frame", markerFrame_, "dragoon_body");

        }

        // main run tasks
        void run()
        {
            constructMarker(transform_);
        }

        // this constructs the marker and publishes it
        void constructMarker(const geometry_msgs::TransformStamped &msg)
        {
            dragoonMarker_.header.frame_id = markerFrame_;
            dragoonMarker_.header.stamp = ros::Time();
            dragoonMarker_.id = 0;
            dragoonMarker_.type = visualization_msgs::Marker::MESH_RESOURCE;
            dragoonMarker_.action = visualization_msgs::Marker::ADD;
            dragoonMarker_.pose.position.x = 0;
            dragoonMarker_.pose.position.y = 0;
            dragoonMarker_.pose.position.z = 0;
            dragoonMarker_.pose.orientation.x = 0;
            dragoonMarker_.pose.orientation.y = 0;
            dragoonMarker_.pose.orientation.z = 0;
            dragoonMarker_.pose.orientation.w = 1;
            // dragoonMarker_.pose.position.x = transform_.transform.translation.x;
            // dragoonMarker_.pose.position.y = transform_.transform.translation.y;
            // dragoonMarker_.pose.position.z = transform_.transform.translation.z;
            // dragoonMarker_.pose.orientation.x = transform_.transform.rotation.x;
            // dragoonMarker_.pose.orientation.y = transform_.transform.rotation.y;
            // dragoonMarker_.pose.orientation.z = transform_.transform.rotation.z;
            // dragoonMarker_.pose.orientation.w = transform_.transform.rotation.w;
            dragoonMarker_.scale.x = 1.0;
            dragoonMarker_.scale.y = 1.0;
            dragoonMarker_.scale.z = 1.0;
            dragoonMarker_.color.a = 1.0; // Don't forget to set the alpha!
            dragoonMarker_.color.r = 1.0;
            dragoonMarker_.color.g = 94/255;
            dragoonMarker_.color.b = 19/255;
            // use the dragoon mesh
            dragoonMarker_.mesh_resource = "package://dragoon_simulation/meshes/dragoon_base.dae";
            markerPub_.publish(dragoonMarker_);
        }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "marker_node");
  MarkerNode node;
  
  while (ros::ok())
    node.run();
    ros::spinOnce();
  return 0;

    return 0;
}