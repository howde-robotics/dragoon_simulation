#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MarkerNode {

    ros::Publisher markerPub_;
    ros::NodeHandle nodeHandler_;
    geometry_msgs::TransformStamped transform_;
    tf2_ros::Buffer tfBuffer_;
    visualization_msgs::Marker dragoonMarker_;
    std::string dragoonFrame_ = "dragoon_body";


    public:
        MarkerNode(){
            // initialize the listener and the publisher
            tf2_ros::TransformListener tfListener(tfBuffer_);
            markerPub_ = nodeHandler_.advertise<visualization_msgs::Marker>("visual_marker", 0);

        }

        // main run tasks
        void run()
        {
            while (nodeHandler_.ok())
            {
                try
                {
                    // listen for the dragoon transform
                    transform_ = tfBuffer_.lookupTransform(dragoonFrame_, "world", ros::Time(0));
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }

            constructMarker(transform_);
        }

        // this constructs the marker and publishes it
        void constructMarker(const geometry_msgs::TransformStamped &msg)
        {
            dragoonMarker_.header.frame_id = dragoonFrame_;
            dragoonMarker_.header.stamp = ros::Time();
            dragoonMarker_.id = 0;
            dragoonMarker_.type = visualization_msgs::Marker::MESH_RESOURCE;
            dragoonMarker_.action = visualization_msgs::Marker::ADD;
            dragoonMarker_.pose.position.x = transform_.transform.translation.x;
            dragoonMarker_.pose.position.y = transform_.transform.translation.y;
            dragoonMarker_.pose.position.z = transform_.transform.translation.z;
            dragoonMarker_.pose.orientation.x = transform_.transform.rotation.x;
            dragoonMarker_.pose.orientation.y = transform_.transform.rotation.y;
            dragoonMarker_.pose.orientation.z = transform_.transform.rotation.z;
            dragoonMarker_.pose.orientation.w = transform_.transform.rotation.w;
            dragoonMarker_.scale.x = 1.0;
            dragoonMarker_.scale.y = 1.0;
            dragoonMarker_.scale.z = 1.0;
            dragoonMarker_.color.a = 1.0; // Don't forget to set the alpha!
            dragoonMarker_.color.r = 1.0;
            dragoonMarker_.color.g = 0.0;
            dragoonMarker_.color.b = 0.0;
            dragoonMarker_.mesh_resource = "package://meshes/dragoon_base.dae";
            markerPub_.publish(dragoonMarker_);
        }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_node");
  MarkerNode node;
  node.run();
  while (ros::ok())
    ros::spinOnce();
  return 0;

    return 0;
}