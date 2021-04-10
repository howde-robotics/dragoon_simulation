#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

namespace gazebo
{
  class DragoonGazebo : public ModelPlugin
  {
    private: 
        // Pointer to the model
        physics::ModelPtr model;
        physics::Joint_V joints;
        common::PID pid;
        ignition::math::Pose3d pose;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        // ROS interface
        std::unique_ptr<ros::NodeHandle> rosNode;

        // subscriber to the joint value topic
        ros::Subscriber jointSub;

        // callback queue
        ros::CallbackQueue rosQueue;

        // thread that runs the rosQueue
        std::thread rosQueueThread;

        // publish the pose and encoders
        ros::Publisher posePub, odomPub;

        // transform broadcaster
        tf2_ros::TransformBroadcaster br;


    public: 
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            double propGain = 1.0;
            double derGain = 10.0;
            double intGain = 0.0;

            ros::param::get("prop_gain", propGain);
            ros::param::get("der_gain", derGain);
            ros::param::get("int_gain", intGain);

            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DragoonGazebo::OnUpdate, this));
                
            ROS_INFO_STREAM("Dragoon Gazebo API loaded. Connected to: [" << this->model->GetName() << "]\n");

            if (!ros::isInitialized()){
                int argc = 0;
                char ** argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            // create the ros node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // sub to the joint topic using some of the subscriber options availiable
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/" + this->model->GetName() + "/joint_commands", 1, boost::bind(&DragoonGazebo::jointCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            this->jointSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind(&DragoonGazebo::queueThread, this));

            this->posePub = this->rosNode->advertise<geometry_msgs::PoseStamped>("/dragoon/pose", 10);
            this->odomPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/dragoon/odom", 10);

            // collect joints
            this->joints = this->model->GetJoints();
            // set pid

            this->pid = common::PID(propGain, derGain, intGain);


        
        }

        void jointCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
        {
            this->setVelocity(msg->data);
        }

        // a ros queue for messages
        void queueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        // set joint of one wheel for testing
        void setVelocity(std::vector<float> speeds)
        {
            // lidar joint is first, then wheel joints
            for (int i = 0; i < (int) this->joints.size();i++)
            {
                
                physics::JointPtr joint = this->joints[i];
                // ROS_INFO_STREAM(joint->GetName());
                if (joint->GetName() == "lidar_revolute"){
                    joint->SetVelocity(0, 0.0);
                }
                else {
                    double speed = speeds[i];
                    // apply P-controller to the joint
                    this->model->GetJointController()->SetVelocityPID(joint->GetScopedName(), this->pid);
                    this->model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), speed);
                }

            }
        }

        // Called by the world update start event
        void OnUpdate()
        {
            geometry_msgs::PoseStamped poseOut;
            geometry_msgs::TransformStamped transform;
            geometry_msgs::TransformStamped transformLIDAR;

            // publish robot pose
            this->pose = this->model->WorldPose();
            ignition::math::Vector3d pos = this->pose.Pos();
            ignition::math::Quaterniond rot = this->pose.Rot();
            poseOut.pose.position.x = pos.X();
            poseOut.pose.position.y = pos.Y();
            poseOut.pose.position.z = pos.Z();
            poseOut.pose.orientation.x = rot.X();
            poseOut.pose.orientation.y = rot.Y();
            poseOut.pose.orientation.z = rot.Z();
            poseOut.pose.orientation.w = rot.W();
            this->posePub.publish(poseOut);

            // publish transform
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = "world";
            transform.child_frame_id = "dragoon_body";
            transform.transform.translation.x = pos.X();
            transform.transform.translation.y = pos.Y();
            transform.transform.translation.z = pos.Z();
            transform.transform.rotation.x = rot.X();
            transform.transform.rotation.y = rot.Y();
            transform.transform.rotation.z = rot.Z();
            transform.transform.rotation.w = rot.W();
            this->br.sendTransform(transform);

            ignition::math::Pose3d lidarPose = this->joints[0]->WorldPose();
            pos = lidarPose.Pos();
            rot = lidarPose.Rot();

            // publish lidar joint transform
            transformLIDAR.header.stamp = ros::Time::now();
            transform.header.frame_id = "world";
            transform.child_frame_id = "lidar";
            transform.transform.translation.x = pos.X();
            transform.transform.translation.y = pos.Y();
            transform.transform.translation.z = pos.Z();
            transform.transform.rotation.x = rot.X();
            transform.transform.rotation.y = rot.Y();
            transform.transform.rotation.z = rot.Z();
            transform.transform.rotation.w = rot.W();
            this->br.sendTransform(transform);

            // encoder publish the left top and the right bottom wheel
            std_msgs::Float32MultiArray outEncoders;
            std::vector<float> values;
            // returns joint position in radians
            values.push_back(this->model->GetJoint("wheel_1_LF_revolute")->Position());
            values.push_back(this->model->GetJoint("wheel_4_RB_revolute")->Position());
            values.push_back(this->model->GetJoint("wheel_4_RB_revolute")->GetVelocity(0));
            values.push_back(this->model->GetJoint("wheel_4_RB_revolute")->GetVelocity(0));

            outEncoders.data = values;
            odomPub.publish(outEncoders);
        }
        
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DragoonGazebo)
}