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

namespace gazebo
{
  class DragoonGazebo : public ModelPlugin
  {
    public: 
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            //     std::bind(&DragoonGazebo::OnUpdate, this));
                
            ROS_INFO_STREAM("Dragoon Gazebo API loaded. Connected to: [" << this->model->GetName() << "]\n");

            if (!ros::isInitialized()){
                int argc = 0;
                char ** argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            // collect joints
            this->joints = this->model->GetJoints();
            // set pid
            this->pid = common::PID(1, 10, 0);

            // create the ros node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // sub to the joint topic using some of the subscriber options availiable
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/" + this->model->GetName() + "/joint_commands", 1, boost::bind(&DragoonGazebo::jointCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            this->jointSub = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind(&DragoonGazebo::queueThread, this));
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

            for (int i = 0; i < (int) this->joints.size();i++)
            {
                physics::JointPtr joint = this->joints[i];
                double speed = speeds[i];
                // apply P-controller to the joint
                this->model->GetJointController()->SetVelocityPID(joint->GetScopedName(), this->pid);
                this->model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), speed);

            }
        }

        // Called by the world update start event
        // void OnUpdate()
        // {
            // Apply a small linear velocity to the model.
            // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

            // // retrieve joints
            // this->joint = this->model->GetJoint("wheel_4_RB_revolute");
            // // set PID
            // this->pid = common::PID(0.1, 10, 0);

            // // apply P-controller to the joint
            // this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);
            // this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 0.1);
        // }

    private: 
        // Pointer to the model
        physics::ModelPtr model;
        physics::Joint_V joints;
        common::PID pid;

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
        
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DragoonGazebo)
}