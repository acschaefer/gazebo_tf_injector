#include "gazebo_tf_injector.h"
#include "ignition/math.hh"
#include <thread>
#include <chrono>


namespace gazebo {
    // Register the plugin with the Gazebo server.
    GZ_REGISTER_MODEL_PLUGIN(GazeboTfInjector)
    
    
    void GazeboTfInjector::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
        // Wait until the roscore is running and create a ROS node.
        std::string nodeName = "gazebo_tf_injector";
        ROS_INFO_STREAM("Starting ROS node \"" << nodeName << "\" ...");
        this->nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle(nodeName));
        ROS_INFO("Node started.");
        this->nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle());

        this->model = model;

        // Create a list of all links of the model.
        ROS_INFO_STREAM("Retrieving child links of model \"" 
            << this->model->GetName() << "\" ...");
        for (unsigned int i = 0u; i < model->GetChildCount(); i++) {
            std::string childName = model->GetChild(i)->GetName();
            physics::LinkPtr link = model->GetChildLink(childName);
            if (link) {
                this->childLinks[childName] = link;
            }
        }

        // Make sure the model comprises at least one link.
        if (this->childLinks.empty()) {
            ROS_FATAL("No links found.");
            return;
        }
        
        // Print the names of the detected links.
        std::stringstream linkMessage;
        linkMessage << "Found the following child links: ";
        for (std::pair<std::string, physics::LinkPtr> link : this->childLinks) {
            linkMessage << std::endl << " - " << link.first;
        }
        ROS_INFO_STREAM(linkMessage.str());

        // Create a transform listener that can retrieve the poses of all
        // links in the ROS TF tree.
        this->tfListener = std::make_shared<tf::TransformListener>(
            *(this->nodeHandle));
        
        // Retrieve the root TF frame.
        ROS_INFO("Retrieving root TF frame ...");
        std::string childFrame = this->childLinks.begin()->first;
        std::string parentFrame = childFrame;
        while (childFrame != parentFrame 
                || this->childLinks.count(parentFrame) > 0) {
            childFrame = parentFrame;
            if (!this->tfListener->getParent(
                        childFrame, ros::Time(0), parentFrame)) {
                ROS_INFO_STREAM_ONCE("Waiting for parent of frame \""
                    << childFrame << "\" to become available ...");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        this->rootFrame = parentFrame;
        ROS_INFO_STREAM("Found root TF frame: \"" << this->rootFrame << "\".");

        // Register the callback that updates the model poses every time the
        // simulated world is rendered.
        ROS_INFO(
            "Registering callback to inject TF poses into Gazebo actor ...");
        this->callback = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboTfInjector::InjectTf, this));
    }


    void GazeboTfInjector::InjectTf()
    {
        // Iterate over all links of the robot, look up their poses in the ROS
        // TF tree, and set their values accordingly.
        ROS_INFO_STREAM_ONCE("Injecting TF poses into Gazebo actor \""
            << this->model->GetName() << "\" ...");
        for (std::pair<std::string, physics::LinkPtr> link : this->childLinks) {
            // Retrieve the link pose.
            tf::StampedTransform transform;
            try {
                this->tfListener->lookupTransform(
                    this->rootFrame, link.first, ros::Time(0), transform);
            } catch (const tf::TransformException & exception) {
                ROS_ERROR_STREAM_THROTTLE(0.1, "Failed to update link \"" 
                    << link.first << "\": " << exception.what() << ".");
                continue;
            }

            // Set the link pose.
            tf::Vector3 pos(transform.getOrigin());
            tf::Quaternion rot(transform.getRotation());
            ignition::math::Pose3d pose(pos.getX(), pos.getY(), pos.getZ(),
                rot.getW(), rot.getX(), rot.getY(), rot.getZ());
            link.second->SetWorldPose(pose);
        }
    }
}
