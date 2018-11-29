#ifndef GAZEBO_TF_INJECTOR_H_
#define GAZEBO_TF_INJECTOR_H_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "ros/ros.h"
#include "tf/transform_listener.h"


namespace gazebo {
    // Continuously updates the poses of the links of the Gazebo actor according
    // to their values in the ROS TF tree.
    //
    // When loading the Gazebo model, this class creates a ROS node. When the
    // model is updated, this node looks up the transformations of the links of
    // the model in the ROS TF tree and sets them accordingly in Gazebo.
    class GazeboTfInjector : public ModelPlugin {
        private:
            // Actor whose links are updated.
            physics::ModelPtr model;

            // Callback object that tells Gazebo which method to call.
            event::ConnectionPtr callback;

            // Handle to the ROS node that is created by this plugin.
            ros::NodeHandlePtr nodeHandle;

            // Object that accesses the ROS TF tree.
            std::shared_ptr<tf::TransformListener> tfListener;

            // Map that contains all links of the model.
            std::map<std::string, physics::LinkPtr> childLinks;

            // Name of the world frame in the ROS TF tree.
            std::string rootFrame;


        public:
            // Creates the ROS node, retrieves the ROS TF root node, finds all
            // links of the model, and registers the callback that continuously
            // updates the link poses.
            void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

            // Looks up the poses of all links of the Gazebo model in the ROS TF
            // tree and updates their poses in Gazebo accordingly.
            void InjectTf();
    };
}

#endif
