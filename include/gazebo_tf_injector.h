#ifndef GAZEBO_TF_INJECTOR_H_
#define GAZEBO_TF_INJECTOR_H_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "ros/ros.h"
#include "tf/transform_listener.h"


namespace gazebo {
    class GazeboTfInjector : public ModelPlugin {
        private:
            physics::ModelPtr model;
            event::ConnectionPtr callback;
            ros::NodeHandlePtr nodeHandle;
            std::shared_ptr<tf::TransformListener> tfListener;
            std::map<std::string, physics::LinkPtr> childLinks;
            std::string parentFrame;


        public:
            void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

            void InjectTf();
    };
}

#endif
