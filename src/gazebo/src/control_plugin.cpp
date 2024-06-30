#include <gazebo/gazebo.hh>
#include <ros/ros.h>

namespace gazebo
{
    class ControlPlugin : public gazebo::ModelPlugin
    {
        public: 
            ControlPlugin() : gazebo::ModelPlugin() {
                std::cout << "Starting control_plugin" << std::endl;
            }
        
            void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr _sdf) override {
                ROS_INFO("Gazebo plugin has been loaded.");

                _model = parent;

                // ros::NodeHandle nh;
                // ros::ServiceClient startControlClient = nh.serviceClient<std_srvs::Empty>("/start_drone_control");

                std::string command = "rosrun catkin_ws control_node.py";

                int result = std::system(command.c_str());

                if (result != 0)
                {
                gzerr << "Error when loading control_node.py" << std::endl;
                }
            }
        private:
            gazebo::physics::ModelPtr _model;
    };
    // Gazebo is informed about this plugin
    GZ_REGISTER_MODEL_PLUGIN(ControlPlugin) 
}