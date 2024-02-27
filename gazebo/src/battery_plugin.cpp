#include <gazebo/gazebo.hh>
#include <ros/ros.h>

namespace gazebo
{
    class BatteryPlugin : public gazebo::ModelPlugin
    {
        public: 
            BatteryPlugin() : gazebo::ModelPlugin() {
                std::cout << "Starting battery_plugin" << std::endl;
            }
        
            void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr _sdf) override {
                ROS_INFO("Gazebo plugin has been loaded.");

                _model = parent;

                std::string command = "rosrun catkin_ws battery_node.py";

                int result = std::system(command.c_str());

                if (result != 0)
                {
                gzerr << "Error when loading battery_node.py" << std::endl;
                }
            }
        private:
            gazebo::physics::ModelPtr _model;
    };
    // Gazebo is informed about this plugin
    GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)
}