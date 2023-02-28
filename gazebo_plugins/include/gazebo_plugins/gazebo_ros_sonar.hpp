#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_

#include <string>

#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/SonarPlugin.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <gazebo_plugins/PubQueue.h>

// using namespace gazebo;

namespace gazebo_plugins
{

    class GazeboRosSonar : public gazebo::SonarPlugin
    {

    public:
        /// \brief Constructor
        GazeboRosSonar();

        /// \brief Destructor
        ~GazeboRosSonar();

        /// \brief Load the plugin
        /// \param take in SDF root element
        void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) override;

    private:
        /// \brief Keep track of number of connctions
        int sonar_connect_count_;

        void SonarConnect();

        void SonarDisconnect();

        // Pointer to the model
        // GazeboRosPtr gazebo_ros_;
        std::string world_name_;

        gazebo::physics::WorldPtr world_;
        /// \brief The parent sensor
        gazebo::sensors::SonarSensorPtr parent_sonar_sensor_;

        /// \brief pointer to ros node
        rclcpp::Node::SharedPtr rosnode_;

        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;

        PubQueue<sensor_msgs::msg::Range>::Ptr pub_queue_;

        /// \brief topic name
        std::string topic_name_;

        /// \brief frame transform name, should match link name
        std::string frame_name_;

        /// \brief tf prefix
        std::string tf_prefix_;

        /// \brief for setting ROS name space
        std::string robot_namespace_;

        /// \brief sensor field of view
        double fov_;
        /// \brief Gaussian noise
        double gaussian_noise_;

        /// \brief Gaussian noise generator
        double GaussianKernel(double mu, double sigma);

        /// update rate of this sensor
        double update_rate_;

        double update_period_;

        gazebo::common::Time last_update_time_;

        // deferred load in case ros is blocking
        sdf::ElementPtr sdf;

        void LoadThread();

        boost::thread deferred_load_thread_;

        unsigned int seed;

        gazebo::transport::NodePtr gazebo_node_;

        gazebo::transport::SubscriberPtr sonar_sub_;

        /// \brief Update the controller
        void OnScan(ConstSonarStampedPtr &_msg);

        /// \brief prevents blocking
        PubMultiQueue pmq;
    };

    /**
     * Accessing model name like suggested by nkoenig at http://answers.gazebosim.org/question/4878/multiple-robots-with-ros-plugins-sensor-plugin-vs/
     * @param parent
     * @return accessing model name
     **/
    inline std::string GetModelName(const gazebo::sensors::SensorPtr &parent)
    {
        std::string modelName;
        std::vector<std::string> values;
        std::string scopedName = parent->ScopedName();
        boost::replace_all(scopedName, "::", ",");
        boost::split(values, scopedName, boost::is_any_of(","));
        if (values.size() < 2)
        {
            modelName = "";
        }
        else
        {
            modelName = values[1];
        }
        return modelName;
    }

    /**
     * @brief Reads the name space tag of a sensor plugin
     * @param parent
     * @param sdf
     * @param pInfo
     * @return node namespace
     **/
    inline std::string GetRobotNamespace(const gazebo::sensors::SensorPtr &parent, const sdf::ElementPtr &sdf, const char *pInfo = NULL)
    {
        std::string name_space;
        std::stringstream ss;
        if (sdf->HasElement("robotNamespace"))
        {
            name_space = sdf->Get<std::string>("robotNamespace");
            if (name_space.empty())
            {
                ss << "The 'robotNamespace' param was empty";
                name_space = GetModelName(parent);
            }
            else
            {
                ss << "Using the 'robotNamespace' param: '" << name_space << "'";
            }
        }
        else
        {
            ss << "The 'robotNamespace' param did not exit";
        }
        if (pInfo != NULL)
        {
            RCLCPP_INFO(rclcpp::get_logger("utils"),
                        "%s Plugin: %s", pInfo, ss.str().c_str());
        }
        return name_space;
    }
}
#endif // GAZEBO_ROS_RANGE_H

// Adapt the code above for ROS2
