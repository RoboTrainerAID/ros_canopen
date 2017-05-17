#include <ati_force_torque/force_torque_sensor_handle.h>
#include <canopen_motor_node/robot_layer.h>
//#include <hardware_interface/force_torque_sensor_interface.h>
namespace canopen {

class RobotLayerWithFTS : public RobotLayer
{
    hardware_interface::ForceTorqueSensorInterface ft_interface_;
    ForceTorqueSensorHandle* ftsh_;
    ros::NodeHandle nh_;

    std::string fts_name;
    std::string fts_sensor_frame;
    std::string fts_transform_frame;

public:
    RobotLayerWithFTS(ros::NodeHandle nh);

    virtual void handleInit(canopen::LayerStatus &status);
};
}
