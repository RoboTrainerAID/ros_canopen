#include <force_controllers/force_controller.h>

#include <ati_force_torque/force_torque_sensor_handle.h>

#include <canopen_motor_node/robot_layer.h>

class RobotLayerWithFTS : public RobotLayer
{
    force_torque_control::ForceTorqueControllerInterface super_interface_;
    ForceTorqueSensorHandle* ftsh_;
    ros::NodeHandle nh_;

    std::string fts_name;
    std::string fts_sensor_frame;
    std::string fts_transform_frame;

public:
    RobotLayerWithFTS(ros::NodeHandle nh);

    virtual void handleInit(canopen::LayerStatus &status);
};
