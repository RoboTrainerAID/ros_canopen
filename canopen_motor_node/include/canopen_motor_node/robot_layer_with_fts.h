#ifndef CANOPEN_MOTOR_NODE_ROBOT_LAYER_WITH_FTS_H_
#define CANOPEN_MOTOR_NODE_ROBOT_LAYER_WITH_FTS_H_

#include <force_controllers/force_controller.h>
#include <force_torque_sensor/force_torque_sensor_handle.h>
#include <canopen_motor_node/robot_layer.h>

using namespace canopen;

class RobotLayerWithFTS : public RobotLayer
{
    force_torque_control::ForceTorqueControllerInterface fts_interface_;
    force_torque_control::ForceTorqueControllerHandle controller_handle_;
    force_torque_sensor::ForceTorqueSensorHandle* ftsh_;
    ros::NodeHandle nh_;

    std::string fts_name;
    std::string fts_sensor_frame;
    std::string fts_transform_frame;

public:
    RobotLayerWithFTS(ros::NodeHandle nh);

    virtual void handleInit(canopen::LayerStatus &status);
};

#endif
