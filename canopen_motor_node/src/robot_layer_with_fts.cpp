#include <canopen_motor_node/robot_layer_with_fts.h>
#include <canopen_motor_node/robot_layer.h>

using namespace canopen;
RobotLayerWithFTS::RobotLayerWithFTS(ros::NodeHandle nh) : RobotLayer(nh)
{
    nh.param<std::string>("FTS/fts_name", fts_name, "FTS");
    nh.param<std::string>("Node/transform_frame", fts_transform_frame, "fts_transform_frame");

    registerInterface(&fts_interface_);
}

void RobotLayerWithFTS::handleInit(canopen::LayerStatus &status){
    RobotLayer::handleInit(status);
    
    force_torque_control::ForceTorqueControllerHandle handle("canopen_fts_controller_handle");
    
    ftsh_ = new force_torque_sensor::ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame);
    
    handle_.addHandle(*ftsh_);
    fts_interface_.registerHandle(handle);
}
