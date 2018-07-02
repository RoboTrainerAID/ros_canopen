#include <canopen_motor_node/robot_layer_with_fts.h>

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
    
    handle.addHandle(*ftsh_);

    std::vector<std::string> vel_names = vel_interface_.getNames();
    for(size_t i = 0; i < vel_names.size(); i ++)
        handle.addVelHandle(vel_interface_.getHandle(vel_names[i]));

    std::vector<std::string> pos_names = pos_interface_.getNames();
    for(size_t i = 0; i < pos_names.size(); i ++)
        handle.addPosHandle(pos_interface_.getHandle(pos_names[i]));

    fts_interface_.registerHandle(handle);
}
