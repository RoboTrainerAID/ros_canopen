#include <canopen_motor_node/robot_layer_with_fts.h>
#include <canopen_motor_node/robot_layer.h>


RobotLayerWithFTS::RobotLayerWithFTS(ros::NodeHandle nh) : RobotLayer(nh)
{
    nh.param<std::string>("FTS/name", fts_name, "fts_sensor_name");
    nh.param<std::string>("Node/transform_frame", fts_transform_frame, "fts_frame");
    registerInterface(&super_interface_);
}

void RobotLayerWithFTS::handleInit(canopen::LayerStatus &status){
    RobotLayer::handleInit(status);

    ftsh_ = new ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame);

    force_torque_control::ForceTorqueControllerHandle handle("ForceTorqueControllerHandle");
    handle.addHandle(*ftsh_);

    std::vector<std::string> vel_names = vel_interface_.getNames();
    for(size_t i = 0; i < vel_names.size(); i ++)
        handle.addVelHandle(vel_interface_.getHandle(vel_names[i]));

    std::vector<std::string> pos_names = pos_interface_.getNames();
    for(size_t i = 0; i < pos_names.size(); i ++)
        handle.addPosHandle(pos_interface_.getHandle(pos_names[i]));

    super_interface_.registerHandle(handle);
}
