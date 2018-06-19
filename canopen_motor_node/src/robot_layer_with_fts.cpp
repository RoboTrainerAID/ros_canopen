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
    
    ftsh_ = new ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame);
    
    std::vector<std::string> pos_names = position_joint_interface_.getNames();
    for(size_t i = 0; i < pos_names.size(); i ++)
        handle.addPosHandle(position_joint_interface_.getHandle(pos_names[i]));
    ROS_DEBUG_STREAM("add position handle");
    std::vector<std::string> vel_names = velocity_joint_interface_.getNames();
    for(size_t i = 0; i < vel_names.size(); i ++)
        handle.addVelHandle(velocity_joint_interface_.getHandle(vel_names[i]));
    ROS_DEBUG_STREAM("add velocity handle");
    
    handle_.addHandle(*ftsh_);
    fts_interface_.registerHandle(handle);
}
