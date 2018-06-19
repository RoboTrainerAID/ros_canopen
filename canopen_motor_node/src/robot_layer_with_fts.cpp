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
    ftsh_ = new ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame);
    RobotLayer::handleInit(status);
    ft_interface_.registerHandle(*ftsh_);
}
