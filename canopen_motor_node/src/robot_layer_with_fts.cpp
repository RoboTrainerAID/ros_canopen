#include <canopen_motor_node/robot_layer_with_fts.h>
#include <canopen_motor_node/robot_layer.h>

using namespace canopen;
RobotLayerWithFTS::RobotLayerWithFTS(ros::NodeHandle nh) : RobotLayer(nh)
{
    nh.param<std::string>("FTS/name", fts_name, "fts_sensor_name");
    nh.param<std::string>("Node/transform_frame", fts_transform_frame, "fts_frame");

    registerInterface(&ft_interface_);
}

void RobotLayerWithFTS::handleInit(canopen::LayerStatus &status){
    ftsh_ = new ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame);
    RobotLayer::handleInit(status);
    ft_interface_.registerHandle(*ftsh_);
}
