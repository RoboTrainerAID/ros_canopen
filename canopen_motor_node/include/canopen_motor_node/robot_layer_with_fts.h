#ifndef CANOPEN_MOTOR_NODE_ROBOT_LAYER_WITH_FTS_H_
#define CANOPEN_MOTOR_NODE_ROBOT_LAYER_WITH_FTS_H_

#include <force_controllers/force_controller.h>

#include <ati_force_torque/force_torque_sensor_handle.h>
#include <canopen_motor_node/robot_layer.h>


using namespace canopen;

class RobotLayerWithFTS : public RobotLayer
{
    force_torque_control::ForceTorqueControllerInterface fts_interface_;
    ForceTorqueSensorHandle* ftsh_;
    ros::NodeHandle nh_;

    std::string fts_name;
    std::string fts_sensor_frame;
    std::string fts_transform_frame;
    
//     struct SwitchData {
//         boost::shared_ptr<HandleLayerBase> handle;
//         canopen::MotorBase::OperationMode mode;
//         bool enforce_limits;
//     };
//     typedef std::vector<SwitchData>  SwitchContainer;
//     typedef boost::unordered_map<std::string, SwitchContainer>  SwitchMap;
//     SwitchMap switch_map_;

public:
    RobotLayerWithFTS(ros::NodeHandle nh);

    virtual void handleInit(canopen::LayerStatus &status);
//     virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
};

#endif
