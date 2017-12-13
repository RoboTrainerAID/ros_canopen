#include <canopen_motor_node/robot_layer_with_fts.h>

RobotLayerWithFTS::RobotLayerWithFTS(ros::NodeHandle nh) : RobotLayer(nh)
{
    nh.param<std::string>("FTS/name", fts_name, "fts_sensor_name");
    nh.param<std::string>("Node/transform_frame", fts_transform_frame, "fts_frame");
    registerInterface(&fts_interface_);
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

    fts_interface_.registerHandle(handle);
}

// bool RobotLayerWithFTS::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) {
//     // compile-time check for mode switching support in ros_control
//     (void) &hardware_interface::RobotHW::prepareSwitch; // please upgrade to ros_control/contoller_manager 0.9.4 or newer
// 
//     // stop handles
//     for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin(); controller_it != stop_list.end(); ++controller_it){
// 
//         if(switch_map_.find(controller_it->name) == switch_map_.end()){
//             ROS_ERROR_STREAM(controller_it->name << " was not started before");
//             return false;
//         }
//     }
// 
//     // start handles
//     for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
//         SwitchContainer to_switch;
//         ros::NodeHandle nh(nh_, controller_it->name);
//         ModeLookup ml(nh);
// 
//         std::set<std::string> claimed_interfaces;
// 
//         if(controller_it->claimed_resources.size() > 0){
//             for (std::vector<hardware_interface::InterfaceResources>::const_iterator cres_it = controller_it->claimed_resources.begin(); cres_it != controller_it->claimed_resources.end(); ++cres_it){
//                 for (std::set<std::string>::const_iterator res_it = cres_it->resources.begin(); res_it != cres_it->resources.end(); ++res_it){
//                     claimed_interfaces.insert(cres_it->hardware_interface);
//                     if(!ml.hasModes()){
//                         ROS_ERROR_STREAM("Please set required_drive_mode(s) for controller " << controller_it->name);
//                         return false;
//                     }
//                     if(claimed_interfaces.size() > 1 && !ml.hasMixedModes()){
//                         ROS_ERROR_STREAM("controller "<< controller_it->name << " has mixed interfaces, please set required_drive_modes.");
//                         return false;
//                     }
// 
//                     boost::unordered_map< std::string, boost::shared_ptr<HandleLayerBase> >::const_iterator h_it = handles_.find(*res_it);
// 
//                     const std::string & joint = *res_it;
// 
//                     if(h_it == handles_.end()){
//                         ROS_ERROR_STREAM(joint << " not found");
//                         return false;
//                     }
//                     SwitchData sd;
//                     sd.enforce_limits = nh.param("enforce_limits", true);
// 
//                     if(!ml.getMode(sd.mode, joint)){
//                         ROS_ERROR_STREAM("could not determine drive mode for " << joint);
//                         return false;
//                     }
// 
// //                     if(g_interface_mapping.hasConflict(cres_it->hardware_interface, sd.mode)){
// //                         ROS_ERROR_STREAM(cres_it->hardware_interface << " cannot be provided in mode " << sd.mode);
// //                         return false;
// //                     }
// 
//                     HandleLayerBase::CanSwitchResult res = h_it->second->canSwitch(sd.mode);
// 
//                     switch(res){
//                         case HandleLayerBase::NotSupported:
//                             ROS_ERROR_STREAM("Mode " << sd.mode << " is not available for " << joint);
//                             return false;
//                         case HandleLayerBase::NotReadyToSwitch:
//                             ROS_ERROR_STREAM(joint << " is not ready to switch mode");
//                             return false;
//                         case HandleLayerBase::ReadyToSwitch:
//                         case HandleLayerBase::NoNeedToSwitch:
//                             sd.handle = h_it->second;
//                             to_switch.push_back(sd);
//                     }
//                 }
//             }
//         }
//         switch_map_.insert(std::make_pair(controller_it->name, to_switch));
//     }
// 
//     // perform mode switches
//     boost::unordered_set<boost::shared_ptr<HandleLayerBase> > to_stop;
//     std::vector<std::string> failed_controllers;
//     for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin(); controller_it != stop_list.end(); ++controller_it){
//         SwitchContainer &to_switch = switch_map_.at(controller_it->name);
//         for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
//             to_stop.insert(it->handle);
//         }
//     }
//     for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
//         SwitchContainer &to_switch = switch_map_.at(controller_it->name);
//         bool okay = true;
//         for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
//             it->handle->switchMode(MotorBase::No_Mode); // stop all
//         }
//         for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
//             if(!it->handle->switchMode(it->mode)){
//                 failed_controllers.push_back(controller_it->name);
//                 ROS_ERROR_STREAM("Could not switch one joint for " << controller_it->name << ", will stop all related joints and the controller.");
//                 for(RobotLayer::SwitchContainer::iterator stop_it = to_switch.begin(); stop_it != to_switch.end(); ++stop_it){
//                     to_stop.insert(stop_it->handle);
//                 }
//                 okay = false;
//                 break;
//             }else{
//                 it->handle->enableLimits(it->enforce_limits);
//             }
//             to_stop.erase(it->handle);
//         }
//     }
//     for(boost::unordered_set<boost::shared_ptr<HandleLayerBase> >::iterator it = to_stop.begin(); it != to_stop.end(); ++it){
//         (*it)->switchMode(MotorBase::No_Mode);
//     }
//     if(!failed_controllers.empty()){
//         stopControllers(failed_controllers);
//         // will not return false here since this would prevent other controllers to be started and therefore lead to an inconsistent state
//     }
// 
//     return true;
// }
