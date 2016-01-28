#include <canopen_401/IO401.h>

namespace canopen
{
  
  void IO401::cb(const std_msgs::UInt16::ConstPtr& msg)    {
    ROS_INFO("msg: %d", msg->data);
     uint16_t value_ = msg->data;
     set(global_brightness_, value_);   
  }

  void IO401::write(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("msg: %d", msg->data);
    uint8_t value_ = msg->data;
    set(writeDigitalOut8_, value_);   
  }

void IO401::handleRead(LayerStatus &status, const LayerState &current_state){
    
}
void IO401::handleWrite(LayerStatus &status, const LayerState &current_state){  
 
}
void IO401::handleDiag(LayerReport &report){
    
}
void IO401::handleInit(LayerStatus &status){
    
}
void IO401::handleShutdown(LayerStatus &status){
    
}
void IO401::handleHalt(LayerStatus &status){
    
}
void IO401::handleRecover(LayerStatus &status){
   
}

} // namespace