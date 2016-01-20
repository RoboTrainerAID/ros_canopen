#include <canopen_401/IO401.h>

namespace canopen
{
  
  void IO401::cb(const std_msgs::UInt16::ConstPtr& msg)    {
     value_ = msg->data;
  }


void IO401::handleRead(LayerStatus &status, const LayerState &current_state){
    
}
void IO401::handleWrite(LayerStatus &status, const LayerState &current_state){  
  set(supported_drive_modes_, value_);   
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