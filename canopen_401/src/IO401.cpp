#include <canopen_401/IO401.h>

namespace canopen
{  

void IO401::writeAnalogOut16(const std_msgs::Int16::ConstPtr& msg) {
    //not tested
    int16_t value_ = msg->data;
    set(writeAnalogOut16_, value_);   
}
  
void IO401::writeDigitalOut8(const std_msgs::UInt8::ConstPtr& msg) {
    //not tested  
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
  if (use_401_) {
    for (int i = 0; i < 4; i++) {
      storage_->entry(rpdo_[i] , 0x1600 + i, 0);
      // Receive PDO i Mapping
      int pdo_entries = rpdo_[i].get();
      rpdo_map[i].resize(pdo_entries + 1);
      for (int j = 1; j <= pdo_entries; j++) {
	storage_->entry(rpdo_map[i][j] , 0x1600 + i, j); 
	// rw: PDO i Mapping for an application object j
      }
    }
    for (int i = 0; i < 5; i++) {
      storage_->entry(tpdo_[i], 0x1A00 + i, 0);
      // Transmit PDO i Mapping
      int pdo_entries = tpdo_[i].get();
      tpdo_map[i].resize(pdo_entries + 1);
      for (int j = 1; j <= pdo_entries; j++) {
	storage_->entry(tpdo_map[i][j], 0x1A00 + i, j);
	// rw: PDO i Mapping for a process data variable j
      }
    }
  }
}
void IO401::handleShutdown(LayerStatus &status){
    
}
void IO401::handleHalt(LayerStatus &status){
    
}
void IO401::handleRecover(LayerStatus &status){
   
}

} // namespace