#include <canopen_401/IO401.h>

namespace canopen
{
  
bool IO401::setTarget(double val){
        boost::mutex::scoped_lock lock(mode_mutex_);
        return selected_mode_ && selected_mode_->setTarget(val);
    return false;
}  
  
bool IO401::isModeSupportedByDevice(uint16_t mode){
    return mode > 0 && supported_drive_modes_.valid() && (supported_drive_modes_.get_cached() & (1<<(mode-1)));
}

void IO401::registerMode(uint16_t id, const boost::shared_ptr<Mode> &m){
    boost::mutex::scoped_lock map_lock(map_mutex_);
    if(m && m->mode_id_ == id) modes_.insert(std::make_pair(id, m));
}

boost::shared_ptr<Mode> IO401::allocMode(uint16_t mode){
    boost::shared_ptr<Mode> res;
    if(isModeSupportedByDevice(mode)){
        boost::mutex::scoped_lock map_lock(map_mutex_);
        boost::unordered_map<uint16_t, boost::shared_ptr<Mode> >::iterator it = modes_.find(mode);
        if(it != modes_.end()){
            res = it->second;
        }
    }
    return res;
}

void IO401::handleRead(LayerStatus &status, const LayerState &current_state){
    
}
void IO401::handleWrite(LayerStatus &status, const LayerState &current_state){
  //canopen::ObjectStorage::Entry<TYPE> target_entry_;
  //target_entry_.set(this->getTarget());
  
       control_word_ = 6;
       Mode::OpModeAccesser cwa(control_word_);
       selected_mode_ = allocMode(IoBase::Profiled_Velocity);  
       selected_mode_->setTarget(1.0);
       selected_mode_->write(cwa);

    
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