#include <canopen_led_node/canopen_led_layer.h>

/* motor_node/robot_layer.cpp
  
   -ledlayer object from config representing ea led-able node
   -request dimensions of the ledlayer(amount: channels,banks,groups)

*/
using namespace canopen;


LedHandle::LedHandle(
        const std::string &name, const boost::shared_ptr<IoBase> & base, const boost::shared_ptr<ObjectStorage> storage,
        XmlRpc::XmlRpcValue & options)
: Layer(name + " Handle"), base_(base), variables_(storage) {

    //number of leds (multiplied by 3 for RGB)
    if(options.hasMember("leds")) leds_ = (const int&) options["leds"];
    if(options.hasMember("banks")) banks_ = (const int&) options["banks"];
    if(options.hasMember("bank_size")) bank_size_ = (const int&) options["bank_size"];
    if(options.hasMember("groups")) groups_ = (const int&) options["groups"];
    
    //TODO setup storage entries

}

void LedHandle::handleRead(LayerStatus &status, const LayerState &current_state){
    
}
void LedHandle::handleWrite(LayerStatus &status, const LayerState &current_state){  
 
}
void LedHandle::handleInit(LayerStatus &status){
    
}
/*
void LedHandle::handleDiag(LayerReport &report){
    
}
void LedHandle::handleShutdown(LayerStatus &status){
    
}
void LedHandle::handleHalt(LayerStatus &status){
    
}
void LedHandle::handleRecover(LayerStatus &status){
   
}*/


void LedLayer::add(const std::string &name, boost::shared_ptr<LedHandle> handle){
    LayerGroupNoDiag::add(handle);
    handles_.insert(std::make_pair(name, handle));
}


LedLayer::LedLayer(ros::NodeHandle nh) : LayerGroupNoDiag<LedHandle>("LedLayer"), nh_(nh)
{
  //LedLayer is created before the ObjectStorage is available,
  //TODO setup callbacks
  
}
