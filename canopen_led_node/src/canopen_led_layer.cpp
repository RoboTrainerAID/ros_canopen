#include <canopen_led_node/canopen_led_layer.h>

/* motor_node/robot_layer.cpp
   402/motor.cpp|.h
   402/base.h

   -ledlayer object from config representing ea led-able node
   -states of leds/banks/groups of ea ledlayer obj
   -request dimensions of the ledlayer(amount: channels,banks,groups)

*/
using namespace canopen;


HandleLayer::HandleLayer(
        const std::string &name, const boost::shared_ptr<IoBase> & base, const boost::shared_ptr<ObjectStorage> storage,
        XmlRpc::XmlRpcValue & options)
: Layer(name + " Handle"), base_(base), variables_(storage) {

    //number of leds (multiplied by 3 for RGB)
    if(options.hasMember("leds")) leds_ = (const uint16_t) options["leds"];
    if(options.hasMember("banks")) banks_ = (const uint16_t) options["banks"];
    if(options.hasMember("bank_size")) bank_size_ = (const uint16_t) options["bank_size"];
    if(options.hasMember("groups")) groups_ = (const uint16_t) options["groups"];
    
    //TODO setup storage entries

}

void LedLayer::add(const std::string &name, boost::shared_ptr<HandleLayer> handle){
    LayerGroupNoDiag::add(handle);
    handles_.insert(std::make_pair(name, handle));
}


LedLayer::LedLayer(ros::NodeHandle nh) : LayerGroupNoDiag<HandleLayer>("LedLayer"), nh_(nh)
{
  //LedLayer is created before the ObjectStorage is available,
  //TODO setup callbacks
  
}
