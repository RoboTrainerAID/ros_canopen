#include <canopen_led_node/canopen_led_layer.h>

/* motor_node/robot_layer.cpp
  
   -ledlayer object from config representing ea led-able node
   -request dimensions of the ledlayer(amount: channels,banks,groups)

*/
using namespace canopen;

  void LedLayer::write(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("msg: %d", msg->data);
    uint8_t value = msg->data;
    set(writeDigitalOut8_, value);   
  }
  
  
LedLayer::LedLayer(ros::NodeHandle nh,
        const std::string &name, const boost::shared_ptr<IoBase> & base, const boost::shared_ptr<ObjectStorage> storage,
        XmlRpc::XmlRpcValue & options)
: Layer(name + " Handle"), base_(base), variables_(storage), nh_(nh) {

    //number of leds (multiplied by 3 for RGB)
    if(options.hasMember("leds")) leds_ = (const int&) options["leds"];
    if(options.hasMember("banks")) banks_ = (const int&) options["banks"];
    if(options.hasMember("bank_size")) bank_size_ = (const int&) options["bank_size"];
    if(options.hasMember("groups")) groups_ = (const int&) options["groups"];
    
    //TODO setup storage entries

    // Write Outputs 8 Bit
    //storage->entry(supported_drive_modes_, 0x6200, 0x00); // ro: Number of Output 8 Bit: default=1
    storage->entry(writeDigitalOut8_, 0x6200, 1); // rw: Write Outputs 0x1 to 0x8
    
    //TODO setup callbacks
    write_ = nh.subscribe("writeDigitalOut8", 1, &LedLayer::write, this);
}


void LedLayer::handleRead(LayerStatus &status, const LayerState &current_state){
    
}
void LedLayer::handleWrite(LayerStatus &status, const LayerState &current_state){  
 
}
void LedLayer::handleInit(LayerStatus &status){
    
}