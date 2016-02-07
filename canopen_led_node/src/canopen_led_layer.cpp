#include <canopen_led_node/canopen_led_layer.h>

/* motor_node/robot_layer.cpp
 * 
 *  -ledlayer object from config representing ea led-able node
 *  -request dimensions of the ledlayer(amount: channels,banks,groups)
 * 
 */
using namespace canopen;

void LedLayer::write(const std_msgs::UInt8::ConstPtr& msg) {
  ROS_INFO("msg: %d", msg->data);
  uint8_t value = msg->data;
  set(writeDigitalOut8_, value);   
}

/*
 *    int16 group
 *    int16 bank
 *    int16 led
 *    uint16[] data 
 */
void LedLayer::setLed(const canopen_led_node::Led::ConstPtr& msg) {
  int group = msg->group ;
  int bank = msg->bank;
  int led = msg->led;
  int data_length = msg->data.size(); 
  if(group != 0) {
    
  } else if(bank != 0) {
    //each channel own value OR one rgb value for bank
    //bank_map[bank].get() == data_length
    if(bank_size_ == data_length) {
      for (int i = 0; i < data_length; i++) {
	channel_map[bank][i].set(msg->data[i]);
      }
    } else if (data_length == 3) {
      for (int i = 0; i < data_length; i++) {
	channel_map[bank][i].set(msg->data[i%3]);
      }
    }
  }
  
  
}

void LedLayer::setB1(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
  int dim = msg->layout.dim.size(); 
  if(dim == 1) { 
    set(b01_ch01_, msg->data[0]);   
    set(b01_ch02_, msg->data[1]); 
    set(b01_ch03_, msg->data[2]); 
  }
  
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
  
  //setup banks and channels
  
  for (int i = 1; i <= banks_; i++) {
    //TODO test if working i > 1
    storage->entry(bank_map[i], (0x2100 + i), 0);
    channel_map[i].resize(bank_size_+ 1);
    for (int j = 1; j <= bank_size_; j++) {
      storage->entry(channel_map[i][j],(0x2100 + i),j);
    }
  }
  
  storage->entry(bank01_, 0x2101, 0);
  storage->entry(b01_ch01_, 0x2101, 1);
  storage->entry(b01_ch02_, 0x2101, 2);
  storage->entry(b01_ch03_, 0x2101, 3);
  
  //TODO setup callbacks
  write_ = nh.subscribe("writeDigitalOut8", 1, &LedLayer::write, this);
  set_led_ =  nh.subscribe("set_led", 1, &LedLayer::setLed, this);
  
  set_B1_ =  nh.subscribe("set_B1", 1, &LedLayer::setB1, this);
}


void LedLayer::handleRead(LayerStatus &status, const LayerState &current_state){
  
}
void LedLayer::handleWrite(LayerStatus &status, const LayerState &current_state){  
  
}
void LedLayer::handleInit(LayerStatus &status){
  
}