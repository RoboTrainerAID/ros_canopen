
#ifndef CANOPEN_LED_NODE_H_
#define CANOPEN_LED_NODE_H_

#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <canopen_master/canopen.h>
#include <canopen_401/base.h>
#include <canopen_led_node/Led.h>


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>

namespace canopen
{
  


class LedLayer : public canopen::Layer{
  
     

    ros::NodeHandle nh_;
    boost::shared_ptr<canopen::IoBase> base_;
    const boost::shared_ptr<ObjectStorage> storage_;
    
    uint16_t leds_, banks_, bank_size_, groups_ ;
    canopen::ObjectStorage::Entry<uint8_t> globalLedArrayEnable_, writeDigitalOut8_, bankBrightness_,groupBrightness_;
    canopen::ObjectStorage::Entry<uint16_t> globalBrightness_;
    
protected: 
  void writeDigitalOut8(const std_msgs::UInt8::ConstPtr& msg);
  void setLed(const canopen_led_node::Led::ConstPtr& msg);
  void setGlobalBrightness(const std_msgs::UInt16::ConstPtr& msg); 
  void globalLedArrayEnable(const std_msgs::Bool::ConstPtr& msg); 
  //void setGroupBrightness(const std_msgs::Int16::ConstPtr& msg); 

    
    //TODO define storage entries
private:    
  ros::Subscriber writeDigitalOut8_sub_, set_led_sub_, globalBrightness_sub_, globalLedArrayEnable_sub_; 
  
  std::map<int, canopen::ObjectStorage::Entry<uint8_t> > bank_map, group_map;
  std::map<int, canopen::ObjectStorage::Entry<int16_t> > bankBrightness_map, groupBrightness_map;
  
  std::map<int, std::vector< canopen::ObjectStorage::Entry<int16_t> > > led_map;
  std::map<int, std::vector< canopen::ObjectStorage::Entry<uint16_t> > > channel_map;
  
  virtual void handleRead(canopen::LayerStatus &status, const LayerState &current_state);
  virtual void handleWrite(canopen::LayerStatus &status, const LayerState &current_state);
  virtual void handleInit(canopen::LayerStatus &status);
  virtual void handleDiag(canopen::LayerReport &report) { }
  virtual void handleShutdown(canopen::LayerStatus &status) {  }
  virtual void handleHalt(canopen::LayerStatus &status) { }
  virtual void handleRecover(canopen::LayerStatus &status) { handleRead(status, Layer::Ready); }
  
      
public:
    template<typename T> bool set(T & entry, const typename T::type &value) {
		try {
			entry.set(value);
		} catch (...) {
			return false;
		}
		return true;
	}
    LedLayer(ros::NodeHandle nh,const std::string &name, const boost::shared_ptr<IoBase> & base, const boost::shared_ptr<canopen::ObjectStorage> storage,  XmlRpc::XmlRpcValue & options);
   

  
    
};
}


#endif
