
#ifndef CANOPEN_LED_NODE_H_
#define CANOPEN_LED_NODE_H_

#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <canopen_master/canopen.h>
#include <canopen_401/base.h>
#include <canopen_led_node/Led.h>


#include <ros/ros.h>
#include <ros/package.h>
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

class ObjectVariables {
    const boost::shared_ptr<canopen::ObjectStorage> storage_;
    struct Getter {
        boost::shared_ptr<double> val_ptr;
        boost::function<bool(double&)> func;
        bool operator ()() { return func(*val_ptr); }
        template<typename T> Getter(const canopen::ObjectStorage::Entry<T> &entry): func(boost::bind(&Getter::readObject<T>, entry, _1)), val_ptr(new double) { }
        template<typename T> static bool readObject(canopen::ObjectStorage::Entry<T> &entry, double &res){
            T val;
            if(!entry.get(val)) return false;
            res = val;
            return true;
        }
        operator double*() const { return val_ptr.get(); }
    };
    boost::unordered_map<canopen::ObjectDict::Key, Getter> getters_;
public:
    template<const uint16_t dt> static double* func(ObjectVariables &list, const canopen::ObjectDict::Key &key){
        typedef typename canopen::ObjectStorage::DataType<dt>::type type;
        return list.getters_.insert(std::make_pair(key, Getter(list.storage_->entry<type>(key)))).first->second;
    }
    ObjectVariables(const boost::shared_ptr<canopen::ObjectStorage> storage) : storage_(storage) {}
    bool sync(){
        bool ok = true;
        for(boost::unordered_map<canopen::ObjectDict::Key, Getter>::iterator it = getters_.begin(); it != getters_.end(); ++it){
            ok = it->second() && ok;
        }
        return ok;
    }
    double * getVariable(const std::string &n) {
        try{
            if(n.find("obj") == 0){
                canopen::ObjectDict::Key key(n.substr(3));
                boost::unordered_map<canopen::ObjectDict::Key, Getter>::const_iterator it = getters_.find(key);
                if(it != getters_.end()) return it->second;
                return canopen::branch_type<ObjectVariables, double * (ObjectVariables &list, const canopen::ObjectDict::Key &k)>(storage_->dict_->get(key)->data_type)(*this, key);
            }
        }
        catch( const std::exception &e){
            ROS_ERROR_STREAM("Could not find variable '" + n + "', reason: " + boost::diagnostic_information(e));
        }
        return 0;
    }
};

template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_VISIBLE_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_OCTET_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_UNICODE_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_DOMAIN >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }


namespace canopen
{
  


class LedLayer : public canopen::Layer{
  
     

    ros::NodeHandle nh_;
    boost::shared_ptr<canopen::IoBase> base_;
    ObjectVariables variables_;
    
    uint16_t leds_, banks_, bank_size_, groups_ ;
    canopen::ObjectStorage::Entry<uint8_t> writeDigitalOut8_, bank01_;
    canopen::ObjectStorage::Entry<int16_t> b01_ch01_, b01_ch02_,b01_ch03_;

protected: 
  void write(const std_msgs::UInt8::ConstPtr& msg);
  void setLed(const canopen_led_node::Led::ConstPtr& msg); 
  void setB1(const std_msgs::Int16MultiArray::ConstPtr& msg); 
    
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
   

    
    
    //TODO define storage entries
private:    
  ros::Subscriber write_, set_led_, set_B1_; 
  
  std::map<int, canopen::ObjectStorage::Entry<uint8_t> > group_map;
  std::map<int, canopen::ObjectStorage::Entry<uint8_t> > bank_map;
  std::map<int, std::vector< canopen::ObjectStorage::Entry<int16_t> > > channel_map;
  
  virtual void handleRead(canopen::LayerStatus &status, const LayerState &current_state);
  virtual void handleWrite(canopen::LayerStatus &status, const LayerState &current_state);
  virtual void handleInit(canopen::LayerStatus &status);
  virtual void handleDiag(canopen::LayerReport &report) { }
  virtual void handleShutdown(canopen::LayerStatus &status) {  }
  virtual void handleHalt(canopen::LayerStatus &status) { }
  virtual void handleRecover(canopen::LayerStatus &status) { handleRead(status, Layer::Ready); }
  
  
    
};
}


#endif
