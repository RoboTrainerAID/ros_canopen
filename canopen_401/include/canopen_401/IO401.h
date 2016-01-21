#ifndef CANOPEN_401_IO_H
#define CANOPEN_401_IO_H

#include <canopen_401/base.h>
#include <canopen_master/canopen.h>
/*
#include <boost/function.hpp>
#include <boost/container/flat_map.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <limits>
#include <algorithm>
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
namespace canopen
{

class IO401 : public IoBase
{
  protected:
    void cb(const std_msgs::UInt8::ConstPtr& msg);
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleDiag(LayerReport &report);
    virtual void handleInit(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);
    
public:
    template<typename T> bool set(T & entry, const typename T::type &value){
        try{
            entry.set(value);
        }
        catch(...){
            return false;
        }
        return true;
    }
    
    void write(const std_msgs::Int16::ConstPtr& msg)    {
       ROS_INFO("msg: %d", msg->data);
    }
   
    
    IO401(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings)
    : IoBase(name)
      
    {
      /*
        storage->entry(status_word_entry_, 0x6041);
        storage->entry(control_word_entry_, 0x6040);
        storage->entry(op_mode_display_, 0x6061);
        storage->entry(op_mode_, 0x6060);
        */
      ros::NodeHandle n;
      sub_ = n.subscribe("Mode", 1, &IO401::cb, this);
     // write_ = n.subscribe("Mode", 1, &IO401::write, this);
      
      storage->entry(supported_drive_modes_, 0x6200, 0x01);
     
      
      
      
      
      
      
      
    }
        
    class Allocator : public IoBase::Allocator{
    public:
        virtual boost::shared_ptr<IoBase> allocate(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings);
	
      
    };
   
private:
  
    ros::Subscriber sub_,write_;
    uint8_t value_;
    canopen::ObjectStorage::Entry<uint8_t>  supported_drive_modes_;
    
 /*
    canopen::ObjectStorage::Entry<uint16_t>  status_word_entry_;
    canopen::ObjectStorage::Entry<uint16_t >  control_word_entry_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_display_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_;
    
    */
};

}

#endif