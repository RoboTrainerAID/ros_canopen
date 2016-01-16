#ifndef CANOPEN_401_IO_H
#define CANOPEN_401_IO_H

#include <canopen_401/base.h>
#include <canopen_master/canopen.h>
#include <boost/function.hpp>
#include <boost/container/flat_map.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <limits>
#include <algorithm>

namespace canopen
{

class IO401 : public IoBase
{
public:

    IO401(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings)
    : IoBase(name)
      
    {
      /*
        storage->entry(status_word_entry_, 0x6041);
        storage->entry(control_word_entry_, 0x6040);
        storage->entry(op_mode_display_, 0x6061);
        storage->entry(op_mode_, 0x6060);
        */
    }


    class Allocator : public IoBase::Allocator{
    public:
        virtual boost::shared_ptr<IoBase> allocate(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings);
    };
    
protected:
 
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleDiag(LayerReport &report);
    virtual void handleInit(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);
 /*
private:

    canopen::ObjectStorage::Entry<uint16_t>  status_word_entry_;
    canopen::ObjectStorage::Entry<uint16_t >  control_word_entry_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_display_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_;
    canopen::ObjectStorage::Entry<uint32_t>  supported_drive_modes_;
    */
};

}

#endif