#ifndef CANOPEN_401_BASE_H
#define CANOPEN_401_BASE_H

#include <canopen_master/canopen.h>

namespace canopen
{

class IoBase : public canopen::Layer {
protected:
    IoBase(const std::string &name) : Layer(name) {}
public:
    enum OperationMode
    {
        No_Mode = 0,
        Profiled_Position = 1,
        Velocity = 2,
        Profiled_Velocity = 3,
        Profiled_Torque = 4,
        Reserved = 5,
        Homing = 6,
        Interpolated_Position = 7,
        Cyclic_Synchronous_Position = 8,
        Cyclic_Synchronous_Velocity = 9,
        Cyclic_Synchronous_Torque = 10,
    };
    
    virtual void registerDefaultModes(boost::shared_ptr<ObjectStorage> storage) {}
    
    class Allocator {
    public:
        virtual boost::shared_ptr<IoBase> allocate(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings) = 0;
        virtual ~Allocator() {}
    };
};

}

#endif
