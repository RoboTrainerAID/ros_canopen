
#ifndef CANOPEN_LED_NODE_H_
#define CANOPEN_LED_NODE_H_


#include <canopen_master/canopen.h>

namespace canopen
{

class LedBase : public canopen::Layer {
protected:
    LedBase(const std::string &name) : Layer(name) {}
public:
    

    class Allocator {
    public:
        virtual boost::shared_ptr<LedBase> allocate(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings) = 0;
        virtual ~Allocator() {}
    };
};

}

#endif
