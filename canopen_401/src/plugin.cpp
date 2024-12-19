#include <class_loader/class_loader.hpp>
#include <canopen_401/IO401.h>

std::shared_ptr<canopen::IoBase> canopen::IO401::Allocator::allocate(const std::string &name, std::shared_ptr<canopen::ObjectStorage> storage, const canopen::Settings &settings) {
    return std::make_shared<canopen::IO401>(name, storage, settings);
}

CLASS_LOADER_REGISTER_CLASS(canopen::IO401::Allocator, canopen::IoBase::Allocator);
