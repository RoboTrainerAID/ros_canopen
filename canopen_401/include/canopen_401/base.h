#ifndef CANOPEN_401_BASE_H
#define CANOPEN_401_BASE_H

#include <canopen_master/canopen.h>

namespace canopen {

/**
 * Base abstraction for ClassAllocator
 */
class IoBase: public canopen::Layer {
protected:
	IoBase(const std::string &name) :
			Layer(name) {
	}
public:

	class Allocator {
	public:
		virtual boost::shared_ptr<IoBase> allocate(const std::string &name,
				boost::shared_ptr<ObjectStorage> storage,
				const canopen::Settings &settings) = 0;
		virtual ~Allocator() { }
	};
};

}

#endif
