#ifndef CANOPEN_401_BASE_H
#define CANOPEN_401_BASE_H

#include <canopen_master/canopen.h>

namespace canopen {

/**
 * Did we need this ? May we could use IO401 and call it IO401_base
 */
class IoBase: public canopen::Layer {
protected:
	IoBase(const std::string &name) :
			Layer(name) {
	}
public:

	// unused
	enum OperationMode {
		Test = 0, Productive = 1,
	};

	// unused
	virtual void registerDefaultModes(
			boost::shared_ptr<ObjectStorage> storage) {
	}

	/**
	 *
	 */
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
