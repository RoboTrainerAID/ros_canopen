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

namespace canopen {
/**
 * class: LedLayer
 *
 * setup storage entries:
 * - ManufacturerObjects LED profile 401 (not the whole profile)
 *
 * Define send functions
 *
 * Define some Topics:
 * - self test: The used Hardware should test the functionality
 * - banks and groups: used to control banks and groups
 * - global led enable: enable all leds ( if it isn't enabled you can't set new states)
 * - global brightness: set all led channels to the same brightness
 */
class LedLayer: public canopen::Layer {

	ros::NodeHandle nh_;
	boost::shared_ptr<canopen::IoBase> base_;
	const boost::shared_ptr<ObjectStorage> storage_;

	uint16_t leds_, banks_, bank_size_, groups_;
	canopen::ObjectStorage::Entry<int8_t> selfTest_;
	canopen::ObjectStorage::Entry<uint8_t> globalLedArrayEnable_,
			bankBrightness_, groupBrightness_, nodeID_;
	canopen::ObjectStorage::Entry<uint16_t> globalBrightness_,
			channelMultiplexer_, bitrate_;
	canopen::ObjectStorage::Entry<int16_t> outputValue_;

protected:
	void writemultiplexedOut16(const canopen_led_node::Led::ConstPtr& msg);
	void setLed(const canopen_led_node::Led::ConstPtr& msg);
	void setGlobalBrightness(const std_msgs::UInt16::ConstPtr& msg);
	void globalLedArrayEnable(const std_msgs::Bool::ConstPtr& msg);
	void selfTest(const std_msgs::Bool::ConstPtr& msg);

private:
	ros::Subscriber selfTest_sub_, set_led_sub_, writemultiplexedOut16_sub_,
			globalBrightness_sub_, globalLedArrayEnable_sub_;

	//Key is the bank/group number.
	//bank_map: for each bank, contains the subentry that specifies the number of channels (0x2101sub0 - 0x21FFsub0)
	//group_map: for each group, contains the subentry that specifies the number of channels (0x2201sub0 - 0x2210sub0)
	std::map<int, canopen::ObjectStorage::Entry<uint8_t> > bank_map, group_map;
	//bankBrightness_map: for each bank, contains the subentry of 0x2100 for that bank.
	//groupBrightness_map: for each group, contains the subentry of 0x2200 for that group.
	std::map<int, canopen::ObjectStorage::Entry<int16_t> > bankBrightness_map,
			groupBrightness_map;

	//for each bank, contains a vector of all the channel-subentries [0x2101 - 0x21FF, 0x01 - 0xFE]
	std::map<int, std::vector<canopen::ObjectStorage::Entry<int16_t> > > led_map;
	//for each group, contains a vector of all the channel-subentries [0x2201 - 0x2210, 0x01 - 0xFE]
	std::map<int, std::vector<canopen::ObjectStorage::Entry<uint16_t> > > channel_map;

	virtual void handleRead(canopen::LayerStatus &status,
			const LayerState &current_state);
	virtual void handleWrite(canopen::LayerStatus &status,
			const LayerState &current_state);
	virtual void handleInit(canopen::LayerStatus &status);
	virtual void handleDiag(canopen::LayerReport &report) { }
	virtual void handleShutdown(canopen::LayerStatus &status) {	}
	virtual void handleHalt(canopen::LayerStatus &status) {	}
	virtual void handleRecover(canopen::LayerStatus &status) {
		handleRead(status, Layer::Ready);
	}

public:
	// dublicated
	template<typename T> bool set(T & entry, const typename T::type &value) {
		try {
			entry.set(value);
		} catch (...) {
			return false;
		}
		return true;
	}
	LedLayer(ros::NodeHandle nh, const std::string &name,
			const boost::shared_ptr<IoBase> & base,
			const boost::shared_ptr<canopen::ObjectStorage> storage,
			XmlRpc::XmlRpcValue & options);

};
}

#endif
