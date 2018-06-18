#ifndef CANOPEN_LED_NODE_H_
#define CANOPEN_LED_NODE_H_

#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/chrono/system_clocks.hpp>

#include <canopen_master/canopen.h>
#include <canopen_401/base.h>
//#include <canopen_led_node/Led.h>
//#include <canopen_led_node/BankMapping.h>
//#include <canopen_led_node/GlobalMapping.h>



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
typedef boost::chrono::high_resolution_clock::time_point time_point;
  
/**
 * Status class for led channels.
 * Compare internal state with given new state to get differences for an update
 */
class LedState {

public:
	LedState(int leds, int banks, int bank_size, int groups) {
		this->leds_ = leds;
		this->banks_ = banks;
		this->bank_size_ = bank_size;
		this->groups_ = groups;

		// set all to zero
		createInitState();
	}

	std::vector<int> getLEDchannels() {
		return this->led_channels;
	}

	int getLEDChannelCount() {
		return this->leds_;
	}
	int getBankCount() {
		return this->banks_;
	}
	int getBankSize() {
		return this->bank_size_;
	}
	int getGroupCount() {
		return this->groups_;
	}
	std::vector<int> getLastState() {
		return this->led_channels_last_version;
	}
	void returnToLastState() {
		this->led_channels = this->led_channels_last_version;	
	}

	/*
	 * Compare all led channels
	 * compare the states and return a map with led channel - 1 and brightness value
	 */
	std::map<int, int> compare(std::vector<int> led_channel_set) {
		std::map<int, int> led_channel_to_change;

		// check the size
		if (led_channel_set.size() != this->led_channels.size())
			return led_channel_to_change;

		std::vector<int>::iterator leftIt = this->led_channels.begin();
		std::vector<int>::iterator rightIt = led_channel_set.begin();

		int c_index = 0;
		while (leftIt != led_channels.end() && rightIt != led_channel_set.end()) {
			if (*leftIt != *rightIt) {
				led_channel_to_change.insert(
						std::pair<int, int>(c_index + 1, *rightIt));
			}

			leftIt++;
			rightIt++;
			c_index++;
		}

		return led_channel_to_change;
	}

	/*
	 * Compare a bank change
	 * compare the states and return a map with led channel - 1 and brightness value (should change
	 */
	std::map<int, int> compare(int bank, std::vector<int> bank_set) {
		std::map<int, int> led_channel_to_change;

		if (bank <= 0 || bank > this->banks_)
			return led_channel_to_change;
		std::vector<int> bank_channels = this->getBank(bank);
		bank -= 1;

		// check the size
		if (bank_channels.size() != bank_set.size())
			return led_channel_to_change;

		std::vector<int>::iterator leftIt = bank_channels.begin();
		std::vector<int>::iterator rightIt = bank_set.begin();

		int c_index = 0;
		while (leftIt != bank_channels.end() && rightIt != bank_set.end()) {
			if (*leftIt != *rightIt) {
				led_channel_to_change.insert(
						std::pair<int, int>(
								(bank * this->bank_size_) + c_index + 1,
								*rightIt));
			}

			leftIt++;
			rightIt++;
			c_index++;
		}

		return led_channel_to_change;
	}
	
	/*
	 * Compare one led channel
	 * compare the state and return a map with led channel - 1 and brightness value
	 */
	std::map<int, int> compareAndUpdate(int led_channel, int value) {
		this->led_channels_last_version = this->led_channels;
		std::map<int, int> led_channel_to_change;
		
		// check the size
		if (led_channel < 0 || led_channel >= this->led_channels.size()) {
		  return led_channel_to_change;
		} else if(led_channels[led_channel] != value) {
		  // update
		  led_channels[led_channel] = value;
		  led_channel_to_change.insert(std::pair<int, int>(led_channel, value));
		}
		//std::cout << "return: " << led_channel_to_change.size() << std::endl;
		return led_channel_to_change;
	}
	
	/*
	 * Compare all led channels
	 * compare the states and return a map with led channel - 1 and brightness value
	 */
	std::map<int, int> compareAndUpdate(std::vector<int> led_channel_set) {
		this->led_channels_last_version = this->led_channels;
		std::map<int, int> led_channel_to_change;

		// check the size
		if (led_channel_set.size() != this->led_channels.size())
			return led_channel_to_change;

		std::vector<int>::iterator leftIt = led_channels.begin();
		std::vector<int>::iterator rightIt = led_channel_set.begin();

		int c_index = 0;
		while (leftIt != led_channels.end() && rightIt != led_channel_set.end()) {
			if (*leftIt != *rightIt) {
				// update
				*leftIt = *rightIt;
				led_channel_to_change.insert(
						std::pair<int, int>(c_index, *rightIt));
			}

			leftIt++;
			rightIt++;
			c_index++;
		}

		return led_channel_to_change;
	}
	/*
	 * Compare a bank change
	 * compare the states and return a map with led channel - 1 and brightness value (should change
	 * 
	 * bank 0 ... n
	 */
	std::map<int, int> compareAndUpdate(int bank, std::vector<int> bank_set) {
		// save last stand
		this->led_channels_last_version = this->led_channels;
		std::map<int, int> led_channel_to_change;

		if (bank < 0 || bank >= this->banks_)
			return led_channel_to_change;
		std::vector<int> bank_channels = this->getBank(bank);
		
		// check the size
		if (bank_channels.size() != bank_set.size())
			return led_channel_to_change;

		std::vector<int>::iterator leftIt = bank_channels.begin();
		std::vector<int>::iterator rightIt = bank_set.begin();

		int c_index = 0;
		while (leftIt != bank_channels.end() && rightIt != bank_set.end()) {
			if (*leftIt != *rightIt) {
				// update
				this->led_channels.at((bank * this->bank_size_) + c_index) =
						*rightIt;
				led_channel_to_change.insert(
						std::pair<int, int>(
								(bank * this->bank_size_) + c_index,
								*rightIt));
			}

			leftIt++;
			rightIt++;
			c_index++;
		}

		return led_channel_to_change;
	}

private:
	int leds_, banks_, bank_size_, groups_;


	// all led channels
	std::vector<int> led_channels;
	// all led channels
	std::vector<int> led_channels_last_version;
	// bank, ptr to led_channels;
	std::map<int, std::vector<int*> > bank_list;
	// group, ptr to the bank not implemented
	std::map<int, std::map<int, std::vector<int*> >*> group_list;

	void createInitState() {
		// first init the led channel vector
		for (int i = 0; i < leds_; i++)
			led_channels.push_back(0); // zero-initialized

		// create bank map with ptrs to the real led channels
		for (int i = 0; i < this->banks_; i++) {
			std::vector<int*> bank_vec;
			for (int k = 0; k < this->bank_size_; k++) {
				// get the real led channel
				int led_channel_obj = (bank_size_ * i) + k;
				if (led_channel_obj < leds_) {
					std::vector<int>::iterator it = led_channels.begin()
							+ (led_channel_obj);

					// set the pointer to the real led_channel
					bank_vec.push_back(&(*it));
				}
			}
			// insert
			bank_list.insert(std::pair<int, std::vector<int*> >(i, bank_vec));
		}
	}

	std::vector<int> getBank(int bank) {
		std::vector<int> bank_channels;
		if (bank < 0 || bank >= this->banks_)
			return bank_channels;
		

		for (std::map<int, std::vector<int*> >::iterator bankit = this->bank_list.begin();
				bankit != this->bank_list.end(); bankit++) {
			if (bankit->first == (bank)) {
				std::vector<int*> vvec = (*bankit).second;
				std::vector<int*>::iterator vit;

				for (std::vector<int*>::iterator vit = bankit->second.begin();
						vit != bankit->second.end(); vit++) {
					int *ptr = (*vit);
					bank_channels.push_back(*ptr);
				}
			}
		}
		return bank_channels;
	}
};  
  
/**
 * class: LedLayer
 *
 * setup storage entries for ManufacturerObjects of LED profile
 *
 *
 * Define Topics:
 * - self test: The used Hardware should test the functionality
 * - banks and groups: used to control banks and groups
 * - global led enable: enable all leds ( if it isn't enabled you can't set new states)
 * - global brightness: set all led channels to the same brightness
 */
class LedLayer: public canopen::Layer {

	ros::NodeHandle nh_;
	boost::shared_ptr<canopen::IoBase> base_;
	const boost::shared_ptr<ObjectStorage> storage_;
	canopen::LedState *ledState_;

	uint16_t leds_, banks_, bank_size_, groups_;
	std::string conf_id_;
	canopen::ObjectStorage::Entry<int8_t> selfTest_;
	canopen::ObjectStorage::Entry<uint8_t> globalLedArrayEnable_,
			bankBrightness_, groupBrightness_, nodeID_;
	canopen::ObjectStorage::Entry<uint16_t> globalBrightness_,
			channelMultiplexer_, bitrate_;
	canopen::ObjectStorage::Entry<int16_t> outputValue_;

protected:
	void setLed(const canopen_led_node::Led::ConstPtr& msg);
	void setGlobalMapping(const canopen_led_node::GlobalMapping::ConstPtr& msg);
	void setBankMapping(const canopen_led_node::BankMapping::ConstPtr& msg);
	void setGlobalBrightness(const std_msgs::UInt16::ConstPtr& msg);
	void globalLedArrayEnable(const std_msgs::Bool::ConstPtr& msg);
	void selfTest(const std_msgs::Bool::ConstPtr& msg);

private:
	boost::mutex mutex_;
	ros::Subscriber selfTest_sub_, set_led_sub_,
			globalBrightness_sub_, globalLedArrayEnable_sub_, bankMapping_sub_,
			globalMapping_sub_;

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
	
	//changes to leds that yet have to be pushed by handleWrite
	std::map<int, int> ledUpdates_;
	
	time_point last_update_;
	boost::chrono::milliseconds step_;

	
	//insert map to_change into ledUpdates with value update
	void insertChanges(std::map<int, int> to_change) {
	  boost::mutex::scoped_lock lock(mutex_);
	  for (std::map<int, int>::iterator it=to_change.begin(); it!=to_change.end(); ++it){
	    ledUpdates_[it->first] = it->second;
	  } 
	}
	
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

}// end canoopen namespace

#endif
