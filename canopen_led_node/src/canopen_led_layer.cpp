#include <canopen_led_node/canopen_led_layer.h>

using namespace canopen;

/*
 *    int16 group
 *    int16 bank
 *    int16 led
 *    uint16[] data 
 */
void LedLayer::setLed(const canopen_led_node::Led::ConstPtr& msg) {
	int group = msg->group;
	int bank = msg->bank;
	int led = msg->led;
	int data_length = msg->data.size();
	if (group != 0) {
		int group_size = group_map[group].get();
		if (data_length == 1) {
			if (led != 0) {
				//set one channel
				channel_map[group][led].set(msg->data[0]);
			} else {
				//groupBrightness
				groupBrightness_map[group].set(msg->data[0]);
			}
		} else if (data_length == group_size) {
			//direct mapping
			for (int i = 1; i <= group_size; i++) {
				channel_map[group][i].set(msg->data[i - 1]);
			}
		}
	} else if (bank != 0) {
		if (data_length == bank_size_) {
			//direct mapping
			for (int i = 1; i <= bank_size_; i++) {
				led_map[bank][i].set(msg->data[i - 1]);
			}
		} else if (data_length == 1) {
			if (led != 0) {
				//set one led channel
				led_map[bank][led].set(msg->data[0]);
			} else {
				//bankBrightness
				bankBrightness_map[bank].set(msg->data[0]);
			}
		} else if (data_length == 3) {
			if (led != 0) {
				//set all 3 channels of one led
				for (int i = led; i < led + 3; i++) {
					led_map[bank][i].set(msg->data[(i - 1) % 3]);
				}
			} else {
				//all leds to same rgb-color
				for (int i = 1; i <= bank_size_; i++) {
					led_map[bank][i].set(msg->data[(i - 1) % 3]);
				}
			}
		}
	}
}

void LedLayer::selfTest(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data) {
		set(selfTest_, (int8_t) 1);
	}
}

void LedLayer::setGlobalBrightness(const std_msgs::UInt16::ConstPtr& msg) {
	uint16_t value = msg->data;
	set(globalBrightness_, value);
}

void LedLayer::globalLedArrayEnable(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data) {
		set(globalLedArrayEnable_, (uint8_t) 1);
	} else {
		set(globalLedArrayEnable_, (uint8_t) 0);
	}
}

void LedLayer::writemultiplexedOut16(
		const canopen_led_node::Led::ConstPtr& msg) {
	int led = msg->led;
	int data_length = msg->data.size();
	if (data_length == 1) {
		set(channelMultiplexer_, led);
		set(outputValue_, msg->data[0]);
	}
}

LedLayer::LedLayer(ros::NodeHandle nh, const std::string &name,
		const boost::shared_ptr<IoBase> & base,
		const boost::shared_ptr<ObjectStorage> storage,
		XmlRpc::XmlRpcValue & options) :
		Layer(name + " Handle"), base_(base), storage_(storage), nh_(nh) {

	//number of leds (multiplied by 3 for RGB)
	if (options.hasMember("leds"))
		leds_ = (const int&) options["leds"];
	else
		leds_ = 0;
	if (options.hasMember("banks"))
		banks_ = (const int&) options["banks"];
	else
		banks_ = 0;
	if (options.hasMember("bank_size"))
		bank_size_ = (const int&) options["bank_size"];
	else
		bank_size_ = 0;
	if (options.hasMember("groups"))
		groups_ = (const int&) options["groups"];
	else
		groups_ = 0;

	//setup storage entries

	// ManufacturerObjects
	storage->entry(nodeID_, 0x2000); // rw: Node ID: default=2
	storage->entry(bitrate_, 0x2001); // rw: CAN Bitrate (kbit): default=500
	storage->entry(globalLedArrayEnable_, 0x2006); //
	storage->entry(globalBrightness_, 0x2007); //
	storage->entry(channelMultiplexer_, 0x2008, 1); //
	storage->entry(outputValue_, 0x2008, 2); //
	storage->entry(selfTest_, 0x200A); //
	storage->entry(bankBrightness_, 0x2100, 0); //
	storage->entry(groupBrightness_, 0x2200, 0); //

	//setup groups, banks and leds
	for (int i = 1; i <= groups_; i++) {
		storage->entry(group_map[i], (0x2200 + i), 0);
		storage->entry(groupBrightness_map[i], 0x2200, i);
	}

	for (int i = 1; i <= banks_; i++) {
		//TODO test if working i > 1
		storage->entry(bank_map[i], (0x2100 + i), 0);
		storage->entry(bankBrightness_map[i], 0x2100, i);

		led_map[i].resize(bank_size_ + 1);
		for (int j = 1; j <= bank_size_; j++) {
			storage->entry(led_map[i][j], (0x2100 + i), j);
		}
	}

	//setup callbacks
	selfTest_sub_ = nh.subscribe("selftest", 1, &LedLayer::selfTest, this);
	set_led_sub_ = nh.subscribe("set_led", 1, &LedLayer::setLed, this);
	globalBrightness_sub_ = nh.subscribe("global_brightness", 1,
			&LedLayer::setGlobalBrightness, this);
	globalLedArrayEnable_sub_ = nh.subscribe("globalLedsEnable", 1,
			&LedLayer::globalLedArrayEnable, this);
	writemultiplexedOut16_sub_ = nh.subscribe("writemultiplexedOut16", 1,
			&LedLayer::writemultiplexedOut16, this);
}

void LedLayer::handleRead(LayerStatus &status,
		const LayerState &current_state) {

}
void LedLayer::handleWrite(LayerStatus &status,
		const LayerState &current_state) {

}
void LedLayer::handleInit(LayerStatus &status) {

	/* init channel_map, needs running cannode to get group_size */
	for (int i = 1; i <= groups_; i++) {
		int group_size = group_map[i].get();
		channel_map[i].resize(group_size + 1);
		for (int j = 1; j <= group_size; j++) {
			storage_->entry(channel_map[i][j], (0x2200 + i), j);
		}
	}
}
