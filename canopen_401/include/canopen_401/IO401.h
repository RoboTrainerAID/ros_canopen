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

namespace canopen {

class IO401: public IoBase {
protected:
	void cb(const std_msgs::UInt16::ConstPtr& msg);
	void write(const std_msgs::UInt8::ConstPtr& msg);
	
	virtual void handleRead(LayerStatus &status,
			const LayerState &current_state);
	virtual void handleWrite(LayerStatus &status,
			const LayerState &current_state);
	virtual void handleDiag(LayerReport &report);
	virtual void handleInit(LayerStatus &status);
	virtual void handleShutdown(LayerStatus &status);
	virtual void handleHalt(LayerStatus &status);
	virtual void handleRecover(LayerStatus &status);

public:
	template<typename T> bool set(T & entry, const typename T::type &value) {
		try {
			entry.set(value);
		} catch (...) {
			return false;
		}
		return true;
	}



	IO401(const std::string &name, boost::shared_ptr<ObjectStorage> storage,
			const canopen::Settings &settings) :
			IoBase(name)

	{

		ros::NodeHandle n;
		sub_ = n.subscribe("global_brightness", 1, &IO401::cb, this);
		write_ = n.subscribe("writeDigitalOut8", 1, &IO401::write, this);
		
		if (settings.get_optional<bool>("use_401", false)) {
		    //start 401 subscriber
		  
		}
		

		// Standard 401 Objects
		/*
		storage->entry(supported_drive_modes_, 0x1000); // ro: Device Type
		storage->entry(supported_drive_modes_, 0x1001); // ro: Error Register
		// Identity
		storage->entry(supported_drive_modes_, 0x1018, 0x01); // ro: Vendor ID
		storage->entry(supported_drive_modes_, 0x1018, 0x02); // ro: Product Code
		storage->entry(supported_drive_modes_, 0x1018, 0x03); // ro: Revision Number
		storage->entry(supported_drive_modes_, 0x1018, 0x04); // ro: Serial Number
		// Error
		storage->entry(supported_drive_modes_, 0x1003, 0x01); // ro: Standard Error Field
		// Manufacturer Software Version
		storage->entry(supported_drive_modes_, 0x100A); // ro: Manufacturer Software Version
		storage->entry(supported_drive_modes_, 0x1014); // rw: Emergency COB ID
		// Server SDO Parameter
		storage->entry(supported_drive_modes_, 0x1200, 0x01); // ro: COB ID Client to Server (Receive SDO)
		storage->entry(supported_drive_modes_, 0x1200, 0x02); // ro: COB ID Server to Client (Transmit SDO)
		// Receive PDO 1 Parameter
		storage->entry(supported_drive_modes_, 0x1400, 0x00); // ro: Highest SubIndex Supported
		storage->entry(supported_drive_modes_, 0x1400, 0x01); // rw: COB ID used by PDO
		storage->entry(supported_drive_modes_, 0x1400, 0x02); // rw: Transmission Type
		storage->entry(supported_drive_modes_, 0x1400, 0x03); // rw: Inhibit Time
		storage->entry(supported_drive_modes_, 0x1400, 0x05); // rw: Event Timer
		storage->entry(supported_drive_modes_, 0x1400, 0x06); // rw: SYNC start value
		// Receive PDO 2 Parameter
		storage->entry(supported_drive_modes_, 0x1401, 0x00); // ro: Highest SubIndex Supported
		storage->entry(supported_drive_modes_, 0x1401, 0x01); // rw: COB ID used by PDO
		storage->entry(supported_drive_modes_, 0x1401, 0x02); // rw: Transmission Type
		storage->entry(supported_drive_modes_, 0x1401, 0x03); // rw: Inhibit Time
		storage->entry(supported_drive_modes_, 0x1401, 0x05); // rw: Event Timer
		storage->entry(supported_drive_modes_, 0x1401, 0x06); // rw: SYNC start value
		// Receive PDO 3 Parameter
		storage->entry(supported_drive_modes_, 0x1402, 0x00); // ro: Highest SubIndex Supported
		storage->entry(supported_drive_modes_, 0x1402, 0x01); // rw: COB ID used by PDO
		storage->entry(supported_drive_modes_, 0x1402, 0x02); // rw: Transmission Type
		storage->entry(supported_drive_modes_, 0x1402, 0x03); // rw: Inhibit Time
		storage->entry(supported_drive_modes_, 0x1402, 0x05); // rw: Event Timer
		storage->entry(supported_drive_modes_, 0x1402, 0x06); // rw: SYNC start value
		// Receive PDO 4 Parameter
		storage->entry(supported_drive_modes_, 0x1403, 0x00); // ro: Highest SubIndex Supported
		storage->entry(supported_drive_modes_, 0x1403, 0x01); // rw: COB ID used by PDO
		storage->entry(supported_drive_modes_, 0x1403, 0x02); // rw: Transmission Type
		storage->entry(supported_drive_modes_, 0x1403, 0x03); // rw: Inhibit Time
		storage->entry(supported_drive_modes_, 0x1403, 0x05); // rw: Event Timer
		storage->entry(supported_drive_modes_, 0x1403, 0x06); // rw: SYNC start value
		// Receive PDO 1 Mapping
		for (int i = 1; i <= 8; i++)
			storage->entry(supported_drive_modes_, 0x1600, i); // rw: PDO 1 Mapping for an application object i
		// Receive PDO 2 Mapping
		for (int i = 1; i <= 8; i++)
			storage->entry(supported_drive_modes_, 0x1601, i); // rw: PDO 2 Mapping for an application object i
		// Receive PDO 3 Mapping
		for (int i = 1; i <= 8; i++)
			storage->entry(supported_drive_modes_, 0x1602, i); // rw: PDO 3 Mapping for an application object i
		// Receive PDO 4 Mapping
		for (int i = 1; i <= 8; i++)
			storage->entry(supported_drive_modes_, 0x1603, i); // rw: PDO 4 Mapping for an application object i
		// Transmit PDO i Parameter
		for (int i = 0x1800; i <= 0x1804; i++) {
			for (int k = 0; i <= 6; i++)
				if (k != 4)
					storage->entry(supported_drive_modes_, i, k);
			// ro: Highest SubIndex Supported
			// rw: COB ID used by PDO
			// rw: Transmission Type
			// rw: Inhibit Time
			// rw: Event Timer
			// rw: SYNC start value
		}
		// Transmit PDO i Mapping
		for (int i = 0x1A00; i <= 0x1A03; i++) {
			for (int k = 1; i <= 8; i++)
				storage->entry(supported_drive_modes_, i, k);
			// rw: PDO i Mapping for a process data variable k
		}
		// Read Inputs 16 Bit
		storage->entry(supported_drive_modes_, 0x6100, 0x00); // ro: Number of Input 16 bit: default=1
		storage->entry(supported_drive_modes_, 0x6100, 0x01); // ro: Read Inputs 0x1 to 0x10: default=0
		*/
		// Write Outputs 8 Bit
		//storage->entry(supported_drive_modes_, 0x6200, 0x00); // ro: Number of Output 8 Bit: default=1
		storage->entry(writeDigitalOut8_, 0x6200, 1); // rw: Write Outputs 0x1 to 0x8
		
		/*
		// Read Analogue Input 16 Bit
		storage->entry(supported_drive_modes_, 0x6401, 0x00); // ro: Number of Analogue Input 16 Bit
		storage->entry(supported_drive_modes_, 0x6401, 0x01); // ro: Analogue Input 1
		// Write Analogue Output 16 Bit
		storage->entry(supported_drive_modes_, 0x6411, 0x00); // ro: Number of Analogue Input 16 Bit
		storage->entry(supported_drive_modes_, 0x6411, 0x01); // rw: Analogue Output 1
		// ManufacturerObjects
		storage->entry(supported_drive_modes_, 0x2000); // rw: Node ID: default=2
		storage->entry(supported_drive_modes_, 0x2001); // rw: CAN Bitrate (kbit): default=500
		*/
		//storage->entry(supported_drive_modes_uint_16, 0x2006); // rw: enable
		storage->entry(global_brightness_, 0x2007); // rw: global brightness 
		//storage->entry(group_brightness_, 0x2200); // rw: group brightness 
		
		
		
	}

	class Allocator: public IoBase::Allocator {
	public:
		virtual boost::shared_ptr<IoBase> allocate(const std::string &name,
				boost::shared_ptr<ObjectStorage> storage,
				const canopen::Settings &settings);

	};

private:

	ros::Subscriber sub_, write_;
	

	canopen::ObjectStorage::Entry<uint8_t> writeDigitalOut8_;
	canopen::ObjectStorage::Entry<int8_t> supported_drive_modes_int_8;
	canopen::ObjectStorage::Entry<uint16_t> supported_drive_modes_uint_16, global_brightness_;
	canopen::ObjectStorage::Entry<int16_t> group_brightness_;
	canopen::ObjectStorage::Entry<uint32_t> supported_drive_modes_uint_32;
	canopen::ObjectStorage::Entry<int32_t> supported_drive_modes_int_32;

};

}

#endif
