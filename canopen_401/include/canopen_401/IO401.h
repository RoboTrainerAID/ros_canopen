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
	void writeAnalogOut16(const std_msgs::Int16::ConstPtr& msg);
	void writeDigitalOut8(const std_msgs::UInt8::ConstPtr& msg);
	
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
			IoBase(name), storage_(storage) {
			  
		ros::NodeHandle n;
		use_401_ = settings.get_optional<bool>("use_401", false);
		
		
		// Standard 401 Objects
		storage->entry(DeviceType_, 0x1000); // ro: Device Type
		storage->entry(ErrorRegister_, 0x1001); // ro: Error Register
		// Error
		storage->entry(StandardErrorField_, 0x1003, 0x01); // ro: Standard Error Field
		// Manufacturer Software Version
		storage->entry(ManufacturerSoftwareVersion_, 0x100A); // ro: Manufacturer Software Version
		storage->entry(emergencyCOBID_, 0x1014); // rw: Emergency COB ID
		// Identity
		storage->entry(VendorID_, 0x1018, 0x01); // ro: Vendor ID
		storage->entry(ProductCode_, 0x1018, 0x02); // ro: Product Code
		storage->entry(RevisionNumber_, 0x1018, 0x03); // ro: Revision Number
		storage->entry(SerialNumber_, 0x1018, 0x04); // ro: Serial Number
		// Server SDO Parameter
		storage->entry(COBIDReceive_, 0x1200, 0x01); // ro: COB ID Client to Server (Receive SDO)
		storage->entry(COBIDTransmit_, 0x1200, 0x02); // ro: COB ID Server to Client (Transmit SDO)
		
		if (use_401_) {
		  
		  // Receive PDO 1 Parameter
		  storage->entry(RPDO1MaxSub_, 0x1400, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(rPDO1COBID_, 0x1400, 0x01); // rw: COB ID used by PDO
		  storage->entry(rPDO1TxType_, 0x1400, 0x02); // rw: Transmission Type
		  storage->entry(rPDO1InhibitT_, 0x1400, 0x03); // rw: Inhibit Time
		  storage->entry(rPDO1EventTimer_, 0x1400, 0x05); // rw: Event Timer
		  storage->entry(rPDO1SyncStartVal_, 0x1400, 0x06); // rw: SYNC start value
		  // Receive PDO 2 Parameter
		  storage->entry(RPDO2MaxSub_, 0x1401, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(rPDO2COBID_, 0x1401, 0x01); // rw: COB ID used by PDO
		  storage->entry(rPDO2TxType_, 0x1401, 0x02); // rw: Transmission Type
		  storage->entry(rPDO2InhibitT_, 0x1401, 0x03); // rw: Inhibit Time
		  storage->entry(rPDO2EventTimer_, 0x1401, 0x05); // rw: Event Timer
		  storage->entry(rPDO2SyncStartVal_, 0x1401, 0x06); // rw: SYNC start value
		  // Receive PDO 3 Parameter
		  storage->entry(RPDO3MaxSub_, 0x1402, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(rPDO3COBID_, 0x1402, 0x01); // rw: COB ID used by PDO
		  storage->entry(rPDO3TxType_, 0x1402, 0x02); // rw: Transmission Type
		  storage->entry(rPDO3InhibitT_, 0x1402, 0x03); // rw: Inhibit Time
		  storage->entry(rPDO3EventTimer_, 0x1402, 0x05); // rw: Event Timer
		  storage->entry(rPDO3SyncStartVal_, 0x1402, 0x06); // rw: SYNC start value
		  // Receive PDO 4 Parameter
		  storage->entry(RPDO4MaxSub_, 0x1403, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(rPDO4COBID_, 0x1403, 0x01); // rw: COB ID used by PDO
		  storage->entry(rPDO4TxType_, 0x1403, 0x02); // rw: Transmission Type
		  storage->entry(rPDO4InhibitT_, 0x1403, 0x03); // rw: Inhibit Time
		  storage->entry(rPDO4EventTimer_, 0x1403, 0x05); // rw: Event Timer
		  storage->entry(rPDO4SyncStartVal_, 0x1403, 0x06); // rw: SYNC start value
		  
		  
		  // Transmit PDO 1 Parameter
		  storage->entry(TPDO1MaxSub_, 0x1800, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(tPDO1COBID_, 0x1800, 0x01); // rw: COB ID used by PDO
		  storage->entry(tPDO1TxType_, 0x1800, 0x02); // rw: Transmission Type
		  storage->entry(tPDO1InhibitT_, 0x1800, 0x03); // rw: Inhibit Time
		  storage->entry(tPDO1EventTimer_, 0x1800, 0x05); // rw: Event Timer
		  storage->entry(tPDO1SyncStartVal_, 0x1800, 0x06); // rw: SYNC start value
		  // Transmit PDO 2 Parameter
		  storage->entry(TPDO2MaxSub_, 0x1801, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(tPDO2COBID_, 0x1801, 0x01); // rw: COB ID used by PDO
		  storage->entry(tPDO2TxType_, 0x1801, 0x02); // rw: Transmission Type
		  storage->entry(tPDO2InhibitT_, 0x1801, 0x03); // rw: Inhibit Time
		  storage->entry(tPDO2EventTimer_, 0x1801, 0x05); // rw: Event Timer
		  storage->entry(tPDO2SyncStartVal_, 0x1801, 0x06); // rw: SYNC start value
		  // Transmit PDO 3 Parameter
		  storage->entry(TPDO3MaxSub_, 0x1802, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(tPDO3COBID_, 0x1802, 0x01); // rw: COB ID used by PDO
		  storage->entry(tPDO3TxType_, 0x1802, 0x02); // rw: Transmission Type
		  storage->entry(tPDO3InhibitT_, 0x1802, 0x03); // rw: Inhibit Time
		  storage->entry(tPDO3EventTimer_, 0x1802, 0x05); // rw: Event Timer
		  storage->entry(tPDO3SyncStartVal_, 0x1802, 0x06); // rw: SYNC start value
		  // Transmit PDO 4 Parameter
		  storage->entry(TPDO4MaxSub_, 0x1803, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(tPDO4COBID_, 0x1803, 0x01); // rw: COB ID used by PDO
		  storage->entry(tPDO4TxType_, 0x1803, 0x02); // rw: Transmission Type
		  storage->entry(tPDO4InhibitT_, 0x1803, 0x03); // rw: Inhibit Time
		  storage->entry(tPDO4EventTimer_, 0x1803, 0x05); // rw: Event Timer
		  storage->entry(tPDO4SyncStartVal_, 0x1803, 0x06); // rw: SYNC start value
		  // Transmit PDO 5 Parameter
		  storage->entry(TPDO5MaxSub_, 0x1804, 0x00); // ro: Highest SubIndex Supported
		  storage->entry(tPDO5COBID_, 0x1804, 0x01); // rw: COB ID used by PDO
		  storage->entry(tPDO5TxType_, 0x1804, 0x02); // rw: Transmission Type
		  storage->entry(tPDO5InhibitT_, 0x1804, 0x03); // rw: Inhibit Time
		  storage->entry(tPDO5EventTimer_, 0x1804, 0x05); // rw: Event Timer
		  storage->entry(tPDO5SyncStartVal_, 0x1804, 0x06); // rw: SYNC start value
		  
		  
		  
		  // Read Inputs 16 Bit
		  //storage->entry(supported_drive_modes_, 0x6100, 0x00); // ro: Number of Input 16 bit: default=1
		  //storage->entry(readDigitalIn16_, 0x6100, 0x01); // ro: Read Inputs 0x1 to 0x10: default=0
		  //readDigitalIn16_pub_ = n.subscribe("readDigitalIn16", 1, &IO401::readDigitalIn16, this);
		  
		  // Write Outputs 8 Bit
		  //storage->entry(supported_drive_modes_, 0x6200, 0x00); // ro: Number of Output 8 Bit: default=1
		  storage->entry(writeDigitalOut8_, 0x6200, 1); // rw: Write Outputs 0x1 to 0x8
		  writeDigitalOut8_sub_ = n.subscribe("writeDigitalOut8", 1, &IO401::writeDigitalOut8, this);
		  /*
		   * //Read Analogue Input 16 Bit
		   * storage->entry(supported_drive_modes_, 0x6401, 0x00); // ro: Number of Analogue Input 16 Bit
		   * storage->entry(supported_drive_modes_, 0x6401, 0x01); // ro: Analogue Input 1
		   */
		  // Write Analogue Output 16 Bit
		  //storage->entry(supported_drive_modes_, 0x6411, 0x00); // ro: Number of Analogue Input 16 Bit
		  storage->entry(writeAnalogOut16_, 0x6411, 0x01); // rw: Analogue Output 1
		  writeAnalogOut16_sub_ = n.subscribe("writeAnalogOut16", 1, &IO401::writeAnalogOut16, this);
  
		}
		
	}

	class Allocator: public IoBase::Allocator {
	public:
		virtual boost::shared_ptr<IoBase> allocate(const std::string &name,
				boost::shared_ptr<ObjectStorage> storage,
				const canopen::Settings &settings);

	};

private:
  
	boost::shared_ptr<ObjectStorage> storage_;
	bool use_401_;
	ros::Subscriber writeDigitalOut8_sub_, writeAnalogOut16_sub_;
	
	canopen::ObjectStorage::Entry<uint8_t> writeDigitalOut8_, ErrorRegister_,
	  RPDO1MaxSub_, RPDO2MaxSub_,RPDO3MaxSub_,RPDO4MaxSub_,
	  rPDO1TxType_, rPDO2TxType_, rPDO3TxType_, rPDO4TxType_,
	  rPDO1SyncStartVal_, rPDO2SyncStartVal_, rPDO3SyncStartVal_, rPDO4SyncStartVal_;
	  
	canopen::ObjectStorage::Entry<uint8_t> TPDO1MaxSub_, TPDO2MaxSub_, TPDO3MaxSub_, TPDO4MaxSub_, TPDO5MaxSub_,
	  tPDO1TxType_, tPDO2TxType_, tPDO3TxType_, tPDO4TxType_, tPDO5TxType_,
	  tPDO1SyncStartVal_, tPDO2SyncStartVal_, tPDO3SyncStartVal_, tPDO4SyncStartVal_, tPDO5SyncStartVal_;
	  
	canopen::ObjectStorage::Entry<uint16_t> readDigitalIn16_, rPDO1InhibitT_, rPDO2InhibitT_, rPDO3InhibitT_, rPDO4InhibitT_, 
	rPDO1EventTimer_, rPDO2EventTimer_, rPDO3EventTimer_, rPDO4EventTimer_, 
	tPDO1InhibitT_, tPDO2InhibitT_, tPDO3InhibitT_, tPDO4InhibitT_, tPDO5InhibitT_,
	tPDO1EventTimer_, tPDO2EventTimer_, tPDO3EventTimer_, tPDO4EventTimer_, tPDO5EventTimer_;
	
	canopen::ObjectStorage::Entry<int16_t> writeAnalogOut16_;
	
	canopen::ObjectStorage::Entry<uint32_t> DeviceType_, StandardErrorField_, emergencyCOBID_, VendorID_,
	ProductCode_, RevisionNumber_, SerialNumber_, COBIDReceive_, COBIDTransmit_,
	rPDO1COBID_, rPDO2COBID_, rPDO3COBID_, rPDO4COBID_,
	tPDO1COBID_, tPDO2COBID_, tPDO3COBID_, tPDO4COBID_, tPDO5COBID_;
	//canopen::ObjectStorage::Entry<int32_t> ;
	
	canopen::ObjectStorage::Entry<std::string> ManufacturerSoftwareVersion_;
	
	std::map<int, canopen::ObjectStorage::Entry<uint8_t> > rpdo_, tpdo_;
	std::map<int, std::vector< canopen::ObjectStorage::Entry<uint32_t> > > rpdo_map, tpdo_map;
	
};

}

#endif
