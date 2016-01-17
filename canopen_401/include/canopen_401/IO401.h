#ifndef CANOPEN_401_IO_H
#define CANOPEN_401_IO_H

#include <canopen_401/base.h>
#include <canopen_master/canopen.h>
#include <boost/function.hpp>
#include <boost/container/flat_map.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <limits>
#include <algorithm>

namespace canopen
{

class Command401 {
  Command401();
  public:
    enum ControlWord
    {
        CW_Switch_On=0,
        CW_Enable_Voltage=1,
        CW_Quick_Stop=2,
        CW_Enable_Operation=3,
        CW_Operation_mode_specific0=4,
        CW_Operation_mode_specific1=5,
        CW_Operation_mode_specific2=6,
        CW_Fault_Reset=7,
        CW_Halt=8,
        CW_Operation_mode_specific3=9,
        // CW_Reserved1=10,
        CW_Manufacturer_specific0=11,
        CW_Manufacturer_specific1=12,
        CW_Manufacturer_specific2=13,
        CW_Manufacturer_specific3=14,
        CW_Manufacturer_specific4=15,
    };
  
};
  
template<uint16_t MASK> class WordAccessor{
    uint16_t &word_;
public:
   enum ControlWord
    {
        CW_Switch_On=0,
        CW_Enable_Voltage=1,
        CW_Quick_Stop=2,
        CW_Enable_Operation=3,
        CW_Operation_mode_specific0=4,
        CW_Operation_mode_specific1=5,
        CW_Operation_mode_specific2=6,
        CW_Fault_Reset=7,
        CW_Halt=8,
        CW_Operation_mode_specific3=9,
        // CW_Reserved1=10,
        CW_Manufacturer_specific0=11,
        CW_Manufacturer_specific1=12,
        CW_Manufacturer_specific2=13,
        CW_Manufacturer_specific3=14,
        CW_Manufacturer_specific4=15,
    };
  
    WordAccessor(uint16_t &word) : word_(word) {}
    bool set(uint8_t bit){
        uint16_t val = MASK & (1<<bit);
        word_ |= val;
        return val;
   }
    bool reset(uint8_t bit){
        uint16_t val = MASK & (1<<bit);
        word_ &= ~val;
        return val;
    }
    bool get(uint8_t bit) const { return word_ & (1<<bit); }
    uint16_t get() const { return word_ & MASK; }
    WordAccessor & operator=(const uint16_t &val){
        uint16_t was = word_;
        word_ = (word_ & ~MASK) | (val & MASK);
        return *this;
    }

   
};
  
class Mode {
public:
    const uint16_t mode_id_;
    Mode(uint16_t id) : mode_id_(id) {}
    typedef WordAccessor<(1<<Command401::CW_Operation_mode_specific0)|(1<<Command401::CW_Operation_mode_specific1)|(1<<Command401::CW_Operation_mode_specific2)|(1<<Command401::CW_Operation_mode_specific3)> OpModeAccesser;
    virtual bool start() = 0;
    virtual bool read(const uint16_t &sw) = 0;
    virtual bool write(OpModeAccesser& cw) = 0;
    virtual bool setTarget(const double &val) { LOG("not implemented"); return false; }
    virtual ~Mode() {}
};


template<typename T> class ModeTargetHelper : public Mode {
    T target_;
    boost::atomic<bool> has_target_;

public:
    ModeTargetHelper(uint16_t mode) : Mode (mode) {}
    bool hasTarget() { return has_target_; }
    T getTarget() { return target_; }
    virtual bool setTarget(const double &val) {
        if(isnan(val)){
            LOG("target command is not a number");
            return false;
        }

        using boost::numeric_cast;
        using boost::numeric::positive_overflow;
        using boost::numeric::negative_overflow;

        try
        {
            target_= numeric_cast<T>(val);
        }
        catch(negative_overflow&) {
            LOG("Command " << val << " does not fit into target, clamping to min limit");
            target_= std::numeric_limits<T>::min();
        }
        catch(positive_overflow&) {
            LOG("Command " << val << " does not fit into target, clamping to max limit");
            target_= std::numeric_limits<T>::max();
        }
        catch(...){
            LOG("Was not able to cast command " << val);
            return false;
        }

        has_target_ = true;
        return true;
    }
    virtual bool start() { has_target_ = false; return true; }
};  
  
template<uint16_t ID, typename TYPE, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK> class ModeForwardHelper : public ModeTargetHelper<TYPE> {
    canopen::ObjectStorage::Entry<TYPE> target_entry_;
public:
    ModeForwardHelper(boost::shared_ptr<ObjectStorage> storage) : ModeTargetHelper<TYPE>(ID) {
        if(SUB) storage->entry(target_entry_, OBJ, SUB);
        else storage->entry(target_entry_, OBJ);
    }
    virtual bool read(const uint16_t &sw) { return true;}
    virtual bool write(Mode::OpModeAccesser& cw) {
        if(this->hasTarget()){
            cw = cw.get() | CW_MASK;
            target_entry_.set(this->getTarget());
            return true;
        }else{
            cw = cw.get() & ~CW_MASK;
            return false;
        }
    }
};

typedef ModeForwardHelper<IoBase::Profiled_Velocity, int32_t, 0x607A, 0, 0> ProfiledVelocityMode;  
  




class IO401 : public IoBase
{
public:

    IO401(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings)
    : IoBase(name)
      
    {
      /*
        storage->entry(status_word_entry_, 0x6041);
        storage->entry(control_word_entry_, 0x6040);
        storage->entry(op_mode_display_, 0x6061);
        storage->entry(op_mode_, 0x6060);
        */
      try{
            storage->entry(supported_drive_modes_, 0x6502);
        }
        catch(...){
        }
    }
    
    virtual bool setTarget(double val);
    
    template<typename T> bool registerMode(uint16_t mode) {
        return mode_allocators_.insert(std::make_pair(mode, boost::bind(&IO401::createAndRegister<T>, this, mode))).second;
    }
    template<typename T, typename T1> bool registerMode(uint16_t mode, const T1& t1) {
        return mode_allocators_.insert(std::make_pair(mode, boost::bind(&IO401::createAndRegister<T,T1>, this, mode, t1))).second;
    }
    template<typename T, typename T1, typename T2> bool registerMode(uint16_t mode, const T1& t1, const T2& t2) {
        return mode_allocators_.insert(std::make_pair(mode, boost::bind(&IO401::createAndRegister<T,T1,T2>, this, mode, t1, t2))).second;
    }
    
    virtual void registerDefaultModes(boost::shared_ptr<ObjectStorage> storage){
        registerMode<ProfiledVelocityMode> (IoBase::Profiled_Velocity, storage);
        
    }

    class Allocator : public IoBase::Allocator{
    public:
        virtual boost::shared_ptr<IoBase> allocate(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings);
    };
    
protected:
 
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleDiag(LayerReport &report);
    virtual void handleInit(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);

private:
  
    template<typename T> void createAndRegister0(uint16_t mode){
        if(isModeSupportedByDevice(mode)) registerMode(mode, boost::shared_ptr<Mode>(new T()));
    }
    template<typename T, typename T1> void createAndRegister(uint16_t mode, const T1& t1){
        if(isModeSupportedByDevice(mode)) registerMode(mode, boost::shared_ptr<Mode>(new T(t1)));
    }
    template<typename T, typename T1, typename T2> void createAndRegister(uint16_t mode, const T1& t1, const T2& t2){
        if(isModeSupportedByDevice(mode)) registerMode(mode, boost::shared_ptr<Mode>(new T(t1,t2)));
    }

    virtual bool isModeSupportedByDevice(uint16_t mode);
    void registerMode(uint16_t id, const boost::shared_ptr<Mode> &m);
    
    boost::shared_ptr<Mode> allocMode(uint16_t mode);
    
    uint16_t control_word_;
    
    boost::mutex map_mutex_;
    boost::unordered_map<uint16_t, boost::shared_ptr<Mode> > modes_;
    boost::unordered_map<uint16_t, boost::function<void()> > mode_allocators_;
    
    boost::shared_ptr<Mode> selected_mode_;
    boost::mutex mode_mutex_;

    canopen::ObjectStorage::Entry<uint32_t>  supported_drive_modes_;
    
 /*
    canopen::ObjectStorage::Entry<uint16_t>  status_word_entry_;
    canopen::ObjectStorage::Entry<uint16_t >  control_word_entry_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_display_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_;
    
    */
};

}

#endif