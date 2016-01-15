
#ifndef CANOPEN_LED_NODE_H_
#define CANOPEN_LED_NODE_H_

#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <canopen_master/canopen.h>

class ObjectVariables {
    const boost::shared_ptr<canopen::ObjectStorage> storage_;
    struct Getter {
        boost::shared_ptr<double> val_ptr;
        boost::function<bool(double&)> func;
        bool operator ()() { return func(*val_ptr); }
        template<typename T> Getter(const canopen::ObjectStorage::Entry<T> &entry): func(boost::bind(&Getter::readObject<T>, entry, _1)), val_ptr(new double) { }
        template<typename T> static bool readObject(canopen::ObjectStorage::Entry<T> &entry, double &res){
            T val;
            if(!entry.get(val)) return false;
            res = val;
            return true;
        }
        operator double*() const { return val_ptr.get(); }
    };
    boost::unordered_map<canopen::ObjectDict::Key, Getter> getters_;
public:
    template<const uint16_t dt> static double* func(ObjectVariables &list, const canopen::ObjectDict::Key &key){
        typedef typename canopen::ObjectStorage::DataType<dt>::type type;
        return list.getters_.insert(std::make_pair(key, Getter(list.storage_->entry<type>(key)))).first->second;
    }
    ObjectVariables(const boost::shared_ptr<canopen::ObjectStorage> storage) : storage_(storage) {}
    bool sync(){
        bool ok = true;
        for(boost::unordered_map<canopen::ObjectDict::Key, Getter>::iterator it = getters_.begin(); it != getters_.end(); ++it){
            ok = it->second() && ok;
        }
        return ok;
    }
    double * getVariable(const std::string &n) {
        try{
            if(n.find("obj") == 0){
                canopen::ObjectDict::Key key(n.substr(3));
                boost::unordered_map<canopen::ObjectDict::Key, Getter>::const_iterator it = getters_.find(key);
                if(it != getters_.end()) return it->second;
                return canopen::branch_type<ObjectVariables, double * (ObjectVariables &list, const canopen::ObjectDict::Key &k)>(storage_->dict_->get(key)->data_type)(*this, key);
            }
        }
        catch( const std::exception &e){
            ROS_ERROR_STREAM("Could not find variable '" + n + "', reason: " + boost::diagnostic_information(e));
        }
        return 0;
    }
};

template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_VISIBLE_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_OCTET_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_UNICODE_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_DOMAIN >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }


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

class HandleLayer: public canopen::Layer{    
  
    boost::shared_ptr<canopen::LedBase> base_;
    ObjectVariables variables_;
public:
    HandleLayer(const std::string &name, const boost::shared_ptr<LedBase> & base, const boost::shared_ptr<canopen::ObjectStorage> storage,  XmlRpc::XmlRpcValue & options);

   /*
   
private:
    virtual void handleRead(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleInit(canopen::LayerStatus &status);
    virtual void handleDiag(canopen::LayerReport &report) { }
    virtual void handleShutdown(canopen::LayerStatus &status) {  }
    virtual void handleHalt(canopen::LayerStatus &status) { }
    virtual void handleRecover(canopen::LayerStatus &status) { handleRead(status, Layer::Ready); }
*/
};



class LedLayer : public canopen::LayerGroupNoDiag<HandleLayer>{
    

    ros::NodeHandle nh_;


public:
    LedLayer(ros::NodeHandle nh);
    
};
}


#endif
