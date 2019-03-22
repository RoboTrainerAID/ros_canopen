#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/ros_chain.h>

#include <canopen_led_node/canopen_led_layer.h>

using namespace can;
using namespace canopen;



class XmlRpcSettings : public Settings{
public:
    XmlRpcSettings() {}
    XmlRpcSettings(const XmlRpc::XmlRpcValue &v) : value_(v) {}
    XmlRpcSettings& operator=(const XmlRpc::XmlRpcValue &v) { value_ = v; return *this; }
private:
    virtual bool getRepr(const std::string &n, std::string & repr) const {
        if(value_.hasMember(n)){
            std::stringstream sstr;
            sstr << const_cast< XmlRpc::XmlRpcValue &>(value_)[n]; // does not write since already existing
            repr = sstr.str();
            return true;
        }
        return false;
    }
    XmlRpc::XmlRpcValue value_;

};

class LedChain : public RosChain{
  ClassAllocator<canopen::IoBase> io_base_allocator_;
  std::shared_ptr< LayerGroupNoDiag<IoBase> > io_bases_;
  std::shared_ptr<LayerGroupNoDiag<LedLayer> > led_layers_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger)
    {
      std::string name = params["name"];
      std::string &channel = name;
      ROS_INFO("adding node %s", name.c_str());
      

        std::string alloc_name = "canopen::IO401::Allocator";
        if(params.hasMember("led_allocator")) alloc_name.assign(params["led_allocator"]);

        XmlRpcSettings settings;
        if(params.hasMember("led_layer")) settings = params["led_layer"];

	std::shared_ptr<IoBase> io_base;

        try{
            io_base = io_base_allocator_.allocateInstance(alloc_name, name + "_led", node->getStorage(), settings);
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
	    ROS_ERROR_STREAM(">>>>>>>>> Failed led allocator");
            return false;
        }

        if(!io_base){
            ROS_ERROR_STREAM("Could not allocate led.");
            return false;
        }
        io_bases_->add(io_base);
	
	std::shared_ptr<LedLayer> layer( new LedLayer(this->nh_,name, io_base, node->getStorage(), params));
        led_layers_->add(layer);
        logger->add(layer);

        return true;
    }


public:
    LedChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv), io_base_allocator_("canopen_401", "canopen::IoBase::Allocator"){}

    virtual bool setup() {
        ROS_INFO("resetting layers");
        led_layers_.reset( new LayerGroupNoDiag<LedLayer>("LedLayers"));
        io_bases_.reset( new LayerGroupNoDiag<IoBase>("IoBases"));


        if(RosChain::setup()){
            ROS_INFO("adding");
            add(led_layers_);      
            add(io_bases_);

            return true;
        }
        ROS_ERROR_STREAM("Failed RosChain setup");
        return false;
    }

};



int main(int argc, char** argv){
  ros::init(argc, argv, "canopen_led_chain_node");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  ROS_INFO("init chain");
  LedChain chain(nh, nh_priv);
  ROS_INFO("setting up chain");
  if(!chain.setup()){
      return -1;
  }

  ros::waitForShutdown();
  return 0;
}
