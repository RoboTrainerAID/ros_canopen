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
  ClassAllocator<canopen::IoBase> led_allocator_;
  boost::shared_ptr< LayerGroupNoDiag<IoBase> > leds_;
  // boost::shared_ptr<LedLayer> led_layer_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger)
    {
      std::string name = params["name"];
      std::string &channel = name;
      ROS_INFO("adding node %s", name.c_str());
      
        //if(params.hasMember("channel")) joint.assign(params["channel"]);

        std::string alloc_name = "canopen::IO401::Allocator";
        if(params.hasMember("led_allocator")) alloc_name.assign(params["led_allocator"]);

        XmlRpcSettings settings;
        if(params.hasMember("led_layer")) settings = params["led_layer"];

        boost::shared_ptr<IoBase> led;

        try{
            led = led_allocator_.allocateInstance(alloc_name, name + "_led", node->getStorage(), settings);
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            return false;
        }

        if(!led){
            ROS_ERROR_STREAM("Could not allocate led.");
            return false;
        }
        leds_->add(led);
        logger->add(led);

        return true;
    }


public:
    LedChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv), led_allocator_("canopen_401", "canopen::IoBase::Allocator"){}

    virtual bool setup() {
        ROS_INFO("resetting layers");
        //led_layer_.reset( new LedLayer());
        leds_.reset( new LayerGroupNoDiag<IoBase>("Led Layer"));


        if(RosChain::setup()){
            ROS_INFO("adding");
            //add(led_layer_);      
            add(leds_);

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
