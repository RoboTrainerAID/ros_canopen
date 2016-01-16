#include <canopen_led_node/canopen_led_layer.h>

/* motor_node/robot_layer.cpp
   402/motor.cpp|.h
   402/base.h

   -ledlayer object from config representing ea led-able node
   -states of leds/banks/groups of ea ledlayer obj
   -request dimensions of the ledlayer(amount: channels,banks,groups)

*/
using namespace canopen;


HandleLayer::HandleLayer(
        const std::string &name, const boost::shared_ptr<IoBase> & base, const boost::shared_ptr<ObjectStorage> storage,
        XmlRpc::XmlRpcValue & options)
: Layer(name + " Handle"), base_(base), variables_(storage) {


   //if(options.hasMember("pos_to_device")) p2d = (const std::string&) options["pos_to_device"];


}


LedLayer::LedLayer(ros::NodeHandle nh) : LayerGroupNoDiag<HandleLayer>("LedLayer"), nh_(nh)
{

}
