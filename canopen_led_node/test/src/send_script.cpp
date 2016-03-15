#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8.h"
#include <canopen_led_node/Led.h>
#include <iostream>

int main(int argc, char **argv)
{
  std::string com_ = " ";
  bool extend = false;
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher setLed_pub = n.advertise<canopen_led_node::Led>("/set_led", 10);
  ros::Rate loop_rate(4);
  
  ros::spinOnce();
  loop_rate.sleep();

  if(argc >= 5) {
    //###########################
    canopen_led_node::Led led_msg;
    led_msg.group = atoi(argv[1]); 
    led_msg.bank = atoi(argv[2]);
    led_msg.led = atoi(argv[3]);
    //data length: 1, 3
    for (int i = 4; i < argc; i++) {
      led_msg.data.push_back(atoi(argv[i]));
    }

    if (ros::ok()) {
      setLed_pub.publish(led_msg);
    } 
  }
  
  
  ros::spinOnce();
  loop_rate.sleep();
  return 0;
}
