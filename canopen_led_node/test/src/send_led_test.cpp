#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/UInt16MultiArray.h"
#include <canopen_led_node/Led.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher testcom_pub = n.advertise<std_msgs::UInt16MultiArray>("/set_B1", 10);
  ros::Publisher setLed_pub = n.advertise<canopen_led_node::Led>("/set_led", 10);
  ros::Rate loop_rate(1);
  
  ros::spinOnce();    
  loop_rate.sleep();
  
  std_msgs::UInt16MultiArray msg;
  
  msg.layout.data_offset = 0;
  msg.layout.dim.resize(1, std_msgs::MultiArrayDimension());
  
  
  msg.layout.dim[0].label = "length";
  msg.layout.dim[0].size = 3;
  msg.layout.dim[0].stride = 3;
  
  
  uint16_t data[] = {0,50,150};
  for (int i = 0; i < 3; i++) {
    msg.data.push_back(data[i]);
  }  
  //###########################
  canopen_led_node::Led led_msg;
  led_msg.group = 1;
  led_msg.bank = 0;
  led_msg.led = 0;
  for (int i = 0; i < 3; i++) {
    led_msg.data.push_back(data[i]);
  } 

    
    if (ros::ok()){
      // Send the message
      //testcom_pub.publish(msg);
      
      setLed_pub.publish(led_msg);
      loop_rate.sleep();
     
      led_msg.led = 1;
      setLed_pub.publish(led_msg);
      loop_rate.sleep();
   
      led_msg.group = 0;
      led_msg.bank = 1;
      setLed_pub.publish(led_msg);
      loop_rate.sleep();
     
      led_msg.led = 0;
      for (int i = 3; i < 15; i++) {
	led_msg.data.push_back(data[i%3]);
      } 
      setLed_pub.publish(led_msg);
      loop_rate.sleep();
      
      ROS_INFO("sent");
      ros::spinOnce();
      loop_rate.sleep();
    }

  
  return 0;
}
