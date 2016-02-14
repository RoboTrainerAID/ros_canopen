#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8.h"
#include <canopen_led_node/Led.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher setLed_pub = n.advertise<canopen_led_node::Led>("/set_led", 10);
  ros::Publisher writeDigi = n.advertise<std_msgs::UInt8>("/writeDigitalOut8", 10);
  ros::Rate loop_rate(1);
  
  ros::spinOnce();    
  loop_rate.sleep();
  
  uint16_t data[] = {50,50,50};

  //###########################
  canopen_led_node::Led led_msg;
  led_msg.group = 0;
  led_msg.bank = 0;
  led_msg.led = 0;
  led_msg.data.push_back(data[0]);
   
  
  if (ros::ok()){
    // Send the message 
    ROS_INFO("Data length 1");
    ROS_INFO("G1");
    led_msg.group = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("G2");
    led_msg.group = 2;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("G2 L1");
    led_msg.led = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B1 L1");
    led_msg.group = 0;
    led_msg.bank = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B1");
    led_msg.led = 0;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B2");
    led_msg.bank = 2;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("Data length 3");
    led_msg.data.push_back(data[1]); 
    led_msg.data.push_back(data[2]); 
    
    ROS_INFO("G1");
    led_msg.group = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("G2");
    led_msg.group = 2;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("G2 L1");
    led_msg.led = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B1 L1");
    led_msg.group = 0;
    led_msg.bank = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B1");
    led_msg.led = 0;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B2");
    led_msg.bank = 2;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("Data length banksize");
    for (int i = 3; i < 15; i++) {
      led_msg.data.push_back(data[i%3]);
    }
    
    ROS_INFO("B2");
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    ROS_INFO("B1");
    led_msg.bank = 1;
    setLed_pub.publish(led_msg);
    loop_rate.sleep();
    
    
    ROS_INFO("done");
  }
  
  
  return 0;
}
