#undef ESP32
#include <ros.h> //USB serial mode
#define ESP32

//ros
ros::NodeHandle nh;

//message
std_msgs::String str_msg;
ros::Publisher pub("chatter", &str_msg);
char hello[13] = "hello world!";

//pin
int const sonsor_signal_pin = 34;


void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  //publish message
  pub.publish( &hello );
  
  nh.spinOnce();
  delay(1000);
}
