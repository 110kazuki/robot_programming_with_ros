//For ESP32
#undef ESP32
#include <ros.h> //USB serial mode
#define ESP32

//For general arduino device
//#include <ros.h> //USB serial mode


#include "ros1_learning/SensorAD.h"

//ros
ros::NodeHandle nh;

//message
ros1_learning::SensorAD sensor_ad;
ros::Publisher pub("sensor_ad", &sensor_ad);

//pin
int const sonsor_signal_pin = 34;


void setup() {
  //ilitialize ros
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub);
  //pin mode setting
  pinMode(sonsor_signal_pin, INPUT);
}

void loop() {
  //AD convert
  sensor_ad.ad_val = analogRead( sonsor_signal_pin );
  
  //publish message
  pub.publish( &sensor_ad );
  
  nh.spinOnce();
  delay(5);
}
