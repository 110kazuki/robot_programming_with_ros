/* [ros motor control with ESCON]
 *  version : 1.0 (20220422)
 *  Auther  : Kazuki Ito
 */

#undef   ESP32
#include <ros.h> //ESP32 USB serial mode
#define  ESP32
#include <std_msgs/Int16.h>

//motor driver property
#define PWM_FREQUENCY  3000 //PWM pulse frequency
#define PWM_RESOLUTION 10   //Duty ratio resolution, 10bit (0~1023)
#define MOTOR_0        0    //PWM channnel

#define MOTOR_EN_PIN 18     //pin number to connect ESCON EN channel
#define PWM_MT_PIN 17       //pin number to connect ESCON PWM channel
//#dedine CONTROLLER_PIN 

int motor_cmd = 0;         // motor command speed, specified as an integer value from -100 to 100.
int pwm_cmd   = 0;         

//ros setting
ros::NodeHandle nh;

void messageCb( const std_msgs::Int16& ctrl_msg ){
  motor_cmd = ctrl_msg.data;
}

//subscriber
  ros::Subscriber<std_msgs::Int16> sub( "motor_ctrl", &messageCb );
//publisher
  //ros::Publisher pub("l_curve_status", &status_msg);

void setup() {
  //ros mode
  nh.getHardware()->setBaud( 57600 );
  nh.initNode();
  nh.subscribe( sub );
  //nh.advertise(pub);

  //pin setting
  pinMode( MOTOR_EN_PIN, OUTPUT );

  //pwm setting
  ledcSetup( MOTOR_0, PWM_FREQUENCY, PWM_RESOLUTION );
  ledcAttachPin( PWM_MT_PIN, MOTOR_0 );
  
  //desable ESCON (safety)
  digitalWrite( MOTOR_EN_PIN, LOW );
}

void loop() {
  if ( nh.connected() ){
    //convert motor command to duty ratio
    pwm_cmd = constrain( motor_cmd * 4.09, 103, 920 );

    digitalWrite( MOTOR_EN_PIN, HIGH );
    ledcWrite( MOTOR_0, 511 + pwm_cmd );    

  }else{
    pwm_cmd = 511;
    digitalWrite( MOTOR_EN_PIN, LOW );
    ledcWrite( MOTOR_0, 511 );
  }

  nh.spinOnce();
  delay(5);

}
