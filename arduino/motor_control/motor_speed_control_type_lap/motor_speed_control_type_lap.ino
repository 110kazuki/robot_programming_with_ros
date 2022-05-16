#undef   ESP32
#include <ros.h> //ESP32 USB serial mode
#define  ESP32
#include <std_msgs/Int16.h>

/* ==== User setting ==== */
//motor driver property
#define PWM_FREQUENCY  3000     //PWM pulse frequency
#define PWM_RESOLUTION 10       //Duty ratio resolution, 10bit (0~1023)
#define PWM_duty_limit_under 10 //[%] Lower limit of PWM duty ratio
#define PWM_duty_limit_upper 90 //[%] Upper limit of PWM duty ratio
#define MOTOR_0 0               //PWM channel

//ESP32 pin number setting
#define PWM_PIN      17         //pin number connect with PWM input channel
#define MOTOR_EN_PIN 18         //pin number connect with enable channel (option)

//motor property
double const gear_ratio = 1.0;

//encoder setting
int    const counter_multiplitaion = 4;
/* ======================== */

int motor_cmd = 0;          // motor command speed, specified as an integer value from -100 to 100.
int pwm_cmd   = 0;         
int PWM_duty_range = (pow( 2, PWM_RESOLUTION )-1) * (PWM_duty_limit_under + (100-PWM_duty_limit_upper)) * 0.005;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
