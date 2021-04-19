#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define PULSES_PER_REV 210
#define pi 3.14
#define BUFFER_TIME 1000
// -------------------------------ENCODER PINS-----------------------------
volatile byte ENCODERA_MOTOR1 = 19; // Hall sensor A connected to pin 3 (external interrupt)
volatile byte ENCODERA_MOTOR2 = 18;
volatile byte ENCODERB_MOTOR1 = 3;
volatile byte ENCODERB_MOTOR2 = 2;

// --------------------------------MOTOR PINS---------------------------------
const byte MOTOR1_EN = 10;
const byte MOTOR1A = 8;
const byte MOTOR1B = 9;
const byte MOTOR2_EN = 7;
const byte MOTOR2A = 5;
const byte MOTOR2B = 6;

// ----------------------------ENCODER TICK VARIABLES-------------------------
volatile long rotary_encoder_left = 0;
volatile long rotary_encoder_right = 0;
float left_rev_count = 0.0, right_rev_count = 0.0, l_dist_val = 0.0, r_dist_val = 0.0;

volatile boolean fired_left, up_left, fired_right, up_right;
unsigned long current_time = 0, previous_time = 0;
const float wheel_rad = 0.0335;
const float wheel_sep = 0.173;
const float dist_per_rev = (2*pi*wheel_rad);
float w_r, w_l, speed_ang, speed_lin;

// ******************** ROS STUFF *****************
ros::NodeHandle n;

std_msgs::Float32 l_dist_msg, r_dist_msg;

ros::Publisher l_dist("l_dist", &l_dist_msg);
ros::Publisher r_dist("r_dist", &r_dist_msg);

void cmd_vel_callback( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad)+ ((speed_ang*wheel_sep)/(2.0*wheel_rad));    // speed_ang*2.91 -2.91
  w_l = (speed_lin/wheel_rad)- ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_vel_callback );

// ********************** FUNCTION DECLARATIONS *********************
//void update_right_encoder(void);
//void update_left_encoder(void);

void to_odom_publisher(void);
float update_left(void);
float update_right(void);

void isr_left(void);
void isr_right(void);

void move_l_motor(int pwm);
void move_r_motor(int pwm);

void pinmode_setup()
{
    pinMode( MOTOR1_EN, OUTPUT);
    pinMode( MOTOR1A , OUTPUT);
    pinMode( MOTOR1B , OUTPUT);

    pinMode( MOTOR2_EN, OUTPUT);
    pinMode( MOTOR2A , OUTPUT);
    pinMode( MOTOR2B , OUTPUT);

    pinMode( ENCODERA_MOTOR1, INPUT_PULLUP);
    pinMode( ENCODERB_MOTOR1, INPUT_PULLUP);
    pinMode( ENCODERA_MOTOR2, INPUT_PULLUP);
    pinMode( ENCODERB_MOTOR2, INPUT_PULLUP);
}

void pin_init()
{
    digitalWrite(MOTOR1_EN, HIGH);
    digitalWrite(MOTOR1A, LOW);
    digitalWrite(MOTOR1B, LOW);

    digitalWrite(MOTOR2_EN, HIGH);
    digitalWrite(MOTOR2A, LOW);
    digitalWrite(MOTOR2B, LOW);
}

void setup() {

    pinmode_setup();
    EncoderInit();

    pin_init();

    n.initNode();
    n.advertise(l_dist);
    n.advertise(r_dist);
    n.subscribe(sub_cmd_vel);

    current_time=millis();
    previous_time = current_time;
}

void loop() {
  
  current_time = millis();
  move_l_motor(w_l*10);
  move_r_motor(w_r*10);

  if (current_time - previous_time >= BUFFER_TIME)
  {
    to_odom_publisher();
    previous_time = millis();
    rotary_encoder_left = 0;
    rotary_encoder_right = 0;
  }

  //rotary_encoder_left = 0;
  //rotary_encoder_right = 0;
 
  n.spinOnce();
}

void EncoderInit()
{
  attachInterrupt(digitalPinToInterrupt(ENCODERA_MOTOR1), isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERA_MOTOR2), isr_right, CHANGE);
}

//void update_right_encoder()
//{
//  rotary_encoder_right++;
//}
//
//
//void update_left_encoder()
//{
//  rotary_encoder_left++;
//}

float update_left()
{
  left_rev_count = (float)rotary_encoder_left/PULSES_PER_REV;
  l_dist_val = (float)left_rev_count * dist_per_rev;
  return l_dist_val/2.0;
}

float update_right()
{
  right_rev_count = (float)rotary_encoder_right/PULSES_PER_REV;
  r_dist_val = (float)right_rev_count * dist_per_rev;
  return r_dist_val/2.0;
}

void to_odom_publisher(void)
{
  l_dist_msg.data = update_left();
  r_dist_msg.data = update_right();

  l_dist.publish(&l_dist_msg);
  r_dist.publish(&r_dist_msg);
  return;
}

void move_l_motor(int pwm){
    if (pwm > 0){
    analogWrite(MOTOR1_EN, pwm);
    digitalWrite(MOTOR1A, HIGH);
    digitalWrite(MOTOR1B, LOW);
    }
    if (pwm < 0){
    pwm=abs(pwm);
    analogWrite(MOTOR1_EN, pwm);
    digitalWrite(MOTOR1A, LOW);
    digitalWrite(MOTOR1B, HIGH);
    }
    if (pwm == 0){
    analogWrite(MOTOR1_EN, pwm);
    digitalWrite(MOTOR1A, LOW);
    digitalWrite(MOTOR1B, LOW);
    }
}

void move_r_motor(int pwm){
    if (pwm > 0){
    analogWrite(MOTOR2_EN, pwm);
    digitalWrite(MOTOR2A, HIGH);
    digitalWrite(MOTOR2B, LOW);
    }
    if (pwm < 0){
    pwm=abs(pwm);
    analogWrite(MOTOR2_EN, pwm);
    digitalWrite(MOTOR2A, LOW);
    digitalWrite(MOTOR2B, HIGH);
    }
    if (pwm == 0){
    analogWrite(MOTOR2_EN, pwm);
    digitalWrite(MOTOR2A, LOW);
    digitalWrite(MOTOR2B, LOW);
    }
}

void isr_left(void)
{
  if (digitalRead(ENCODERA_MOTOR1))
  {
    up_left = digitalRead(ENCODERB_MOTOR1);
  }
  else
  {
    up_left = !digitalRead(ENCODERB_MOTOR1);
  }
  fired_left= true;
  
  if (fired_left)
  {
    if (up_left)
    {
      rotary_encoder_left--;
    }
    else
    {
      rotary_encoder_left++;
    }
  }
}

void isr_right(void)
{
  if (digitalRead(ENCODERA_MOTOR2))
  {
    up_right = digitalRead(ENCODERB_MOTOR2);
  }
  else
  {
    up_right = !digitalRead(ENCODERB_MOTOR2);
  }
  fired_right= true;

  if (fired_right)
  {
    if (up_right)
    {
      rotary_encoder_right++;
    }
    else
    {
      rotary_encoder_right--;
    }
  }
}
