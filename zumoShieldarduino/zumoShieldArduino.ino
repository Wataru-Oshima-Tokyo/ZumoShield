#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <ZumoShieldEncoders.h>
#include <ZumoShield.h>

#define RIGHT_CNT_FLAG 0x0001
#define LEFT_CNT_FLAG  0x0002


long timer=0;              // Elapsed time since program started (milli second)
int vright = 0;            // Left Morter velocity (speed of motor)
int vleft = 0;  // Right Morter velocity (speed of motor)
int r_cnt = 0;
int l_cnt = 0;
int flag = 0;
int basespeed = 150;       // Base speed of Morter (Effective Range: 1 - 150)
int16_t positionLeft  = 0; // For encoder verification
int16_t positionRight = 0; // For encoder verification
int16_t newLeft = 0;       // Value of Encorder
int16_t newRight = 0;      // Value of Encorder
float forwardSpeed = 0.0f;
float rotateSpeed = 0.0f;
std_msgs::String str_msg;  // Sensor value to be published
geometry_msgs::Twist cmd_vel; //cmd_vel value
  
LSM303 compass;            // Magnetometer
L3G gyro;                  // Gyrometer
ZumoMotors motors;     // Morter
ZumoShieldEncoders encoders; // Encoder
ros::NodeHandle nh;        // NodeHandler of ROS

void control_Callback(const geometry_msgs::Twist& cmd_vel)
{
  
    //ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x,  cmd_vel.linear.y,  cmd_vel.linear.z);
    //ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
   forwardSpeed = cmd_vel.linear.x;
   rotateSpeed = cmd_vel.angular.z;
  
    motors.setSpeeds(0, 0);
}
void motorcontrol(const std_msgs::String& cmd_msg)
{

  // I : forward    ,  , : backward
  // L : turn right ,  J : turn left
  // S : stop

  String cmd = "";
  
  //Serial.println(cmd_msg.data);  // Debug Print

  if (strlen(cmd_msg.data) != 0)
  {
    cmd = cmd_msg.data;
  }

  if (cmd == "i")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    vleft = basespeed;
    vright = basespeed;
    motors.setSpeeds(vleft, vright);
  }
  else if (cmd == ",")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    vleft = -1*basespeed;
    vright = -1*basespeed;
    motors.setSpeeds(vleft, vright);
  }
  else if (cmd == "l")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    vleft = (basespeed+220);
    vright = -1*(basespeed+220);
    motors.setSpeeds(vleft, vright);
  }
  else if (cmd == "j")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    vleft = -1*(basespeed+220);
    vright = (basespeed+220);
    motors.setSpeeds(vleft, vright);
  }
  else if (cmd == "s")
  {
    vleft = 0;
    vright = 0;
    motors.setSpeeds(vleft, vright);
    delay(2);
  }

}

//ros::Subscriber<std_msgs::String> sub("/command", motorcontrol);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", control_Callback);
ros::Publisher chatter("/sensorval", &str_msg);


void setup()
{
  //Serial.begin(9600);     // Debug Print
  // put your setup code here, to run once:
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(11, INPUT);
  pinMode(13, INPUT);
  
    /*
   * ICP1はアナログコンパレータと機能を兼用しているので
   * それをDISABLEとする。
   * 「0」でENABLE、「1」でDISABLE
   */
  ACSR = 0x80;
  ADCSRB = 0x00;
  DIDR1 = 0x00;
  /*
   * ICP1(インプットキャプチャー)の設定
   */
  TCCR1A= 0x00;
  TCCR1B = 0x41;  // Disable Input Capture Noise Canceler
                  // Input Capture Edge : RISE
  TIMSK1 = 0x20;  // Enable Input Capture Interrupt
  /*
   * ICP3(インプットキャプチャー)の設定
   */
  TCCR3A= 0x00;
  TCCR3B = 0x41;  // Disable Input Capture Noise Canceler
                  // Input Capture Edge : RISE
  TIMSK3 = 0x20;  // Enable Input Capture Interrupt
  
  Wire.begin();

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  nh.initNode();           // Init ROS Node
  nh.advertise(chatter);   // Init ROS Publisher
  nh.subscribe(sub);       // Init ROS Subscriber

  compass.init();          // Init magnetometer
  compass.enableDefault();

  gyro.init();             // Init gyrometer
  gyro.enableDefault();
}

void loop()
{
  compass.read();   // Read magnetometer
  gyro.read();      // Read gyrometer
  timer = millis();
  newLeft = encoders.getCountsAndResetLeft();
  newRight = encoders.getCountsAndResetRight();
//   if (!(encoders.checkErrorLeft()) && !(encoders.checkErrorRight())) {
//     positionLeft = newLeft;
//     positionRight = newRight;    
//   }
  String s = "";
  s += timer;          // [0]  Elapsed time since program started (milli second)
  s += ',';
  s += compass.a.x;    // [1]  Accelerometer.x
  s += ',';
  s += compass.a.y;    // [2]  Accelerometer.y
  s += ',';
  s += compass.a.z;    // [3]  Accelerometer.z
  s += ',';
  s += compass.m.x;    // [4]  Magnetometer.x
  s += ',';
  s += compass.m.y;    // [5]  Magnetometer.y
  s += ',';
  s += compass.m.z;    // [6]  Magnetometer.z
  s += ',';
  s += vleft;          // [7]  Left Morter velocity (speed of motor)
  s += ',';
  s += vright;         // [8]  Right Morter velocity (speed of motor)
  s += ',';
  s += l_cnt;   // [9]  Left Morter odometry (Rotation angle of motor)
  s += ',';
  s += r_cnt;  // [10] Right Morter odometry (Rotation angle of motor)
  s += ',';
  s += gyro.g.x;       // [11] Gyrometer.x
  s += ',';
  s += gyro.g.y;       // [12] Gyrometer.y
  s += ',';
  s += gyro.g.z;       // [13] Gyrometer.z
  
  Serial.println(s);  // Debug Print

  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
}

ISR(TIMER1_CAPT_vect)
{ 
  bit_is_clear(TIFR1, ICF1);
  if (digitalRead(5) == HIGH) {
    r_cnt++;
  } else {
    r_cnt--;
  }
  flag |= RIGHT_CNT_FLAG;
}

ISR(TIMER3_CAPT_vect)
{ 
  bit_is_clear(TIFR3, ICF3);
  if (digitalRead(11) == LOW) {
    l_cnt++;
  } else {
    l_cnt--;
  }
  flag |= LEFT_CNT_FLAG;
}
