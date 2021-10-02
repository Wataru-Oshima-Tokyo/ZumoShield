#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
//#include <ros/time.h>
//#include <tf/transform_broadcaster.h>
#include <Wire.h>
//#include <LSM303.h>
//#include <L3G.h>
//#include <ZumoShieldEncoders.h>
#include <ZumoShield.h>

#define RIGHT_CNT_FLAG 0x0001
#define LEFT_CNT_FLAG  0x0002


long timer=0;              // Elapsed time since program started (milli second)
//int16_t vright = 0;            // Left Morter velocity (speed of motor)
//int16_t vleft = 0;  // Right Morter velocity (speed of motor)
int16_t previousNum = 0;
int16_t r_cnt = 0;
int16_t l_cnt = 0;
int16_t flag = 0;
int16_t basespeed = 150;       // Base speed of Morter (Effective Range: 1 - 150)
int16_t positionLeft  = 0; // For encoder verification
int16_t positionRight = 0; // For encoder verification
int16_t newLeft = 0;       // Value of Encorder
int16_t newRight = 0;      // Value of Encorder
float ForwardSpeed = 0.0f;
float AngularSpeed = 0.0f;
std_msgs::String str_msg;  // Sensor value to be published
geometry_msgs::Twist cmd_vel; //cmd_vel value
//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;


LSM303 compass;            // Magnetometer
L3G gyro;                  // Gyrometer
ZumoMotors motors;     // Morter
//ZumoShieldEncoders encoders; // Encoder
ros::NodeHandle nh;        // NodeHandler of ROS


float setANGULARSPEED(float angSpeed) {
      if (angSpeed <=1.0 ) {
        angSpeed = 1.0;
      } else if (angSpeed >1.0 && angSpeed <1.5) {
         angSpeed = 1.3;
      } else {
        angSpeed = 1.5;
      }
      return angSpeed;
}
float setPositionRight(float angSpeed, int16_t posR) {
      if (angSpeed <=1.0 ) {
        posR -= 60*angSpeed;
      } else if (angSpeed ==1.3) {
         posR -=110;
      } else {
        posR -=190;
      }
      return posR;
}


void control_Callback(const geometry_msgs::Twist& cmd_vel)
{
    
    //ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x,  cmd_vel.linear.y,  cmd_vel.linear.z);
    //ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
//   ForwardSpeed = cmd_vel.linear.x;
//   AngularSpeed = cmd_vel.angular.z;
//
       ForwardSpeed =  cmd_vel.linear.x;
       AngularSpeed = cmd_vel.angular.z;
//    int16_t moving_time = 500;
//    while (moving_time > 0) 
//    {


      if (AngularSpeed == 0) 
      {
        if(ForwardSpeed == 0) {
          motors.setSpeeds(0, 0);
        }
        else if(ForwardSpeed < 0){
          motors.setSpeeds(basespeed*-1, basespeed*-0.7);
          positionLeft -= 150;
          positionRight -= 150;
          delay(500);i
        }else {
          motors.setSpeeds(basespeed, basespeed*0.7);
          positionLeft += 150;
          positionRight += 150;
          delay(500);
        }
         
         
      } else if (AngularSpeed < 0) 
      { 
        //turn right
         AngularSpeed *=-1;
         AngularSpeed = setANGULARSPEED(AngularSpeed) * basespeed;
         motors.setSpeeds(basespeed*1.2, basespeed*-1);
         delay(500);
         positionLeft += 150;
         positionRight -=150;
         
      }else 
      {
        //turn left
        AngularSpeed = setANGULARSPEED(AngularSpeed) *basespeed*0.9;
        motors.setSpeeds(basespeed*-1.2, basespeed); //to adjust the speed
        delay(500);
        positionRight += 150;
        positionLeft -=150;
      }
      
    motors.setSpeeds(0, 0);
}

//ros::Subscriber<std_msgs::String> sub("/command", control_Callback);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", control_Callback);
ros::Publisher chatter("/sensorval", &str_msg);
//ros::Publisher motor("/encoder", &str_msg);

void setup()
{
//  cli();
  //Serial.begin(9600);     // Debug Print
  // put your setup code here, to run once:
//  pinMode(4, INPUT);
//  pinMode(5, INPUT);
//  pinMode(11, INPUT);
//  pinMode(13, INPUT);
//  attachInterrupt(digitalPinToInterrupt(5),rightEncoder,CHANGE);
//  attachInterrupt(digitalPinToInterrupt(11),leftEncoder,CHANGE);
    /*
   * ICP1はアナログコンパレータと機能を兼用しているので
   * それをDISABLEとする。
   * 「0」でENABLE、「1」でDISABLE
   */
//  ACSR = 0x80;
//  ADCSRB = 0x00;
//  DIDR1 = 0x00;
//  /*
//   * ICP1(インプットキャプチャー)の設定
//   */
//  TCCR1A= 0x00;
//  TCCR1B = 0x41;  // Disable Input Capture Noise Canceler
//                  // Input Capture Edge : RISE
//  TIMSK1 = 0x20;  // Enable Input Capture Interrupt
//  /*
//   * ICP3(インプットキャプチャー)の設定
//   */
//  TCCR3A= 0x00;
//  TCCR3B = 0x41;  // Disable Input Capture Noise Canceler
//                  // Input Capture Edge : RISE
//  TIMSK3 = 0x20;  // Enable Input Capture Interrupt
  Wire.begin();

//  encoders.getCountsAndResetLeft();
//  encoders.getCountsAndResetRight();

  nh.initNode();           // Init ROS Node
  nh.advertise(chatter);// Init ROS Publisher
  nh.subscribe(sub);       // Init ROS Subscriber
  compass.init();          // Init magnetometer
  compass.enableDefault();

  gyro.init();             // Init gyrometer
  gyro.enableDefault();
  Serial.begin(9600);
  sei();
}

void loop()
{
  compass.read();   // Read magnetometer
  gyro.read();      // Read gyrometer
  timer = millis();
//  newLeft = encoders.getCountsAndResetLeft();
//  newRight = encoders.getCountsAndResetRight();
//   if (!(encoders.checkErrorLeft()) && !(encoders.checkErrorRight())) {
//     positionLeft = newLeft;
//     positionRight = newRight;    
//   }
  String s = "";

  s += timer;          // [0]  Elapsed time since program started (milli second)
  s += ',';
  s += positionRight; //r_cnt;   // [9]  Left Morter odometry (Rotation angle of motor)
  s += ',';
  s += positionLeft;//;  // [10] Right Morter odometry (Rotation angle of motor)
  s += ','; 
  s += compass.a.x;    // [1]  Accelerometer.x
  s += ',';
  s += compass.a.y;    // [2]  Accelerometer.y
  s += ',';
  s += compass.a.z;    // [3]  Accelerometer.z
  s += ',';
//  s += compass.m.x;    // [4]  Magnetometer.x
//  s += ',';
//  s += compass.m.y;    // [5]  Magnetometer.y
//  s += ',';
//  s += compass.m.z;    // [6]  Magnetometer.z
//  s += ',';
//  s += vleft;          // [7]  Left Morter velocity (speed of motor)
//  s += ',';
//  s += vright;         // [8]  Right Morter velocity (speed of motor)
//  s += ',';
//  s += gyro.g.x;       // [11] Gyrometer.x
//  s += ',';
//  s += gyro.g.y;       // [12] Gyrometer.y
//  s += ',';
//  s += gyro.g.z;       // [13] Gyrometer.z

  Serial.println(s);  // Debug Print
  
//  t.header.stamp = nh.now();
//  broadcaster.sendTransform(t);

  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(10);
}
