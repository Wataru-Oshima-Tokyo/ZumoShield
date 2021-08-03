#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <ZumoShieldEncoders.h>
#include <ZumoShield.h>

long timer=0;              // Elapsed time since program started (milli second)
int vright = 0;            // Left Morter velocity (speed of motor)
int vleft = 0;             // Right Morter velocity (speed of motor)
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
ros::Subscriber<std_msgs::String> sub("/cmd_vel", control_Callback);
ros::Publisher chatter("/sensorval", &str_msg);


void setup()
{
  //Serial.begin(9600);     // Debug Print

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
  if (!(encoders.checkErrorLeft()) && !(encoders.checkErrorRight())) {
    positionLeft = newLeft;
    positionRight = newRight;    
  }
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
  s += positionLeft;   // [9]  Left Morter odometry (Rotation angle of motor)
  s += ',';
  s += positionRight;  // [10] Right Morter odometry (Rotation angle of motor)
  s += ',';
  s += gyro.g.x;       // [11] Gyrometer.x
  s += ',';
  s += gyro.g.y;       // [12] Gyrometer.y
  s += ',';
  s += gyro.g.z;       // [13] Gyrometer.z
  
  //Serial.println(s);  // Debug Print

  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
}

