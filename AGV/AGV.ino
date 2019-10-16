#include <Cytron_SmartDriveDuo.h>
#include <ros.h>
#include <std_msgs/Float32.h>


#define IN1 32 // Arduino pin 32 is connected to MDDS30 pin IN1.  IN1 is Digital Signal(Direction) for LEFT motor
#define AN1 12 // Arduino pin 12 is connected to MDDS30 pin AN1.  AN1 - Analog/PWM signal for motor LEFT.
#define AN2 11 // Arduino pin 13 is connected to MDDS30 pin AN2.  AN2 - Analog/PWM signal for motor RIGHT.
#define IN2 33 // Arduino pin 33 is connected to MDDS30 pin IN2.  IN2 is Digital Signal(Direction) for RIGHT motor


//GLOBAL VARIABLES
int left_encoderA = 2;
int left_encoderB = 3;
int right_encoderA = 20;
int right_encoderB = 21;
unsigned long start_time;// time_difference;
int flag = 1;
volatile int left_lastEncoded = 0;
volatile long left_encoderValue = 0.0f;
volatile int right_lastEncoded = 0;
volatile long right_encoderValue = 0.0f;
long left_lastencoderValue = 0;
long right_lastencoderValue = 0;
char inChar;
signed int speedLeft, speedRight;
signed int counter=0;
ros::NodeHandle nh;
std_msgs::Float32 left_encoder_msg;
std_msgs::Float32 right_encoder_msg;
std_msgs::Float32 left_encoder_msg_rate;
std_msgs::Float32 right_encoder_msg_rate;
std_msgs::Float32 time_diff;
//std_msgs::Float32 rate_left_encoder_msg;
//std_msgs::Float32 rate_right_encoder_msg;


Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, IN1, IN2, AN1, AN2);


float target_left_velocity;
float target_right_velocity;
float left_velocity;
float right_velocity;



void rightWheelCb(const std_msgs::Float32 &wheel_power){
  //speedRight = 100*max(min(wheel_power.data,1.0f),-1.0f);
  //smartDriveDuo30.control(speedLeft, speedRight);
  //turnWheel(wheel_power,AN2,IN2);
  target_right_velocity = wheel_power.data;
  
}

void leftWheelCb(const std_msgs::Float32 &wheel_power){
  //speedLeft = 100*max(min(wheel_power.data,1.0f),-1.0f);
  //smartDriveDuo30.control(speedLeft, speedRight);
  //turnWheel(wheel_power,AN1,IN1);
  target_left_velocity = wheel_power.data;
}

void update_left_Encoder(){
  int MSB = digitalRead(left_encoderA); //MSB = most significant bit
  int LSB = digitalRead(left_encoderB); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (left_lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) left_encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) left_encoderValue --;

  left_lastEncoded = encoded; //store this value for next time
}



void update_right_Encoder(){
  int MSB = digitalRead(right_encoderA); //MSB = most significant bit
  int LSB = digitalRead(right_encoderB); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (right_lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) right_encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) right_encoderValue --;

  right_lastEncoded = encoded; //store this value for next time
}


ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right", &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left", &leftWheelCb );
ros::Publisher pub_left("left_wheel_encoder", &left_encoder_msg);
ros::Publisher pub_right("right_wheel_encoder", &right_encoder_msg);

ros::Publisher pub_left_rate("left_wheel_encoder_rate", &left_encoder_msg);
ros::Publisher pub_right_rate("right_wheel_encoder_rate", &right_encoder_msg);



float actual_left_speed,actual_right_speed,req_left_speed,req_right_speed,left_speed_error,right_speed_error;
float prev_left_speed_error, sum_left_speed_error, prev_right_speed_error, sum_right_speed_error;
float Kp = 0.06; //0.05;
float Kd = 0.047; //0.032; //0.5 * Kp;
float Ki = 0.0; //0.016; //0.5 * Kd;


void setup()
{
  nh.initNode();
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.advertise(pub_left);
  nh.advertise(pub_right);
  nh.advertise(pub_left_rate);
  nh.advertise(pub_right_rate);
  
  Serial.begin(57600);
  smartDriveDuo30.control(0, 0);
  pinMode(left_encoderA, INPUT); 
  pinMode(left_encoderB, INPUT);
  pinMode(right_encoderA, INPUT); 
  pinMode(right_encoderB, INPUT);
  

  digitalWrite(left_encoderA, HIGH); //turn pullup resistor on
  digitalWrite(left_encoderB, HIGH); //turn pullup resistor on
  digitalWrite(right_encoderA, HIGH); //turn pullup resistor on
  digitalWrite(right_encoderB, HIGH); //turn pullup resistor on
  

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, update_left_Encoder, CHANGE); 
  attachInterrupt(1, update_left_Encoder, CHANGE);
  attachInterrupt(2, update_right_Encoder, CHANGE); 
  attachInterrupt(3, update_right_Encoder, CHANGE);
  
  //pinMode(HALLSEN_A, INPUT_PULLUP); // Set hall sensor A as input pullup
}

void loop()
{
  if((millis()-start_time)>=200){
    start_time = millis();
    
    left_encoder_msg.data = left_encoderValue;
    right_encoder_msg.data = right_encoderValue;
    
    actual_left_speed = 0.0060877*(left_encoderValue - left_lastencoderValue);   
    left_lastencoderValue = left_encoderValue;
    
    actual_right_speed = 0.0060877*(right_encoderValue - right_lastencoderValue);   
    right_lastencoderValue = right_encoderValue;
    
     
    if(target_left_velocity>-0.01 && target_left_velocity<0.01)
        left_velocity = 0;
    else{
      //e1_error = left_velocity - actual_speed;
      left_speed_error = target_left_velocity - actual_left_speed;
      left_velocity = left_velocity + left_speed_error*Kp + prev_left_speed_error*Kd + sum_left_speed_error*Ki;
      prev_left_speed_error = left_speed_error;
      sum_left_speed_error += left_speed_error;
    }

    if(target_right_velocity>-0.01 && target_right_velocity<0.01)
        right_velocity = 0;
    else{
      //e1_error = left_velocity - actual_speed;
      right_speed_error = target_right_velocity - actual_right_speed;
      right_velocity = right_velocity + right_speed_error*Kp + prev_right_speed_error*Kd + sum_right_speed_error*Ki;
      prev_right_speed_error = right_speed_error;
      sum_right_speed_error += right_speed_error;
    }
/*    
    left_speed_error = target_left_velocity - actual_left_speed;
    right_speed_error = target_right_velocity - actual_right_speed;

    left_velocity = left_velocity + left_speed_error*Kp + prev_left_speed_error*Kd + sum_left_speed_error*Ki;
    prev_left_speed_error = left_speed_error;
    sum_left_speed_error += left_speed_error;

    right_velocity = right_velocity + right_speed_error*Kp + prev_right_speed_error*Kd + sum_right_speed_error*Ki;
    prev_right_speed_error = right_speed_error;
    sum_right_speed_error += right_speed_error;
*/      
    left_encoder_msg_rate.data = actual_left_speed;  
    right_encoder_msg_rate.data = actual_right_speed;


    
    //actual_speed = 0.0060877*(abs(left_encoderValue) - abs(left_lastencoderValue));// /980)*0.180*3.14;
    //left_encoder_msg_rate.data = actual_speed;
    
    
    /*
    e1_error = actual_speed - left_velocity;
    left_velocity= speedLeft+e1_error*Kp;
    left_velocity = max(min(left_velocity,1.0f),-1.0f);
    */

    if(left_velocity<-1.0){
      left_velocity = -1.0;
      speedLeft = -100;
    }
    else if(left_velocity>1.0){
      left_velocity = 1.0;
      speedLeft = 100;
    }
    else
      speedLeft = left_velocity*100;

    if(right_velocity<-1.0){
      right_velocity = -1.0;
      speedRight = -100;
    }
    else if(right_velocity>1.0){
      right_velocity = 1.0;
      speedRight = 100;
    }
    else
      speedRight = right_velocity*100;
     
    //left_velocity = max(min(left_velocity,1.0),-1.0);
    //right_velocity = max(min(right_velocity,1.0),-1.0);
    //speedLeft = left_velocity*100;
    //speedRight = right_velocity*100;
    
    smartDriveDuo30.control(30, 30);
    pub_left.publish( &left_encoder_msg );
    pub_right.publish( &right_encoder_msg );
    pub_left_rate.publish(&left_encoder_msg_rate);   
    pub_right_rate.publish(&right_encoder_msg_rate);   
     
  }
  nh.spinOnce();   
}
