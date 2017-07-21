

/*
##############################################################################################
"ExploDroid" Arduino Code
 Version 1.0

This Code is Copyrighted under the MLAB-Act-2017

PIN CONNECTIONS
 *  Motor 1- >LEFT MOTOR
 *           EncoderA,EncoderB-2,3
 *  Motor 2- >RIGHT MOTOR
 *           EncoderA,EncoderB-19,18
 *  IMU    - Software I2C bus
 *           SCL-50,SDA-51,INT-2 
 * MOTOR CONTROL PINS - LEFT MOTOR 
 *                    - 22,23 and 7 as PWM pin
 *                    - RIGHT MOTOR
 *                    - 24,25 and 8 as PWM Pin
 * Ultrasonic Sensor - Is optional, Not Used in version-1
 *                   Echo - 28 
 *                   Trig - 29
##############################################################################################
*/

#include <Messenger.h>
#include <limits.h>


Messenger Messenger_Handler = Messenger(); //Serial Comm Object

// Variables for Encoders
  long Left_ticks,Right_ticks;
  int lprev=0,rprev=0;
  int rmul=0,lmul=0;


//Time Update Variables
  unsigned long LastUpdateMicrosecs = 0;    
  unsigned long LastUpdateMillisecs = 0;
  unsigned long CurrentMicrosecs = 0;
  unsigned long MicrosecsSinceLastUpdate = 0;
  float SecondsSinceLastUpdate = 0; 
//Motor_encoder pins definitions
  int leftEncoderA=2;
  int leftEncoderB=3;
  int rightEncoderA=19;
  int rightEncoderB=18 ; 
//Motor Speed Variables
  float motor_left_speed = 0;
  float motor_right_speed= 0;
  int stopvariable =1;
//Motor_Pins Definition
  #define A_1 22
  #define B_1 23
  //PWM 1 pin number
  #define PWM_1 7
  //Right Motor
  #define A_2 24 
  #define B_2 25
  //PWM 2 pin number
  #define PWM_2 8
//Ultrasonic pins definition
  const int echo = 28, Trig = 29;
  long duration, cm;

void LEFT_A_CHANGE(){
  if( digitalRead(leftEncoderB) == 0 ) {
    if ( digitalRead(leftEncoderA) == 0 ) {
      // A fell, B is low
      Left_ticks--; // moving reverse
    } else {
      // A rose, B is low
      Left_ticks++; // moving forward
    }
  }else {
    if ( digitalRead(leftEncoderA) == 0 ) {
      // B fell, A is high
      Left_ticks++; // moving reverse
    } else {
      // B rose, A is high
      Left_ticks--; // moving forward
    }
  }
}

void LEFT_B_CHANGE(){
  if ( digitalRead(leftEncoderA) == 0 ) {
   if ( digitalRead(leftEncoderB) == 0 ) {
      // B fell, A is low
      Left_ticks++; // moving forward
    } else 
      // B rose, A is low
      Left_ticks--; // moving reverse
    }
  else {
    if ( digitalRead(leftEncoderB) == 0 ) {
      // B fell, A is high
      Left_ticks--; // moving reverse
    } else {
      // B rose, A is high
      Left_ticks++; // moving forward
    }
  }
}

void RIGHT_A_CHANGE(){
  if( digitalRead(rightEncoderB) == 0 ) {
    if ( digitalRead(rightEncoderA) == 0 ) {
      // A fell, B is low
      Right_ticks--; // moving reverse
    } else {
      // A rose, B is low
      Right_ticks++; // moving forward
    }
  }else {
    if ( digitalRead(rightEncoderA) == 0 ) {
      // B fell, A is high
      Right_ticks++; // moving reverse
    } else {
      // B rose, A is high
      Right_ticks--; // moving forward
    }
  }
}

void RIGHT_B_CHANGE(){
  if ( digitalRead(rightEncoderA) == 0 ) {
   if ( digitalRead(rightEncoderB) == 0 ) {
      // B fell, A is low
      Right_ticks++; // moving forward
    } else {
      // B rose, A is low
      Right_ticks--; // moving reverse
    }
 } else {
    if ( digitalRead(rightEncoderB) == 0 ) {
      // B fell, A is high
      Right_ticks--; // moving reverse
    } else {
      // B rose, A is high
      Right_ticks++; // moving forward
    }
  }
}

  
void SetupEncoders()
{
  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(leftEncoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), LEFT_A_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), LEFT_B_CHANGE, CHANGE);
  pinMode(rightEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), RIGHT_A_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), RIGHT_B_CHANGE, CHANGE);
}

void SetupMotors()
{
  //Left motor
 pinMode(A_1,OUTPUT);
 pinMode(B_1,OUTPUT); 
 pinMode(PWM_1,OUTPUT);
 //Right Motor
 pinMode(A_2,OUTPUT);
 pinMode(B_2,OUTPUT);  
 pinMode(PWM_2,OUTPUT); 
 digitalWrite(A_1,HIGH);
 digitalWrite(B_1,HIGH);
 digitalWrite(A_2,HIGH);
 digitalWrite(B_2,HIGH);
 analogWrite(PWM_1,170);
 analogWrite(PWM_2,170);
}

void SetupUltrasonic()
{
 pinMode(Trig, OUTPUT);
 pinMode(echo, INPUT); 
}

void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 SetupEncoders();
 SetupMotors();
 SetupUltrasonic();
 Messenger_Handler.attach(OnMssageCompleted);
}

void Read_From_Serial()
{
   while(Serial.available() > 0)
    {
       int data = Serial.read();
       Messenger_Handler.process(data); 
    }   
}

void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  if(Messenger_Handler.checkString(reset))
  {
     Serial.println("Reset Done"); 
  }
  if(Messenger_Handler.checkString(set_speed))
  { 
    //This will set the speed
     Set_Speed();
     return;   
  }
}

void Set_Speed()
{  
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong(); 
}

void UpdateEncoders()
{
 //Send it via Serial
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_ticks);
  Serial.print("\t");
  Serial.print(Right_ticks);
  Serial.print("\n");
}

void Update_Time()
{
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
  {
     MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
  }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;
  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");  
}

void Update_Ultra_Sonic()
{
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echo, HIGH);
  // convert the time into a distance
  cm = duration / 29 / 2;
  //Sending through serial port
  Serial.print("u");
  Serial.print("\t");
  Serial.print(cm);
  Serial.print("\n"); 
}

void Update_Motors()
{
  //Move the Motors according to the speed
  Move_Motors(motor_left_speed,motor_right_speed);
  delayMicroseconds(100);
  //Update the values
  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);  
  Serial.print("\n");
}

void Move_Motors(float leftServoValue,float rightServoValue)
{
  if((leftServoValue > 0))
  { 
    if(rightServoValue >0 )
    {
      digitalWrite(A_1,LOW);
      digitalWrite(B_1,HIGH);
      digitalWrite(A_2,HIGH);
      digitalWrite(B_2,LOW);
    }
    if(rightServoValue<0)
    {
      
      digitalWrite(A_1,LOW);
      digitalWrite(B_1,HIGH);
      digitalWrite(A_2,LOW);
      digitalWrite(B_2,LOW);
      }   
  }
  
  else if((leftServoValue < 0) )
  {
    if(rightServoValue < 0 )
    {
     digitalWrite(A_1,HIGH);
     digitalWrite(B_1,LOW);
     digitalWrite(A_2,LOW);
     digitalWrite(B_2,HIGH);
    }
    if(rightServoValue > 0)
    {
       
      digitalWrite(A_1,LOW);
      digitalWrite(B_1,LOW);
      digitalWrite(A_2,HIGH);
      digitalWrite(B_2,LOW);
    }
  } 
    else 
    {
           digitalWrite(A_1,HIGH);
           digitalWrite(B_1,HIGH);
           digitalWrite(A_2,HIGH);
           digitalWrite(B_2,HIGH);
     }  
}

void loop() {
  // put your main code here, to run repeatedly:
   Read_From_Serial();
  UpdateEncoders();
  //Update_Ultra_Sonic();
  Update_Motors();
  Update_Time();
}
