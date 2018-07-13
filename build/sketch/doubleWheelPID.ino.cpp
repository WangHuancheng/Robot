#include <Arduino.h>
#line 1 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
#line 1 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
#include<MsTimer2.h>
//编码器
#define ENCODER_R1 3
#define ENCODER_R2 4
#define ENCODER_L1 2
#define ENCODER_L2 5
//驱动信号
#define PWML_R 10 
#define INL_R1 A2
#define INL_R2 A1
#define PWML_L 9
#define INL_L1 A4
#define INL_L2 A3
#define PERIOD 12.0

float targetRv = 0;
float targetLv = 20;

volatile long encoderVal_R = 0;
volatile long encoderVal_L = 0;

volatile int encodertime_L = 0;
volatile int encodertime_R = 0;


volatile float velocityR;
volatile float velocityL;
float ukR = 0;
float ukL = 0;

float ekR1 = 0;//last error
float ekR2 = 0;//last last error
float ekL1 = 0;//last error
float ekL2 = 0;//last last error

   


#line 39 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
void getEncoderR(void);
#line 67 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
void getEncoderL(void);
#line 96 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
int pidControllerR(float targetRv,float currentRv);
#line 132 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
int pidControllerL(float targetLv,float currentLv);
#line 164 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
void control(void);
#line 216 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
void setup();
#line 239 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
void loop();
#line 39 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
void getEncoderR(void)
{
  //Serial.println("in func getEncoderR!");
  encodertime_R++;
   if(digitalRead(ENCODER_R1) == LOW)
  {
    if(digitalRead(ENCODER_R2) == LOW)
    {
      encoderVal_R--;
    }
      else
    {
      encoderVal_R++;
    }
  }
  else
  {
    if(digitalRead(ENCODER_L2) == LOW)
    {
      encoderVal_R++;
    }
      else
    {
      encoderVal_R--;
    }
  }
}

void getEncoderL(void)
{
  //Serial.println("L");
  encodertime_L++;
   if(digitalRead(ENCODER_L1) == LOW)
  {
    if(digitalRead(ENCODER_L2) == LOW)
    {
      encoderVal_L--;
    }
      else
    {
      encoderVal_L++;
    }
  }
  else
  {
    if(digitalRead(ENCODER_L2) == LOW)
    {
      encoderVal_L++;
    }
      else
    {
      encoderVal_L--;
    }
  }
}


int pidControllerR(float targetRv,float currentRv)
{
  
    float u;
    float output;
    float q0,q1,q2;
    float k = 25;
    float ti = 10;//积分时间
    float td = 5;//微分事件
    float ek = targetRv - currentRv;
    //Serial.println(ek);
    
    q0 = k*(1 + PERIOD/ti + td/PERIOD);
    q1 = -k*(1 + 2*td/PERIOD);
    q2 = k*td/PERIOD;
      

    u = q0*ek + q1*ekR1 + q2*ekR2;
    output = ukR+u;
        //Serial.println(output);


    
    if (output > 255)
        output = 255;
    
    if (output < -255)
        output = -255;
    
    ukR = output;
    ekR2 = ekR1;
    ekR1 = ek;
    return (int)output;

}

int pidControllerL(float targetLv,float currentLv)
{
    float u;
    float output;
    float q0,q1,q2;
    float k = 25;
    float ti = 0;//积分时间
    float td = 0;//微分事件
    float ek = targetLv - currentLv;

    
    q0 = k*(1 + PERIOD/ti + td/PERIOD);
    q1 = -k*(1 + 2*td/PERIOD);
    q2 = k*td/PERIOD;
      

    u = q0*ek + q1*ekL1 + q2*ekL2;
    output = ukL+u;
     //Serial.println(output);
       
    if (output > 255)
        output = 255;
    
    if (output < -255)
        output = -255;
    
    ukL = output;
    ekL2 = ekL1;
    ekL1 = ek;
    return (int)output;
}

void control(void)
{
  //测速 PID
  //Serial.print("encodertime_L:");
  //Serial.print(encodertime_L);
  //Serial.print("\tencodertime_R:");
  //Serial.println(encoderVal_L);

  encodertime_L = 0;
  encodertime_R = 0;

  velocityR = (encoderVal_R*2.0)*3.1415*2.0*(1000/PERIOD)/780;
  encoderVal_R = 0;

  velocityL = (encoderVal_L*2.0)*3.1415*2.0*(1000/PERIOD)/780;
  encoderVal_L = 0;
 

  int dutyCycleR = pidControllerR(targetRv,velocityR);
  int dutyCycleL = pidControllerL(targetLv,velocityL);
  //Serial.print("dutyCycle:");
  //Serial.println(dutyCycleR);


  if(dutyCycleR > 0) //control Right wheel
  {
      
      digitalWrite(INL_R1,LOW);
      digitalWrite(INL_R2,HIGH);
      analogWrite(PWML_R,dutyCycleR);
  }
  else
  {
      digitalWrite(INL_R1,HIGH);
      digitalWrite(INL_R2,LOW);
      analogWrite(PWML_R,abs(dutyCycleR));
  }

    if(dutyCycleL > 0) //control left wheel
  {
      
      digitalWrite(INL_L1,HIGH);
      digitalWrite(INL_L2,LOW);
      analogWrite(PWML_L,dutyCycleL);
  }
  else
  {
      digitalWrite(INL_L1,LOW);
      digitalWrite(INL_L2,HIGH);
      analogWrite(PWML_L,abs(dutyCycleL));
  }
}
void setup() 
{
    TCCR1B = TCCR1B & B11111000 | B00000001;
    pinMode(INL_L1,OUTPUT);
    pinMode(INL_L2,OUTPUT);
    pinMode(PWML_L,OUTPUT);
    pinMode(INL_R1,OUTPUT);
    pinMode(INL_R2,OUTPUT);
    pinMode(PWML_R,OUTPUT);

    pinMode(ENCODER_R1,INPUT);
    pinMode(ENCODER_R2,INPUT);
    pinMode(ENCODER_L1,INPUT);
    pinMode(ENCODER_L2,INPUT);
    
    Serial.begin(9600);

    attachInterrupt(ENCODER_R1 - 2,getEncoderR,CHANGE);//
    attachInterrupt(ENCODER_L1 - 2,getEncoderL,CHANGE);//中断通道0对应port 2，1对应port3
    MsTimer2::set(PERIOD,control);
    MsTimer2::start();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  // analogWrite(PWML_B,255);
  //digitalWrite(INLA1,HIGH);
  //digitalWrite(INLA2,LOW);
  //Serial.print("left v: ");
  Serial.println(velocityL);
  //Serial.print(",");
  //Serial.print("right v");
  //Serial.println(velocityR);
  
}

