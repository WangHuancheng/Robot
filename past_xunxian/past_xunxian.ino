
#include<MsTimer2.h>
//左右电机码盘
#define ENCODER_R1 3
#define ENCODER_R2 4
#define ENCODER_L1 2
#define ENCODER_L2 5
//左右电机PWM波以及电机正负极接入
#define PWML_R 10 
#define INL_R1 A2
#define INL_R2 A1
#define PWML_L 9
#define INL_L1 A4
#define INL_L2 A3
#define PERIOD 10
//从前进方向的最左边开始排序红外传感器引脚
#define trac1  A0 
#define trac2  A5 
#define trac3  6 
#define trac4  7 
#define trac5  8 
#define trac6  11 
#define trac7  13 
const float originTargetV = 5;
volatile float targetRv = originTargetV;//右轮目标速度
volatile float targetLv = originTargetV;//左轮目标速度

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


int pidControllerR(float lTargetRv,float currentRv)
{
  
    float u;
    float output;
    float q0,q1,q2;
    float k  = 20;
    float ti = 80;//积分时间
    float td = 5;//微分事件
    float ek = lTargetRv - currentRv;
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

int pidControllerL(float lTargetLv,float currentLv)
{
    float u;
    float output;
    float q0,q1,q2;
    float k  = 10;
    float ti = 80;//积分时间
    float td = 5;//微分事件
    float ek = lTargetLv - currentLv;

    
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
  int data[7];
  float dVelocity = 0;
  data[0] = !digitalRead(A0);
  data[1] = !digitalRead(A5);
  data[2] = !digitalRead(6);
  data[3] = !digitalRead(7);
  data[4] = !digitalRead(8);
  data[5] = !digitalRead(11);
  data[6] = !digitalRead(13);

  velocityR = (encoderVal_R*2.0)*3.1415*2.0*(1000/PERIOD)/780;
  encoderVal_R = 0;
  velocityL = (encoderVal_L*2.0)*3.1415*2.0*(1000/PERIOD)/780;
  encoderVal_L = 0;

  if(!(data[1]||data[2]||data[3]||data[4]||data[5]))
  {
        targetRv = -10;
        targetLv = -10;
  }

  else if(data[3]&&(data[1]||data[5]))
  {
    if(data[1])
    {
        //dVelocity = 70;
        targetRv = 30;
        targetLv = -30;
    }
    else if (data[5])
    {
        //dVelocity = -70;
        targetRv = -35;
        targetLv = 35;
    }
  }
  else
  {
    dVelocity = 0*data[0] +10* data[1] + 6*data[2] - 6*data[4] - 10*data[5] - 0*data[6];
    targetRv += 0.5*dVelocity;
    targetLv -= 0.5*dVelocity;
  }
  //dVelocity = 30*data[0] +10* data[1] + 6*data[2] - 6*data[4] - 10*data[5] - 30*data[6];


  int dutyCycleR2 = pidControllerR(targetRv,velocityR);
  int dutyCycleL2 = pidControllerL(targetLv,velocityL);
 
  targetRv = originTargetV;
  targetLv = originTargetV;

  //int dutyCycleL2 = dutyCycleL1 - D_value / 2;
  //int dutyCycleR2 = dutyCycleR1 + D_value / 2;
  if(dutyCycleR2 > 0) //control Right wheel
  {
      
      digitalWrite(INL_R1,LOW);
      digitalWrite(INL_R2,HIGH);
      analogWrite(PWML_R,dutyCycleR2);
  }
  else
  {
      digitalWrite(INL_R1,HIGH);
      digitalWrite(INL_R2,LOW);
      analogWrite(PWML_R,abs(dutyCycleR2));
  }

    if(dutyCycleL2 > 0) //control Right wheel
  {
      
      digitalWrite(INL_L1,HIGH);
      digitalWrite(INL_L2,LOW);
      analogWrite(PWML_L,dutyCycleL2);
  }
  else
  {
      digitalWrite(INL_L1,LOW);
      digitalWrite(INL_L2,HIGH);
      analogWrite(PWML_L,abs(dutyCycleL2));
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
    
    

    attachInterrupt(ENCODER_R1 - 2,getEncoderR,FALLING);
    attachInterrupt(ENCODER_L1 - 2,getEncoderL,FALLING);
      //寻迹模块D0引脚初始化
    pinMode(trac1, INPUT);
    pinMode(trac2, INPUT);
    pinMode(trac3, INPUT);
    pinMode(trac4, INPUT);
    pinMode(trac5, INPUT);
    pinMode(trac6, INPUT);
    pinMode(trac7, INPUT);
    MsTimer2::set(PERIOD,control);
    MsTimer2::start();
    Serial.begin(9600);
    
}

void loop() 
{

  Serial.print(velocityL);
  Serial.print(",");
  Serial.println(velocityR);
  //Serial.println(encodertime_L);
  //Serial.print("Right");
  //Serial.println(encodertime_R);

}
