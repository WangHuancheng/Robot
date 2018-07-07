#include<MsTimer2.h>
//编码器
#define ENCODER_1A 2
#define ENCODER_1B 3
#define ENCODER_2A
#define ENCODER_2B
//驱动信号
#define PWML_A 10 
#define INLA1 6
#define INLA2 7
#define PWML_B 
#define INLB1
#define INLB2
#define PERIOD 20.0
#define TARGER_V 65
volatile long encoderVal;//ENCODER
volatile float velocity;
int uk = 0;
float ek1 = 0;//last error
float ek2 = 0;//last last error

   
void getEncoder(void)
{
   if(digitalRead(ENCODER_1A) == LOW)
  {
    if(digitalRead(ENCODER_1B) == LOW)
    {
      encoderVal--;
    }
      else
    {
      encoderVal++;
    }
  }
  else
  {
    if(digitalRead(ENCODER_1B) == LOW)
    {
      encoderVal++;
    }
      else
    {
      encoderVal--;
    }
  }
}


int pidController(float targerV,float currentV)
{
    float output;
    float q0,q1,q2;
    float k = 20;
    float ti = 10;//积分时间
    float td = 0;//微分事件
    
    float ek = targerV - currentV;
    
    q0 = k*(1 + PERIOD/ti + td/PERIOD);
    q1 = -k*(1 + 2*td/PERIOD);
    q2 = k*td/PERIOD;

    output = q0*ek + q1*ek1 + q2*ek2;
    ek2 = ek1;
    ek1 = ek;
    return (int)output;

}
void control()
{
  //测速 PID
  //Serial.print("in func control");
  velocity = (encoderVal*1.0)*3.1415*2.0*(1000/PERIOD)/980;
  encoderVal = 0;
  //Serial.print(velocity);
  //Serial.print("\r\n");
  int dutyCycle = pidController(TARGER_V,velocity);
  if(dutyCycle > 0)
  {
      digitalWrite(INLA1,LOW);
      digitalWrite(INLA2,HIGH);
      analogWrite(PWML_A,dutyCycle);
  }
  else
  {
      digitalWrite(INLA1,HIGH);
      digitalWrite(INLA2,LOW);
      analogWrite(PWML_A,abs(dutyCycle));
  }
}
void setup() 
{
    TCCR1B = TCCR1B & B11111000 | B00000001;
    pinMode(INLA1,OUTPUT);
    pinMode(INLA2,OUTPUT);
    pinMode(PWML_A,OUTPUT);
   // pinMode(INLB1,OUTPUT);
   // pinMode(INLB2,OUTPUT);
    //pinMode(PWML_B,OUTPUT);

    pinMode(ENCODER_1A,INPUT);
    pinMode(ENCODER_1B,INPUT);
   // pinMode(ENCODER_2A,INPUT);
    //pinMode(ENCODER_2B,INPUT);
    
    Serial.begin(9600);

    attachInterrupt(0,getEncoder,CHANGE);
    MsTimer2::set(PERIOD,control);
    MsTimer2::start();
}

void loop() 
{
  // put your main code here, to run repeatedly:
//  //analogWrite(PWML_A,255);
//  digitalWrite(INLA1,HIGH);
//  digitalWrite(INLA2,LOW);
  Serial.print(velocity);
  Serial.print("\r\n");
  
}
