#include <MsTimer2.h> 
#define PWML 9
#define PWMR 10
#define IN1 11
#define IN2 12
#define IN3 8
#define IN4 8                             
volatile long encoderValL;
volatile long encoderValR;  
float velocityL;            
float velocityR;
float Kp_Velocity = 5;
float Ti_Velocity = 100;
float Td_Velocity = 1;
float kpL= 0;     
float kppL = 0;     
float PoutputL = 0;       
float kpR= 0;     
float kppR = 0;     
float PoutputR = 0;       
#define MID 4
float Kp_Route = 1.0;
//float Ki_Route = 90;
float Kd_Route = 0;
float error_midValLast1 = 0;
float midVal;
#define Red1  A5
#define Red2  A4
#define Red3  A3
#define Red4  A2
#define Red5  A1
#define Red6  A0
#define Red7  6
int _redVal[7];
float midLast = 4;
#define ENCODER_A 2
#define ENCODER_B 4
#define ENCODER_C 3
#define ENCODER_D  5
#define PERIOD 10    //定义测速周期
#define TARGET 3     //定义目标速度值
void getEncoderL(void)
{
  if (digitalRead(ENCODER_A) == LOW) //如果是下降沿触发的中断
  {
    if (digitalRead(ENCODER_B) == LOW)
    {
      encoderValL++; //根据另外一相电平判定方向
    }
    else
    {
      encoderValL--;
    }
  }
  else
  {
    if (digitalRead(ENCODER_B) == LOW)
    {
      encoderValL--;
    }
    else
    {
      encoderValL++;
    }
  }
}

void getEncoderR(void)
{
  if (digitalRead(ENCODER_C) == LOW) //如果是下降沿触发的中断
  {
    if (digitalRead(ENCODER_D) == LOW)
    {
      encoderValR--; //根据另外一相电平判定方向
    }
    else
    {
      encoderValR++;
    }
  }
  else
  {
    if (digitalRead(ENCODER_D) == LOW)
    {
      encoderValR++;
    }
    else
    {
      encoderValR--;
    }
  }
}

//PID速度环控制参数
int pidControllerL(float targetVelocity, float currentVelocity)
{
  float output;
  float u;
  float d;
  u= targetVelocity - currentVelocity;
  d = u* Kp_Velocity * (1 + PERIOD / Ti_Velocity + Td_Velocity / PERIOD) - Kp_Velocity * (1 + 2 * Td_Velocity / PERIOD) * kpL+ (Kp_Velocity * Td_Velocity / PERIOD) * kppL;
  kppL=kpL;
  kpL=u;
  output =PoutputL + d;
  if (output > 255)
    output = 255;
  if (output < -255)
    output = -255;
  PoutputL= output;
  return (int)output;
}

int pidControllerR(float targetVelocity, float currentVelocity)
{
  float output;
  float u;
  float d;
  u= targetVelocity - currentVelocity;
  d = u* Kp_Velocity * (1 + PERIOD / Ti_Velocity + Td_Velocity / PERIOD) - Kp_Velocity * (1 + 2 * Td_Velocity / PERIOD) * kpL+ (Kp_Velocity * Td_Velocity / PERIOD) *kppL;
  kppL=kpL;
kpL=u;
  output = PoutputR+ d;
  if (output > 255)
    output = 255;
  if (output < -255)
    output = -255;
  PoutputR= output;
  return (int)output;
}


void control(void)
{ 
  float TARGET_L=TARGET;
  float TARGET_R=TARGET;
  velocityL = (encoderValL / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  velocityR = (encoderValR / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderValL=0;
  encoderValR=0;
  
  Serial.println(velocityL);
  Serial.println(velocityR);
  float d = pidRoute();
  //Serial.println(d);
 TARGET_L -= d;
 TARGET_R += d;
int outputL = pidControllerL(TARGET_L, velocityL);
  int outputR = pidControllerL(TARGET_R, velocityR);
  if (outputL > 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWML, outputL);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWML, abs(outputL));
  }

 if (outputR > 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(PWMR, outputR);
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWMR, abs(outputR));
  }
}
float pidRoute()
{
  float u=  MID - midVal;
  float d =u * Kp_Route + (u- error_midValLast1) * Kd_Route; //PD
  error_midValLast1 =u;
  return d;
}

#define pin_V A5
int n = 0;
float sum = 0;

void readVault(void)
{
  n++;
  float dataV = analogRead(pin_V);
  sum += dataV;
  if ( n > 200)
  {
    float vault = (sum / n) * 0.063;
    Serial.print("电池电压：");
    Serial.println(vault);
    sum = 0;
    n = 0;
  }
  if (dataV < 11)
    Serial.println("电量过低！");
}
void infraredInit()
{
  pinMode(Red1, INPUT);
  pinMode(Red2, INPUT);
  pinMode(Red3, INPUT);
  pinMode(Red4, INPUT);
  pinMode(Red5, INPUT);
  pinMode(Red6, INPUT);
  pinMode(Red7, INPUT);
}

void readInfrared()
{

  _redVal[0] = digitalRead(Red1);
  _redVal[1] = digitalRead(Red2);
  _redVal[2] = digitalRead(Red3);
  _redVal[3] = digitalRead(Red4);
  _redVal[4] = digitalRead(Red5);
  _redVal[5] = digitalRead(Red6);
  _redVal[6] = digitalRead(Red7);
}

float infraredFindMidVal()
{
  float mid;
  int sum = 0;
  int n = 0;
  readInfrared();
  for (int i = 0; i < 7; i++)
  {
    if (_redVal[i] == 0)
    {
      sum += (i + 1);
      n++;
    }
  }
  if (sum == 0)
    mid = midLast;
  else
    mid = (float)sum / n;
  midLast = mid;
  return mid;
}

void encoderInit()
{
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(ENCODER_C, INPUT);
  pinMode(ENCODER_D, INPUT);
}

void setup()
{
  infraredInit();
  encoderInit();
  TCCR1B = TCCR1B & B11111000 | B00000001;

  //pinMode(pin_V, INPUT);
  pinMode(PWML, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //使编码器引脚外部中断
  attachInterrupt(0, getEncoderL, CHANGE);
  attachInterrupt(1, getEncoderR, CHANGE);
  MsTimer2::set(PERIOD, control);
  MsTimer2::start();
  Serial.begin(9600);
}

void loop()
{
  midVal = infraredFindMidVal();
  //Serial.println(midVal);
   //readVault();
  delay(5);
  
}
