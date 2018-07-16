# 1 "d:\\code\\Robot\\past_xunxian\\past_xunxian.ino"
# 1 "d:\\code\\Robot\\past_xunxian\\past_xunxian.ino"

# 3 "d:\\code\\Robot\\past_xunxian\\past_xunxian.ino" 2
//左右电机码盘




//左右电机PWM波以及电机正负极接入







//从前进方向的最左边开始排序红外传感器引脚







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
   if(digitalRead(3) == 0x0)
  {
    if(digitalRead(4) == 0x0)
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
    if(digitalRead(5) == 0x0)
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
   if(digitalRead(2) == 0x0)
  {
    if(digitalRead(5) == 0x0)
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
    if(digitalRead(5) == 0x0)
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
    float k = 20;
    float ti = 80;//积分时间
    float td = 5;//微分事件
    float ek = lTargetRv - currentRv;
    //Serial.println(ek);

    q0 = k*(1 + 20/ti + td/20);
    q1 = -k*(1 + 2*td/20);
    q2 = k*td/20;


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
    float k = 10;
    float ti = 80;//积分时间
    float td = 5;//微分事件
    float ek = lTargetLv - currentLv;


    q0 = k*(1 + 20/ti + td/20);
    q1 = -k*(1 + 2*td/20);
    q2 = k*td/20;


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
  data[0] = digitalRead(A0);
  data[1] = !digitalRead(A5);
  data[2] = !digitalRead(6);
  data[3] = !digitalRead(7);
  data[4] = !digitalRead(8);
  data[5] = !digitalRead(11);
  data[6] = digitalRead(13);

  velocityR = (encoderVal_R*2.0)*3.1415*2.0*(1000/20)/780;
  encoderVal_R = 0;
  velocityL = (encoderVal_L*2.0)*3.1415*2.0*(1000/20)/780;
  encoderVal_L = 0;

  if(!(data[0]||data[1]||data[2]||data[3]||data[4]||data[5]||data[6]))
  {
        targetRv = -4;
        targetLv = -4;
  }

  else if(data[3]&&(data[0]||data[1]||data[2]||data[4]||data[5]||data[6]))
  {
    if(data[0]||data[1]||data[2])
    {
        //dVelocity = 70;
        targetRv = 60;
        targetLv = 0;
    }
    else if (data[4]||data[5]||data[6])
    {
        //dVelocity = -70;
        targetRv = 0;
        targetLv = 60;
    }
  }
  else
  {
    dVelocity = 10*data[0] +8*data[1] + 6*data[2] - 6*data[4] - 8*data[5] - 10*data[6];
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

      digitalWrite(A2,0x0);
      digitalWrite(A1,0x1);
      analogWrite(10,dutyCycleR2);
  }
  else
  {
      digitalWrite(A2,0x1);
      digitalWrite(A1,0x0);
      analogWrite(10,((dutyCycleR2)>0?(dutyCycleR2):-(dutyCycleR2)));
  }

    if(dutyCycleL2 > 0) //control Right wheel
  {

      digitalWrite(A4,0x1);
      digitalWrite(A3,0x0);
      analogWrite(9,dutyCycleL2);
  }
  else
  {
      digitalWrite(A4,0x0);
      digitalWrite(A3,0x1);
      analogWrite(9,((dutyCycleL2)>0?(dutyCycleL2):-(dutyCycleL2)));
  }
}
void setup()
{
    (*(volatile uint8_t *)(0x81)) = (*(volatile uint8_t *)(0x81)) & 248 | 1;
    pinMode(A4,0x1);
    pinMode(A3,0x1);
    pinMode(9,0x1);
    pinMode(A2,0x1);
    pinMode(A1,0x1);
    pinMode(10,0x1);

    pinMode(3,0x0);
    pinMode(4,0x0);
    pinMode(2,0x0);
    pinMode(5,0x0);



    attachInterrupt(3 - 2,getEncoderR,2);
    attachInterrupt(2 - 2,getEncoderL,2);
      //寻迹模块D0引脚初始化
    pinMode(A0, 0x0);
    pinMode(A5, 0x0);
    pinMode(6, 0x0);
    pinMode(7, 0x0);
    pinMode(8, 0x0);
    pinMode(11, 0x0);
    pinMode(13, 0x0);
    MsTimer2::set(20,control);
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
