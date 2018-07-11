# 1 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
# 1 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
# 2 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino" 2
//编码器




//驱动信号
# 16 "d:\\code\\Robot\\doubleWheelPID\\doubleWheelPID.ino"
float targetRv = 20;
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




void getEncoderR(void)
{
  //Serial.println("in func getEncoderR!");
  encodertime_R++;
   if(digitalRead(2) == 0x0)
  {
    if(digitalRead(11) == 0x0)
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
    if(digitalRead(12) == 0x0)
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
   if(digitalRead(3) == 0x0)
  {
    if(digitalRead(12) == 0x0)
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
    if(digitalRead(12) == 0x0)
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

    q0 = k*(1 + 12.0/ti + td/12.0);
    q1 = -k*(1 + 2*td/12.0);
    q2 = k*td/12.0;


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
    float k = 100;
    float ti = 10;//积分时间
    float td = 0;//微分事件
    float ek = targetLv - currentLv;


    q0 = k*(1 + 12.0/ti + td/12.0);
    q1 = -k*(1 + 2*td/12.0);
    q2 = k*td/12.0;


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
  Serial.print("encodertime_L:");
  Serial.print(encodertime_L);
  Serial.print("\tencodertime_R:");
  Serial.println(encodertime_R);

  encodertime_L = 0;
  encodertime_R = 0;

  velocityR = (encoderVal_R*2.0)*3.1415*2.0*(1000/12.0)/780;
  encoderVal_R = 0;

  velocityL = (encoderVal_L*2.0)*3.1415*2.0*(1000/12.0)/780;
  encoderVal_L = 0;


  int dutyCycleR = pidControllerR(targetRv,velocityR);
  int dutyCycleL = pidControllerL(targetLv,velocityL);
  //Serial.print("dutyCycle:");
  //Serial.println(dutyCycleR);


  if(dutyCycleR > 0) //control Right wheel
  {

      digitalWrite(A2,0x0);
      digitalWrite(A1,0x1);
      analogWrite(10,dutyCycleR);
  }
  else
  {
      digitalWrite(A2,0x1);
      digitalWrite(A1,0x0);
      analogWrite(10,((dutyCycleR)>0?(dutyCycleR):-(dutyCycleR)));
  }

    if(dutyCycleL > 0) //control left wheel
  {

      digitalWrite(A4,0x1);
      digitalWrite(A5,0x0);
      analogWrite(9,dutyCycleL);
  }
  else
  {
      digitalWrite(A4,0x0);
      digitalWrite(A5,0x1);
      analogWrite(9,((dutyCycleL)>0?(dutyCycleL):-(dutyCycleL)));
  }
}
void setup()
{
    (*(volatile uint8_t *)(0x81)) = (*(volatile uint8_t *)(0x81)) & 248 | 1;
    pinMode(A4,0x1);
    pinMode(A5,0x1);
    pinMode(9,0x1);
    pinMode(A2,0x1);
    pinMode(A1,0x1);
    pinMode(10,0x1);

    pinMode(2,0x0);
    pinMode(11,0x0);
    pinMode(3,0x0);
    pinMode(12,0x0);

    Serial.begin(9600);

    attachInterrupt(2 - 2,getEncoderR,1);//
    attachInterrupt(3 - 2,getEncoderL,1);//中断通道0对应port 2，1对应port3
    MsTimer2::set(12.0,control);
    MsTimer2::start();
}

void loop()
{
  // put your main code here, to run repeatedly:
  // analogWrite(PWML_B,255);
  //digitalWrite(INLA1,HIGH);
  //digitalWrite(INLA2,LOW);
  //Serial.print("left v: ");
  //Serial.print(velocityL);
  //Serial.print(",");
  //Serial.print("right v");
  //Serial.println(velocityR);

}
