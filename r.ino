void getEncoderR(void)
{
  //Serial.println("in func getEncoderR!");
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