void getEncoderL(void)
{
  //Serial.println("L");
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