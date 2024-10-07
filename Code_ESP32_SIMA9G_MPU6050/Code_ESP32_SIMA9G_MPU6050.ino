#include "config.h"
int cmode=0;
String my_phone = "0359637036";
int ckl=0;

void setup() {
  pininit();
  mpu_init();
}

void loop() {
  if (digitalRead(sw1)==0)
  {
    delay(30);
    if (digitalRead(sw1)==0)
    {
      while (digitalRead(sw1)==0);
      digitalWrite(led1,0);
      digitalWrite(led2,1);
      digitalWrite(led3,1);
      cmode=1;
    }
  }
  if (digitalRead(sw2)==0)
  {
    delay(30);
    if (digitalRead(sw2)==0)
    {
      while (digitalRead(sw2)==0);
      digitalWrite(led1,1);
      digitalWrite(led2,0);
      digitalWrite(led3,1);
      cmode=2;
    }
  }
  if (digitalRead(sw3)==0)
  {
    delay(30);
    if (digitalRead(sw3)==0)
    {
      while (digitalRead(sw3)==0);
      digitalWrite(led1,1);
      digitalWrite(led2,1);
      digitalWrite(led3,0);
      cmode=3;
    }
  }

  switch(cmode)
  {
    case 1:
      if (digitalRead(rung)==1)
      {
        tich(1,50);
        if (digitalRead(rung)==1)
        {
          digitalWrite(buzz,1);
          delay(2000);
          send_sms("Canh bao trom. ", my_phone);
          while(digitalRead(rung)==1);
          digitalWrite(buzz,0);
        }
      }
      else digitalWrite(buzz,0);
      break;
    case 2:
      mpu_read();
      ax = (AcX-2050)/16384.00;
      ay = (AcY-77)/16384.00;
      az = (AcZ-1947)/16384.00;
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.println(az);
      if (ax>0.5 || ax<-0.5 || ay>0.5 || ay<-0.5)
      {
        tich();
        mpu_read();
        ax = (AcX-2050)/16384.00;
        ay = (AcY-77)/16384.00;
        az = (AcZ-1947)/16384.00;
        if (ax>0.5 || ax<-0.5 || ay>0.5 || ay<-0.5)
        {
          digitalWrite(red,0); digitalWrite(green,1);
          if (ckl==0)
          {
            Serial.println("Canh bao lat xe");
            send_sms("Canh bao lat xe. ", my_phone);
            ckl=1;
          }
          tich(1,1000);
        }
      }
      else 
      {
        ckl=0;
        digitalWrite(green,0); digitalWrite(red,1);
      }
      break;
    case 3:
      if (Serial2.available()>0)
      {
        String rev = Serial2.readStringUntil('\0');
        Serial.println(rev);
        rev.toUpperCase();
        if (rev.indexOf("GETGPS")!=-1)
        {
          int tt = rev.indexOf("+84");
          String sdt = "0" + rev.substring(tt+3,tt+12);
          Serial.println("Gui tin nhan Dinh vi:" +sdt);
          send_sms("Vi tri: ", sdt);
          tich(3,100);
        }
      }
      break;
    default:
      break;
  }
}
