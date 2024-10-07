#define red 14
#define green 13
#define buzz 27
#define led3 26
#define led2 25
#define led1 33
#define rung 23
#define sw3 19
#define sw2 18
#define sw1 5

#include <Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
void(* resetFunc) (void) = 0;

void mpu_init()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void mpu_read()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void tich(int x=1, int y=100)
{
  while(x--)
  {
    digitalWrite(buzz,1); delay(y);
    digitalWrite(buzz,0); delay(y);
  }
}

String guidulieu(String lenh, const int thoigian, boolean debug=true) 
{
  String chuoigiatri = "";
  Serial2.print(lenh);
  long int time = millis();
 
  while ((time + thoigian) > millis()) {
    while (Serial2.available()) {
      char c = Serial2.read(); // read the next character.
      chuoigiatri += c;
      if (chuoigiatri.indexOf("OK")!=-1 && debug)
      {
        Serial.println(chuoigiatri);
        return chuoigiatri;
      }
    }
  }
  Serial.print(chuoigiatri);
  return chuoigiatri;
}
void checkSIM()
{
  while (1)
  {
    String st = guidulieu("AT\r\n",1000,true);
    if (st.indexOf("OK")!=-1) break;
    digitalWrite(green,0); delay(500);
    digitalWrite(green,1); delay(500);
  }
  tich(1,100);
  guidulieu("AT+GIZSTOP\r\n",1000,true);
  while (1)
  {
    String st = guidulieu("AT+CPIN?\r\n",1000,false);
    String st2 = guidulieu("AT+CMGF=1\r\n",1000,false);
    String st3 = guidulieu("AT+CNMI=2,2,0,0,0\r\n",1000,false);
    if (st.indexOf("+CPIN:READY")!=-1 && st2.indexOf("OK")!=-1 && st2.indexOf("OK")!=-1) break;
    tich(1,1000);
  }
  tich(1,100);
  guidulieu("AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n",1000,false);
  
  delay(1500);
  tich(3,100);
  Serial.println("START....");
}
void getLocationA9(String *v)
{
  tich(1,80);
  guidulieu("AT+GPS=1\r\n",1000,true);
  tich(3,80);
  delay(3000);
  int cnt=0;
  while(cnt<100)
  {
    tich(1,100);
    String getGPS = guidulieu("AT+LOCATION=2\r\n",3000,false);
    int st = getGPS.indexOf("OK");
    if (st!=-1)
    {
      String dat="";
      getGPS = getGPS.substring(getGPS.indexOf('\n'));
      for (int j=0;j<getGPS.length();j++)
      {
        if(getGPS[j]=='.' || getGPS[j]==',' || (getGPS[j]>='0' && getGPS[j]<='9'))
        dat += getGPS[j];
      }
      dat = dat.substring(1);
      Serial.println("Data GPS: "+dat);
      *v = dat;
      return;
    }
    tich(1,1000);
    cnt++;
  }
}
void send_sms(String smstext, String _number)
{
  String vs="";
  String gps="https://www.google.com/maps/place/";
  getLocationA9(&vs);
  gps = gps+vs;
  Serial.println(gps);
  guidulieu("AT+CMGF=1\r\n",1000,false);
  guidulieu("AT+CMGS=\"" + _number + "\"\r\n",1000,false);
  guidulieu(smstext + gps, 500,false);
  Serial2.println((char)26);
  tich(3,100);
}

void pininit()
{
//  pinMode(12,OUTPUT); digitalWrite(12,0); delay(1000); 
//  digitalWrite(12,1); pinMode(12,INPUT);
  pinMode(buzz,OUTPUT); digitalWrite(buzz,0);
  pinMode(red,OUTPUT);  digitalWrite(red,1);
  pinMode(green,OUTPUT); digitalWrite(green,1);
  pinMode(led1,OUTPUT); digitalWrite(led1,1);
  pinMode(led2,OUTPUT); digitalWrite(led2,1);
  pinMode(led3,OUTPUT); digitalWrite(led3,1);
  
  pinMode(sw1,INPUT); // nhan 0
  pinMode(sw2,INPUT); // nhan 0
  pinMode(sw3,INPUT); // nhan 0
  pinMode(rung,INPUT); // rung 1
  tich();
  Serial.begin(9600);
  Serial2.begin(9600);
  checkSIM();
}
