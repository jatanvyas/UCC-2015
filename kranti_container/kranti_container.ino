#include <XBee.h>
#include <Servo.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <I2Cdev.h>
#include <Printers.h>

SFE_BMP180 pressure;
Servo mech;
TinyGPS gps;
XBee xbee = XBee();

void getgps(TinyGPS &gps);

#define p0 914.8
#define pv 26
#define ALTITUDE 0.0
#define buzzer 35            //buzzer
#define ead 0x50

int16_t batlevel;
int i = 0;
long tmp = 0;
int pos =0;
unsigned long cnt = 0;
char status;
double T,P,a;
float latitude, longitude;
byte month, day, hour, minute, second, hundredths;
float gps_alti, gps_speed;
unsigned int ev = 0;
byte payload[pv];
boolean hei = false;
unsigned long previousMillis = 0;

XBeeAddress64 addr64 = XBeeAddress64(0x0, 0x0);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

void setup()
{
  Wire.begin();
  pinMode(buzzer, OUTPUT);
  pinMode(A0, INPUT);
  mech.attach(13);
  Serial.begin(115200);            // actual 57600  
  Serial1.begin(19200);            // GPS
  xbee.setSerial(Serial);          //xbee on serial 0
  
  if (pressure.begin())
  {
  }
  else
  {
    //while(1); // Pause forever.
  }
}

void loop()
{
   while(Serial1.available())     
   {
      int c = Serial1.read();    
      if(gps.encode(c))      
      {
        getgps(gps);         
      }
  }
  if(cnt >= 5000)
  {
    status = pressure.startTemperature();
    if (status != 0)
    {
      delay(status);
      status = pressure.getTemperature(T);
     
      if (status != 0)
      {
        status = pressure.startPressure(3);
        if (status != 0)
        {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          a = pressure.altitude(P,p0);
        }
       }
      }
    }
    batlevel = analogRead(A0);
    wtp((batlevel>>8)&0xff);
    wtp((batlevel)&0xff);
    ftol(T);
    ftol(P);
    ftol(a);
    ftol(latitude);
    ftol(longitude);
    ftol(gps_alti);   
    
    if(hei)
      wtm();
    
    xbee.send(zbTx);
    cnt = 0;
  }
  
  
  if(a > 60.0)
  {
    hei = true;
  }
  
  if( a > 40.0 && a  < 60.0 && hei)
     digitalWrite(buzzer,HIGH);
  
  if(a > 40.0 && a < 60.0 && hei)
    release_payload();
   
   cnt++;
}

void release_payload(void)
{
  for(pos = 0; pos <= 147; pos += 1)  
  {                                   
    mech.write(pos);               
    delay(4);                        
  } 
  for(pos = 147; pos>=0; pos -= 1)      
  {                                
    mech.write(pos);               
    delay(4);                        
  }
  digitalWrite(buzzer, HIGH);
}

void getgps(TinyGPS &gps)
{
  
  gps.f_get_position(&latitude, &longitude);
  
  int year;
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
  gps_alti = gps.f_altitude();
  //gps_speed = gps.f_speed_kmph();
}

void ftol(float l)
{
  byte y;
  tmp = (long)(l*1000000);
  y = ((tmp & 0xff000000)>>24);
  wtp(y);
  y = ((tmp & 0x00ff0000)>>16);
  wtp(y);
  y = ((tmp & 0x0000ff00)>>8);
  wtp(y);
  y = ((tmp & 0xff));
  wtp(y);
  
}

void wtp(byte b){
  if(i>=pv){
    i = 0;
  }
  payload[i] = b;
  i++;
}

void wtm(void)
{
  if(ev>=4096)
  {
    ev = 0;
  }
  for(int k=0;k<pv;k++)
  {
    writeEEPROM( ead, ev, payload[k]);
    ev++; 
  }
}

//main EEPROM
 
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}
