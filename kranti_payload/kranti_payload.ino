#include <XBee.h>
#include <TinyGPS.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <Printers.h>
#include <SFE_BMP180.h>

#define p0 914.8
#define pv 38
#define startvid 10
#define buzzer 35            //buzzer
#define ead 0x50
#define ALTITUDE 0.0

//creating required objects
MPU6050 accelgyro;
TinyGPS gps;
XBee xbee = XBee();
SFE_BMP180 pressure;

void getgps(TinyGPS &gps);

int i = 0;
int16_t batlevel = 0;
int16_t ax,ay,az;
int16_t gx,gy,gz;
unsigned long previousMillis = 0;
unsigned long cnt = 0;
long tmp=0;
byte payload[pv];
float latitude, longitude;
float gps_alti, bmp_alti;
unsigned int ev = 0;
char status;
double T,P,a;
boolean hei = false;

XBeeAddress64 addr64 = XBeeAddress64(0x0, 0x0);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

void setup()
{
  pinMode(A0, INPUT);
  pinMode(startvid, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(19200);
  Wire.begin();
  accelgyro.initialize();          //initialize IMU
  xbee.setSerial(Serial);          //xbee on serial 0
  digitalWrite(startvid,HIGH);
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
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis <= 4000UL) {
      digitalWrite(startvid, LOW);              //actuate camera after 3 seconds
  }
  else
  {
    digitalWrite(startvid, HIGH);
  }

  while(Serial1.available())
  {
    int c = Serial1.read();
    if(gps.encode(c))                        //checks for valid sentence
    {
      getgps(gps);                           //Parses data
    }
  }
  
  
    
  if(cnt>=5000)                              //randomly selected value to reduce processor load
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
    
    accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    wtp((ax>>8)&0xff);
    wtp(ax&0xff);
    wtp((ay>>8)&0xff);
    wtp(ay&0xff);
    wtp((az>>8)&0xff);
    wtp(az&0xff);
    wtp((gx>>8)&0xff);
    wtp(gx&0xff);
    wtp((gy>>8)&0xff);
    wtp(gy&0xff);
    wtp((gz>>8)&0xff);
    wtp(gz&0xff);
    
    ftol(latitude);
    ftol(longitude);
    ftol(gps_alti);
    ftol(T);
    ftol(P);
    ftol(a);
    if(hei)
      wtm();
    cnt = 0;            //reset cnt
    xbee.send(zbTx);      //sends via Xbee
  }
  if(a >= 95.0)
  {
    hei = true;
  }
  if( a > 55.0 && a  < 85.0 && hei)
     digitalWrite(buzzer,HIGH);
 
 cnt++;
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

//function to process GPS raw data
void getgps(TinyGPS &gps)
{
  gps.f_get_position(&latitude, &longitude);
  int year;
  byte month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
  gps_alti = gps.f_altitude();
  
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
  if(ev>=8192)
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
