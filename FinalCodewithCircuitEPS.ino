/**************Libraries***************/
#include <XBee.h>
#include <EEPROM.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>            
#include <TimeLib.h> 
#include <PWMServo.h>
#include <SD.h>

/**************Variable***************/
bool transPaused = true;
int ledpin = 13;
float referenceAltitude,actualAltitude,realAltitude,i,t,volts;
int flag,val,trig=3;
uint8_t GPS_TIME_1, GPS_TIME_2, GPS_TIME_3;
int MISSION_TIME_1, MISSION_TIME_2, MISSION_TIME_3;
const float referenceVolts = 3.3;
const float R1 = 4700; 
const float R2 = 1000;
const float resistorFactor = 1023.0 * (R2/(R1 + R2));
const int batteryPin = A4;
const int airspeedPin = A0; 
const float airDensity = 1.225; 
const float pitotFactor = 0.005;
unsigned long time_now = 0;
const int chipSelect = BUILTIN_SDCARD;
uint32_t timer = millis();
char storedArray[21],guiCommand,buffers[200],AIR_SPEED[10],VOLTAGE[10];
char ALTITUDE[10],GPS_LATITUDE[10],GPS_LONGITUDE[10],GPS_ALTITUDE[10];
char TEMPERATURE[10],PRESSURE[10],TILT_X[10],TILT_Y[10],ROT_Z[10];
char HS_DEPLOYED = 'N',PC_DEPLOYED = 'N',receivedData;
int period=1000, GPS_SATS, actualTime,MISSION_TIME;
int a,packetCount,packetFlag,PACKET_COUNT;

/**************Define***************/
#define MPU9250_ADDR 0x68
#define GPSSerial5 Serial5
#define GPSECHO false
#define TIME_HEADER  "T"

/**************Object Declaration***************/
Adafruit_BMP280 bmp; 
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
MPU9250_asukiaaa mySensor;
Adafruit_GPS GPS(&GPSSerial5); 
PWMServo pcservo;
PWMServo hsservo; 
File myFile;

/**************User Defined Function Declaration***************/
void xbeedata();
void bmpsetup();
void bmploop();
void tiltsetup();
void tiltloop();
void rotasetup();
void rotaloop();
void gpssetup();
void rtcsetup();
void gpsloop();
void rtcloop(); 
void packcountsetup();
void packcountloop();
void battvoltloop();
void airspeedloop();
void bmpcalib();
void videocalib();
void sdloop();
void bufferfunc();

/**************Setup***************/
void setup()
{
  pinMode(ledpin,OUTPUT);
  digitalWrite(ledpin,HIGH);
  Serial5.begin(9600);
  Wire.begin();
  bmpsetup();
  bmpcalib();
  tiltsetup();
  rotasetup();
  gpssetup();
  rtcsetup();
  videocalib();
  packcountsetup();
  hsservo.attach(9);
  pcservo.attach(11);
  Serial5.println("Arduino started receiving bytes via XBee");
}

void loop()
{
  if(millis() > time_now + period)
  {
   xbeedata();
   bufferfunc();
   time_now=millis();
  }
}

void xbeedata() 
{
  if(Serial5.available()>0) 
  {
    receivedData = Serial5.read();
  }
  switch(receivedData)
  {
   case '1':
   transPaused = 0;
   break;

   case '2':
   transPaused = 1;
   break;

   case '3':
   hsservo.write(90);
   break;

   case '4':
   hsservo.write(30);
   break;

   case '5':
   pcservo.write(90);
   break;

   case '6':
   pcservo.write(30);
   break;
      
   case '7':
   digitalWrite(trig, LOW);   
   delay(750);               
   digitalWrite(trig, HIGH);  

   default:    
   receivedData=0;
   break;
  }
}

/////////////////////BMP280 Sensor/////////////////////////////
void bmpsetup() 
{
  Serial5.println(F("BMP280 test"));
  if(!bmp.begin(0x76))
  {
    Serial5.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   
  flag=EEPROM.read(25);
  if(flag==0)
  {
    referenceAltitude = bmp.readAltitude();
    EEPROM.write(20,referenceAltitude);
    flag = 9;
    EEPROM.write(25,flag);
  }
  else
  {
    referenceAltitude = EEPROM.read(20);
  }
  Serial5.print("The reference Altitude is : ");
  Serial5.println(referenceAltitude);
}

void bmpcalib()
{
  for(unsigned int cal = 0 ; cal < EEPROM.length() ; cal++ )
  {
    EEPROM.write(cal, 0);
  }
}

void bmploop()
{
  dtostrf(bmp.readTemperature(),1,3,TEMPERATURE);
  dtostrf((bmp.readPressure()/1000),1,3,PRESSURE);
  actualAltitude=(bmp.readAltitude());
  realAltitude=actualAltitude-referenceAltitude;
  dtostrf(realAltitude,1,1,ALTITUDE);
}

//////////////////MPU9250 Sensor for Tilt Angles///////////////////////
void tiltsetup() 
{
  if(!myMPU9250.init())
  {
    Serial5.println("MPU9250 does not respond");
  }
  else
  {
    Serial5.println("MPU9250 is connected");
  }
  Serial5.println("Position you MPU9250 flat and don’t move it – calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial5.println("Done!");
  myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void tiltloop() 
{
  xyzFloat angle = myMPU9250.getAngles();
  dtostrf(angle.x,1,2,TILT_X);
  dtostrf(angle.y,1,2,TILT_Y);
}

//////////////////MPU9250 Sensor for Rotation Rate///////////////////////
void rotasetup() 
{
  while(!Serial5);
  Serial5.println("started");
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
}

void rotaloop() 
{
  uint8_t sensorId;
  int result;
  result = mySensor.readId(&sensorId);
  if(result == 0) 
  {
  } 
  else 
  {
    Serial5.println("Cannot read sensorId " + String(result));
  }
  result = mySensor.gyroUpdate();
  if (result == 0) 
  {
    dtostrf(mySensor.gyroZ(),1,3,ROT_Z);
  } 
  else 
  {
    Serial5.println("Cannot read gyro values " + String(result));
  }
  delay(200);
}

//////////////////Mission Time (RTC)///////////////////////
void rtcsetup()
{
  setSyncProvider(getTeensy3Time);
  while(!Serial5);
  delay(100);
  if(timeStatus()!= timeSet) 
  {
    Serial5.println("Unable to sync with the RTC");
  } 
  else
  {
    Serial5.println("According to UTC Time");
  }
}

void rtcloop()
{ 
  actualTime=hour()*60+minute();
  MISSION_TIME = actualTime-(5*60+30);
  MISSION_TIME_1 = MISSION_TIME/60;
  MISSION_TIME_2 = MISSION_TIME%60;
  MISSION_TIME_3 = second();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

//////////////////Packet Count///////////////////////
void packcountsetup() 
{
  packetCount=EEPROM.read(5);
  packetFlag=EEPROM.read(8);
  if(packetFlag==0)
  {
    packetCount=0;
    EEPROM.write(5,packetCount);
    packetFlag=1;
    EEPROM.write(8,packetFlag);
  }
  else
  {
    packetCount=EEPROM.read(5);
    packetCount++;
    EEPROM.write(5,packetCount);
  }
}

void packcountloop()
{
  Serial5.print("Packet Count:");
  Serial5.println(packetCount); 
  packetCount++;
  EEPROM.write(5,packetCount);
  delay(1000);
}

//////////////////Adafruit MTK3339 GPS Sensor///////////////////////
void gpssetup()
{
  Serial5.println("Adafruit GPS library basic parsing test!");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  delay(1000);
  GPSSerial5.println(PMTK_Q_RELEASE); 
}

void gpsloop()
{
  GPS_TIME_1 = GPS.hour; 
  GPS_TIME_2 = GPS.minute;  
  GPS_TIME_3 = GPS.seconds;  
  dtostrf(GPS.latitude,1,4,GPS_LATITUDE);
  dtostrf(GPS.longitude,1,4,GPS_LONGITUDE);
  dtostrf(GPS.altitude,1,1,GPS_ALTITUDE);
  GPS_SATS = (int)GPS.satellites;
}

//////////////////Battery Voltage Sensor///////////////////////
void battvoltloop()
{
  val = analogRead(batteryPin);
  volts = (val / resistorFactor) * referenceVolts;  
  dtostrf(volts,1,1,VOLTAGE);
}

//////////////////MS4525DO Airspeed Sensor///////////////////////
void airspeedloop() 
{
  int rawValue = analogRead(airspeedPin);
  float differentialPressure = rawValue * 5.0 / 1023.0; 
  float airspeed = sqrt(2 * differentialPressure / (airDensity * pitotFactor));
  dtostrf(airspeed,1,4,AIR_SPEED);
}

//////////////////Adafruit 3202 Mini Spy Camera//////////////////////
void videocalib()
{
  pinMode(trig, OUTPUT);         
  digitalWrite(trig, HIGH); 
}

////////////////////////SD Card Storage///////////////////////////
void sdloop()
{
  if (!SD.begin(chipSelect)) 
  {
    Serial5.println("Initialization failed!");
    return;
  }
  myFile = SD.open("CodeData.csv", FILE_WRITE);
}

//////////////////Buffer for Telemetry Packet Data///////////////////////
void bufferfunc()
{
  if(!transPaused)
  {
    bmploop();
    tiltloop();
    rotaloop();
    gpsloop();
    rtcloop();
    battvoltloop();
    // airspeedloop();
    PACKET_COUNT = packetCount; 
    PACKET_COUNT++;
    EEPROM.write(5,PACKET_COUNT);
    sprintf(buffers, "2005, %d:%d:%d, %d, %s, %s, %s, %s, %s, %02d:%02d:%02d, %s, %s, %s, %d, %s, %s, %s", MISSION_TIME_1, MISSION_TIME_2, MISSION_TIME_3, PACKET_COUNT, ALTITUDE, AIR_SPEED, TEMPERATURE, VOLTAGE, PRESSURE, GPS_TIME_1, GPS_TIME_2, GPS_TIME_3, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, TILT_X, TILT_Y, ROT_Z);
    Serial5.println(buffers);
  }
  myFile = SD.open("CodeData.csv", FILE_WRITE);
  myFile.print(buffers);
  myFile.println();
  myFile.close();
  int j=20;
  char array[j];
  myFile = SD.open("sd_card.csv");
  myFile.read(&array[j], array[j]);
  myFile.close();  
}