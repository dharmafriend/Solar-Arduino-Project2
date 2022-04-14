#include "RTClib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>   //Auduino code library for math operations 
#include <Chrono.h>
 RTC_DS3231 rtc;
 //pins for actuator
 int in1=7;  //motor pin positive
int in2=8;   //second motor pin (negative?)
int ENA=5;     //pin for actuator motor board
const int Speed=150; //actuator motor speed


//hypothetical winch motor variables
int winchmotorsensitivityfactor=2;  //in degrees
int winchforday;

//pins and variables for tilt sensor
int tiltpinX=A5;
int tiltpinY=A4;
int tiltpinZ=A3;

//variables to convert tilt sensor info into angles
float valX, valY, valZ; //raw analog values for x, y, and z axis readings.
float accX, accY, accZ; //analog readings converted to acceleration, in g's.
float angleX, angleY, angleZ; //angle of inclination with x,y, and z axis.
float absAngleX, absAngleY, absAngleZ; //positive incline angles

float mapf(float x_, float in_min, float in_max, float out_min, float out_max)
 {//****start mapf function****
   return (x_ - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 }//****endof mapf function****


// variables for solar calculations
float LST; //local solar time
float CT; //clock time
float a1;
float a2;
float a3;
int DT=0;
int y;
int yt;
int xt;
int m;
float elevationatsolarnoon;
float elevationatsolarnoonold;
float converttoradians=0.01745329;
int daylight;
int dayofyear;
int dayofyearold;
float EoT;
float latitude=39.96;
int Lstd=75;  // standard longitude for time zone
float Lloc=82.9988;  //actual exact longitude of location
int n;  //day of the month
int currenthour;
int currentminute;
double B; //for equation of time
float hourangleatsunrise;
float pie=3.141593;
float h; //hour angle
float d; //declination
float Sunrise;
float Sunset;
float elevation;
float TimeCorrect;
float azimuth;
float TC60;
float d3; 
float d2;
float d1;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {
  // put your setup code here, to run once:
 yt=0; //sets this to zero unless it's a leap year
 winchforday=0; //so winch will only lift once per day
Serial.begin(57600);
pinMode (in1,OUTPUT);
pinMode (in2,OUTPUT);
pinMode (ENA,OUTPUT);
pinMode (tiltpinX,INPUT);
pinMode (tiltpinY,INPUT);
pinMode (tiltpinZ,INPUT);

//constrain analog x, y, z readings to actual measured ranges
  constrain(valX,259,390);
  constrain(valY,254,384);
  constrain(valZ,263,392);

//clock setup
#ifndef ESP8266
while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}


void loop() {
  // put your main code here, to run repeatedly:
 DateTime now = rtc.now();
 y=now.year();
 m=now.month();
   n=now.day();
  
   if (now.month()==1){xt=0;}  //my clunky way of totaling the days in the year so far
   if (now.month()==2){xt=31;}
   if (now.month()==3){xt=59;}
   if (now.month()==4){xt=90;}
    if (now.month()==5){xt=120;}
   if (now.month()==6){xt=151;}
   if (now.month()==7){xt=181;}
   if (now.month()==8){xt=212;}
    if (now.month()==9){xt=243;}
   if (now.month()==10){xt=273;}
   if (now.month()==11){xt=304;}
 if(now.month()==12){xt=334;}
   if (y==2024){ if (now.month()>2){yt=1;}} //yt variable adds one day to dayofyear variable for leap years
   if (y==2028){ if (now.month()>2){yt=1;}}
   if (y==2032){ if (now.month()>2){yt=1;}}
   if (y==2036){ if (now.month()>2){yt=1;}}
   if (y==2040){ if (now.month()>2){yt=1;}}
   if (y==2044){ if (now.month()>2){yt=1;}}
   dayofyear=n+xt+yt;

   
   if (now.month()==3){                                      //determine if daylight savingstime 
      if (daylight==0){
    
      
//          if (strcmp(daysOfTheWeek[now.dayOfTheWeek()], daysOfTheWeek[0]) == 0 && now.day()>=8);
          if (isSunday() && now.day() > 7) {
            daylight=1;
          }
       }
   }
   if (now.month()==11 && daylight==1 && strncmp(daysOfTheWeek[now.dayOfTheWeek()], "Sunday", 12) == 0)
            {daylight=0;}
      
   
   currenthour=now.hour();
   currentminute=now.minute();
CT=currenthour + currentminute/60.0;  
//Serial.print ("   Currenttime(decimal) =  ");
//Serial.print (CT);
d2=(360.0/365.0)*(10.0+dayofyear);
d1=cos(d2*converttoradians);

d=-23.45*d1;  //calculating declination angle

B=(360.0/365.0)*(dayofyear-81.0);
EoT=9.87*sin(2*B*converttoradians)-7.53*cos(B*converttoradians)-1.5*sin(B*converttoradians);
TimeCorrect=(-4*(Lloc-Lstd))+EoT;
LST=CT+(TimeCorrect/60)-daylight;
TC60=TimeCorrect/60;
h=15.0*(LST-12.0); //hour angle in degrees
elevation=asin((sin(d*converttoradians)*sin(latitude*converttoradians))+(cos(d*converttoradians)*cos(latitude*converttoradians)*cos(h*converttoradians)))*180.0/3.1415;
Sunrise=12.0-((1.0/15.0)*acos(-tan(latitude*converttoradians)*tan(d*converttoradians))*180.0/3.1415)-TC60+daylight;
elevationatsolarnoon=asin((sin(d*converttoradians)*sin(latitude*converttoradians))+(cos(d*converttoradians)*cos(latitude*converttoradians)*cos(0*converttoradians)))*180.0/3.1415;
Sunset=12.0+((1.0/15.0)*acos(-tan(latitude*converttoradians)*tan(d*converttoradians))*180.0/3.1415)-TC60+daylight;
a1=(sin(d*converttoradians))*(cos(latitude*converttoradians));
a2=(cos(d*converttoradians))*(sin(latitude*converttoradians))*cos((h*converttoradians));
a3=cos(elevation*converttoradians);
azimuth=(acos((a1-a2)/a3))*180.0/3.1415;
hourangleatsunrise=15.0*(Sunrise-12.0);
if (LST > 12) {azimuth=360-azimuth;}
//Serial.print("   Daylight=");
   // Serial.print(daylight);
   Serial.print("   elevation at solar noon=");
    Serial.print(elevationatsolarnoon);
    //Serial.print("   Sunrise=");
  //  Serial.print(Sunrise);
  //  Serial.print("   Sunset=");
  //  Serial.print(Sunset);
  //  Serial.print("    EoT=");
  //  Serial.print(EoT);
  //  Serial.print("   declination=");
   // Serial.print(d);
   // Serial.print("   TimeCorrect=");
  //  Serial.print(TimeCorrect);
  //  Serial.print("   LST=   ");
  //  Serial.print(LST);
  //  Serial.print("   h=   ");
  //  Serial.print(h);
  //  Serial.print("   elevation=");
  //  Serial.print(elevation);
  //   Serial.print("   azimuth=");
  //  Serial.println(azimuth);
valX=analogRead(tiltpinX);
valY=analogRead(tiltpinY);
valZ=analogRead(tiltpinZ);
 delay(10);  // short delay to allow readings to stabilize

//map the analog sensor readings to a g value between -1 g to + 1 g
  //to match the values in AN-1057, the z readings were reversed.
  accX = mapf(valX,259.0,390.0,+1.0,-1.0); //user defined mapf function
  accY = mapf(valY,254.0,384.0,+1.0,-1.0); //user defined mapf function
  accZ = mapf(valZ,263.0,392.0,-1.0,+1.0); //user defined mapf function

  //calculate the angle of inclination with each axis.
  angleX = atan2(accX,(sqrt(pow(accY,2)+pow(accZ,2))))*(180/PI);
  angleY = atan2(accY,(sqrt(pow(accX,2)+pow(accZ,2))))*(180/PI);
  angleZ = atan2((sqrt(pow(accX,2)+pow(accY,2))),accZ)*(180/PI); 

  //use fabs() "f"loating point absolute value vs abs()
  absAngleX = fabs(angleX);
  absAngleY = fabs(angleY);
  absAngleZ = fabs(angleZ);

   Serial.print("analog readings for x, y, z: ");
Serial.print(valX, DEC);    // print the acceleration in the X axis
Serial.print(" ");          // prints a space between the numbers
Serial.print(valY, DEC);    // print the acceleration in the Y axis
Serial.print(" ");          // prints a space between the numbers
Serial.println(valZ, DEC);  // print the acceleration in the Z axis

Serial.print("angleX ");
Serial.println(angleX,DEC);
Serial.print("angleY ");
Serial.println(angleY,DEC);
Serial.print("angleZ ");
Serial.println(angleZ,DEC);

//at beginning of each day, set z angle to equal solar elevation at solar noon
if (CT > Sunrise){
if (winchforday==0){  //so will only happen once at new day

  if (elevationatsolarnoon>elevationatsolarnoonold){     //solar position will need to continually raise from winter solstice to summer solstice (once a day)
      if (elevationatsolarnoon < (angleZ-90)-winchmotorsensitivityfactor){    //(have to check that angleZ-90 is correct here!!!)
        //use motor to raise winch
}
      if (elevationatsolarnoon > (angleZ-90)-winchmotorsensitivityfactor){
        //stop winch motor
}winchforday=1;}

  if (elevationatsolarnoon<=elevationatsolarnoonold)  //solar position will get lower in the sky until winter solstice (northern hemisphere)
{
      if (elevationatsolarnoon > angleZ-winchmotorsensitivityfactor){
      //use motor to lower winch
      }
      if (elevationatsolarnoon < angleZ-winchmotorsensitivityfactor){
        //stop winch motor}
      }winchforday=1;
}}}


  //Begin tracking sun at sunrise
  if (CT > Sunrise+0.1){
    while (y < sin(h)*elevationatsolarnoon){
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
      analogWrite(ENA,Speed);
    }
      
    
    }

  //Sunset-retract fully
  if (CT > Sunset) {
    while (y > sin(hourangleatsunrise)*elevationatsolarnoon){
       digitalWrite(in1,LOW);  //retract until y is back to previous sunrise position
       digitalWrite(in2,HIGH);
       analogWrite(ENA,Speed);
        ;}}

//do nothing overnight
if (CT > Sunset+0.07){
  winchforday=0;
  yt=0;
  elevationatsolarnoonold=elevationatsolarnoon;

      while (CT > Sunset+0.06 && CT < Sunrise){
          digitalWrite(in1,LOW);
          digitalWrite(in2,LOW);
          delay(200000);
  }}
    delay (300000); //delay 5 minutes between readings
    
   
}


boolean isSunday() {
  DateTime now = rtc.now();
  // return( strncmp(daysOfTheWeek[now.dayOfTheWeek()], daysOfTheWeek[0], 12) == 0 );
  return( now.dayOfTheWeek() == 0 );
}
