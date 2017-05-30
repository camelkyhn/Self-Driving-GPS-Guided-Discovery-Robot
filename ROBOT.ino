#include <I2Cdev.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <AFMotor.h>
#include <NewPing.h>

//Adafruit Motor Shield
AF_DCMotor left_motor(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor right_motor(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm

//HC-SR04 Sonar sensor
#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//wire pins are in my board( Mega2560)  20 (SDA), 21 (SCL)
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

//GY-NEO6MV2 GPS Module
TinyGPS gps;
#define rxPin 14 // to yellow wire in my gps
#define txPin 15

bool feedgps();
boolean condition;
boolean finish = false; // to writing records to file.
boolean passNextLevel = false;
boolean toNorth, beginning = true;
float flat, flon,flat1,flon1,x2lat,x2lon,head,distance;
int i,levelIndex = 0; // i for each record, levelIndex for how many level to scan.(And I want to travel 20 level to EAST)

//SD Card
File myFile;

void setup() 
{
  Serial.begin(9600);
  delay(100);                   // Power up delay

  Serial.println("S.A.");
  Serial1.begin(9600);

  Wire.begin();

  Serial1.print("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
  Serial1.print("$PMTK220,200*2C\r\n");

  // Set operating mode to continuous in compass
  Wire.beginTransmission(address); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
}

void loop() 
{
  if(beginning)
  {
    feedgps();
    unsigned char cc = Serial1.read();
    float flat, flon,x2lat,x2lon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    
    float flat1=flat;  // flat1 = our current latitude. flat is from the gps data. 
    float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
    Serial.println("latitude: ");
    Serial.println(flat1);
    Serial.println("longtitude: ");
    Serial.println(flon1);

    distance = getGpsDistance(flat1, flon1);
    head = getHeadingDegree();
    beginning = false;
    feedgps();
    Serial.println("S.A.");
  }
  if(finish)
  {
    //We are done.
    stopMotors();
    //close the file:
    myFile.close();
  }
  else
  {
    if( distance < 2) //we came to target spot.
    {
      stopMotors();
      feedgps();
      rotationCorrection(200,head-90);
      ifObstacle(flat1,flon1);
      moveForward();
      delay(1000);
      ifObstacle(flat1,flon1);
      stopMotors();
      feedgps();
      levelIndex++; // I scanned one level
      
      if(levelIndex>20)
      {
        finish = true;
      }
    
      if(toNorth)
      {
        rotationCorrection(200,head);
        flon1 = flon1 + 0.000010; //10 meters to north
        toNorth = false; //for the next level
        feedgps();
      }
      else
      {
        rotationCorrection(200,head-180);
        flon1 = flon1 - 0.000010; //10 meters to south
        toNorth = true; //for the next level
        feedgps();
      }
    }
    else
    {
      condition = directionTrue(flat1, flon1); //is my direction looking to target coordinates?
      if(condition)
      {
        ifObstacle(flat1,flon1);
        moveForward();
        ifObstacle(flat1,flon1);
        delay(3000);
        stopMotors();
      }
      else
      {
        rotationCorrection(200,getGpsAngle(flat1, flon1));
      }
    }
  }
}

//Can be used for finding mines
void ifObstacle(float aimLatitude, float aimLongtitude)
{
  float head = getHeadingDegree();
  float distance = sonar.ping_cm();
  unsigned long age;
  if (distance < 50)
  {
    stopMotors();
    //get coordinates of obstacle or mine and record them to sd card.
    gps.f_get_position(&flat, &flon, &age);
    // if the file opened okay, write to it:
    if (myFile) 
    {
      myFile.print("Obstacle");
      myFile.print(i);
      myFile.print(" latitude: ");
      myFile.print(flat);
      myFile.print(" longtitude ");
      myFile.println(flon);
      i++;
    } 
    else 
    {
      // if the file didn't open, print an error:
    }
    
    rotationCorrection(200, head-90);
    ifObstacle(aimLatitude,aimLongtitude);
    moveForward();
    delay(1000);
    stopMotors();
    rotationCorrection(200,head);
    ifObstacle(aimLatitude,aimLongtitude);
    while(directionTrue(aimLatitude, aimLongtitude))
    {
      rotationCorrection(200, getGpsAngle(aimLatitude, aimLongtitude));
    }
  }
}

boolean directionTrue(float aimLatitude, float aimLongtitude)
{
  feedgps();

  unsigned char cc = Serial1.read();
  float flat, flon,x2lat,x2lon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  feedgps();
  

  float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
  float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
  float dist_calc=0;
  float angle_calc=0;
  x2lat=aimLatitude;
  x2lon=aimLongtitude;
  //------------------------------------------ distance formula below. Calculates distance from current location to waypoint

  dist_calc=sqrt((((flon1)-(x2lon))*((flon1)-(x2lon)))+(((x2lat-flat1)*(x2lat-flat1))));
  dist_calc*=110567 ; //Converting to meters

  //=======================angle==========================================
  angle_calc=atan2((x2lon-flon1),(x2lat-flat1));

  float declinationAngle2 = 57.29577951;
  angle_calc*= declinationAngle2;
  feedgps();
  

  //angle correction sanırım
  if(angle_calc < 0){
    angle_calc = 360 + angle_calc;
    feedgps();
    
  }
  // Check for wrap due to addition of declination.
  if(angle_calc > 360){
    angle_calc= angle_calc - 360;
    feedgps();
    
  }

  float angleDegrees = angle_calc;
  feedgps();
  //this is compass code==================================

  // Initiate communications with compass
  float headingDegrees = getHeadingDegree();
  feedgps();
  

  //Am I looking the right angle?
  return ((angleDegrees - 5) < headingDegrees) && ((angleDegrees + 5) > headingDegrees);
}

//this function returns the distance between robot's and target coordinates.
float getGpsDistance(float aimLatitude, float aimLongtitude)
{
  unsigned char cc = Serial1.read();
  float flat, flon,x2lat,x2lon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  feedgps();
  

  float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
  float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
  float dist_calc=0;
  x2lat=aimLatitude;
  x2lon=aimLongtitude;
  //------------------------------------------ distance formula below. Calculates distance from current location to waypoint

  dist_calc=sqrt((((flon1)-(x2lon))*((flon1)-(x2lon)))+(((x2lat-flat1)*(x2lat-flat1))));
  dist_calc*=110567 ; //Converting to meters

  return dist_calc;
}

//this function returns the angle between robot's and target coordinates.
float getGpsAngle(float aimLatitude, float aimLongtitude)
{
  unsigned char cc = Serial1.read();
  float flat, flon,x2lat,x2lon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);

  feedgps();

  float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
  float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
  float angle_calc=0;
  x2lat=aimLatitude;
  x2lon=aimLongtitude;

  //=======================angle==========================================
  angle_calc=atan2((x2lon-flon1),(x2lat-flat1));

  float declinationAngle2 = 57.29577951; // Yani bu dünyanın eksenindeki kayma açısı ise ebenin amı ali sami en azından 22 mart'mış gibi davranırız.
  angle_calc*= declinationAngle2; // Aslında çarpmak değil de toplamak gerekmiyor mu? Bi de 57 falan bulmuş aq dünya 23 derece kayıyo sadece .s

  feedgps();

  //angle correction sanırım
  if(angle_calc < 0){
    angle_calc = 360 + angle_calc;
    feedgps();
  }
  // Check for wrap due to addition of declination.
  if(angle_calc > 360){
    angle_calc= angle_calc - 360; // bu ne aq? hayır 360 dan fazlaysa diye bakması gerekmiyo mu aq? Onu geçtim adam değişkeni kendine eşitlemiş xD
    feedgps();
  }
  return angle_calc;
}

//This function make robot turn to right angle
void rotationCorrection(int timeToRun, float aimDegree)
{
  boolean wrongAngle = true;
  float head = getHeadingDegree();
  while(wrongAngle)
  {
    Serial.println("S.A.");
    if(aimDegree + 10 > head && aimDegree - 10 < head)
    {
      wrongAngle = false;
      stopMotors();
    }
    else
    {
      if(aimDegree > head){
      turnLeft();
      }
      else{
        turnRight();
      }
      delay(timeToRun);
      stopMotors();
      head = getHeadingDegree();
    }
  }
}

//Get the compass value to know robot's heading angle.
float getHeadingDegree()
{
  //this is compass code==================================      
  int x, y, z;
  feedgps();
  
  // Initiate communications with compass
  Wire.beginTransmission(address);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(address, 6);    // Request 6 bytes; 2 bytes per axis
  if(6<=Wire.available()) // If 6 bytes available
  {
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
    feedgps();
  }

  float heading = atan2(y,x);
  float declinationAngle = 0.0457; // Manisa'nınki farklı değişmesi lazım.
  heading += declinationAngle;
  feedgps();

  // Correct for when signs are reversed.
  if(heading < 0)
  {
    heading += 2*PI;
    feedgps();
  }

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
  {
    heading -= 2*PI;
    feedgps();
  }

  float headingDegrees = heading * 180/M_PI;
  feedgps();
  return headingDegrees;
}

static bool feedgps()        
{
  while (Serial1.available())
  {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
}

//motor functions start here
void moveForward()
{//motors go forward
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
}

void moveBackward()
{//motors go backward
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
}

void stopMotors()
{//motors stop
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
}

void turnRight()
{
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);
}

void turnLeft()
{
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);
}

