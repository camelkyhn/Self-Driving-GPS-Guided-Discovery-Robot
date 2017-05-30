# Self-Driving-GPS-Guided-Discovery-Robot
  First of all, my project's aim is to find the mines and the obstacles in the area and record their coordinates to an SD card. But I had no time to make a metal detector, so it can record only obstacles right now. Additionally, I did not handle all the failure possibilities about my sensors like compass and gps module. For example if the robot goes in a sloping road, compass values will be changes. I should measure z axis changes and use gyroscope to rotate better and minimize the failures.

  Algorithm basically scans 10 meters to north or south and stops and then passes the next level to east. Each level has 1 second distance to another level(about 0.5-1 m).For example, if I scanned 10 meters to north then I pass the 1 second dictance to east and then I started to scan 10 meters to south. If it's finished, then I'm passing the next level to east and starting to scan 10 meters to north and it goes on like that. The travel looks like shape of the letter "S".
  
Materials:
Arduino Mega 2560
HC-SR04 Sonar sensor
HMC5883 Compass sensor
GY-NEO6MV2 GPS Module
SD Card Module
2 DC Motors Tamiya 70168(+1 Servo to sonar for more safe trip)
Adafruit Motor Shield
