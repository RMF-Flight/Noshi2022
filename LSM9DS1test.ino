/*
  Arduino LSM9DS1 - Simple Accelerometer

  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>
#include <Math.h>



float acx,acy,acz;
float magx,magy,magz;
float gyrox,gyroy,gyroz;
double angle;
double gyodegree;
double magxhose=2.1166;
double magyhose=2.6555;



void setup() {
  Serial.begin(115200);
  while (!Serial);
  //Serial.println("Started");

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //Serial.print("Accelerometer sample rate = ");
  //Serial.print(IMU.accelerationSampleRate());
  //Serial.println(" Hz");
  //Serial.println();
  //Serial.println("Acceleration in G's");
  //Serial.println("X\tY\tZ");
}

void loop() {
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acx, acy, acz);

    angle=asin(acy);//arcsinはradで渡される。
    gyodegree=angle*(180/M_PI);//定常状態での仰角を表示する。
    //Serial.print("angle:");
    //Serial.print(angle); 
    //Serial.print("grodeg:");
    //Serial.print(gyodegree); //radになってる

    //Serial.print('\t');   
    //Serial.print("acx:");
    //Serial.print(acx);
    //Serial.print('\t');
    //Serial.print("acy:");
    //Serial.print(acy);
    //Serial.print('\t');
    //Serial.print("acz:");
    //Serial.println(acz);
    
    
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyrox, gyroy, gyroz);

    //Serial.print("gyrox:");
    //Serial.print(gyrox);
    //Serial.print('\t');
    //Serial.print("gyroy:");
    //Serial.print(gyroy);
    //Serial.print('\t');
    //Serial.print("gyroz:");
    //Serial.println(gyroz);
  }

   if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magx, magy, magz);
    magx=magx+magxhose;
    magy=magy+magyhose;
    Serial.printf(" %f,%f,%f\n", magx, magy, magz);
   }


  delay(100);

  


  
}
