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

/*
 *補正についての参考https://qiita.com/Suzumushi724/items/0b25efeaaa2c416f0f83

*/

#include <Arduino_LSM9DS1.h>
#include <Math.h>

//sda 21  scl 22



float acx,acy,acz;
float magx,magy,magz;
float gyrox,gyroy,gyroz;
double angle;
double gyodegree;
float magxhose=2.1166;
float magyhose=2.6555;
float theta;

float LSM_mag[2];
float fix_value[2];

/////////////////////////////
void fix(){//地磁気の補正を行う関数これを行う前にモーターを動かしておく必要がある。一回転させたい。
  float max_buffer[2];
  float min_buffer[2];
  Serial.println("Start!");
  for(int i=0;i<30;i++){
    for(int v=0;v<2;v++){
      IMU.readMagneticField(magx, magy, magz);
      LSM_mag[0]=magx;
      LSM_mag[1]=magy;
     
      float tmp=LSM_mag[v];

      if(i==0){
        max_buffer[v]=tmp;
        min_buffer[v]=tmp; 
      }
      else{
        if(tmp>max_buffer[v]){
          max_buffer[v]=tmp;
        }
        if(tmp<min_buffer[v]){
          min_buffer[v]=tmp;
        }
      }
    }
    delay(300);//0.3*30=9秒回転
  }
  for(int i=0;i<2;i++){
    fix_value[i]=(max_buffer[i]+min_buffer[i])/2;
  }
}

float compass(float x_mag,float y_mag){//地磁気をコンパスとするやつ
  float theta2 = atan2(y_mag,x_mag)*180/3.1415;
  //if(x_mag>0){
    //if(y_mag<0){
      //theta=theta+360;
    //}
  //}

  if(theta2<0){
    theta2=theta2+360;
  }
  return theta2;
}


//////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
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


  fix();
  while(1){
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(magx, magy, magz);
      magx=magx-fix_value[0];
      magy=magy-fix_value[1];
      theta=compass(magx,magy);
    
      Serial.print("theta:");
      Serial.print(theta);
      Serial.print('\n');
    }
    delay(10);
  }



  
}
