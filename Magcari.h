#include <Arduino.h>
//sda 21  scl 22

float LSM_mag[2];
float fix_value[2];

/////////////////////////////
void fix(){//地磁気の補正を行う関数これを行う前にモーターを動かしておく必要がある。一回転させたい。
  float max_buffer[2];
  float min_buffer[2];
  
  for(int i=0;i<30;i++){
    if(GPSflag=true){//タイマー都度処理
    GPSvalue();
    GPSflag=false;    
  }
    for(int v=0;v<2;v++){
      if(GPSflag=true){//タイマー都度処理
    GPSvalue();
    GPSflag=false;    
  }
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

  Serial.println("Calibration finished");
}

float compass(float x_mag,float y_mag){//地磁気をコンパスとするやつ
  float theta2 = atan2(y_mag,x_mag)*180/3.1415;
  //if(x_mag>0){
    //if(y_mag<0){
      //theta=theta+360;
    //}
  //}

  if(theta2<0){//0から360度にしたいときに使う。
    theta2=theta2+360;
  }
  return theta2;
}


//////////////////////////////////////////////
