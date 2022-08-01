#include <Arduino.h>
#include <Wire.h>
//UnitV AI Camera
//白SCL
//黄色:SDA
//UnitV のスレーブアドレス
#define UV_ADDRESS  0x24
double getAngleFromCenter(double x,double y){
  double angle;
  if(x > 35){
    angle = 90-(180/PI)*atan2(y,x-35);
  }else{
    angle = -(90-(180/PI)*atan2(y,35-x));
  }
  return angle;
}
//0:中心x,2:中心y,4:幅w,6:高さ
void SendID(int ID){
  Wire.beginTransmission(UV_ADDRESS);
  Wire.write(ID);
  Wire.endTransmission();
}
//受け取った配列の場所にUnitVのデータを格納していく
void getData(int16_t *data){
  for(int i=0;i<8;i+=2){
      SendID(i);
      //delay(400);
      Wire.requestFrom(UV_ADDRESS, 2, true );
      //読み取りが終了するまで待機:availableは読み取りできる残りbyte数
      //while (Wire.available() < 2);
      data[i/2] =  Wire.read() << 8 | Wire.read();
      delay(100);
  }
}
