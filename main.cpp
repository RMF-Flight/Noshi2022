#include <Wire.h>
#include <SPI.h>
//motor gps 

#include <ESP32Servo.h>
 
#include "SparkFun_TB6612.h"

#include <Arduino_LSM9DS1.h>
#include <Math.h>
//bluetooth
//#include <BluetoothSerial.h>

#include <TinyGPS++.h>
#include "SoftwareSerial.h"
//#include <Ticker.h>
#include "UnitV.h"


//モータドライバ
#define AIN1 12//
#define BIN1 32//
#define AIN2 14//
#define BIN2 33//
#define PWMA 26//
#define PWMB 25//
#define STBY 13//こいつ3V3でよくね？

//HCSR
#define TrigPin 15 // D1
#define EchoPin 2 // D2
const float time_30=0.5;//30ド回る
const float time_90=1.5;//90ド回る
const float time_10=0.1666;//10ド回る

TaskHandle_t thp[1];//マルチスレッドのタスクハンドル格納用


static const int RXPin = 34, TXPin = 0;//5,18成功。RXTXピンである必要はない。
const int SERVO = 27;
Servo servo;  



const double goal_x = 139.543420;             //ゴールの経度(初期値は適当,35.657941,139.542798
const double goal_y =35.658809; //学校のどこか
const double R = 6371000.0;  
//モータドライバのよくわからん変数(speedにかける)
const int offsetA = 1;
const int offsetB = 1;

bool GPSflag=false;

static const uint32_t GPSBaud = 9600;
double delta_x, delta_y, now_x, now_y, sita, goal_y_rad,temp1,temp2; 
double speedSound = 331.5 + 0.61 * 20; // 20は現在の気温
double Ultradistance = 0;
float acx,acy,acz;
float magx,magy,magz;
float gyrox,gyroy,gyroz;
float Magtheta;
double GPSsita;//北からの角度が出せる
float GPSsitaf;
float rotate;
float gyoukaku;
double r;//GPS用
double HCSRt;
//カメラ用
int16_t data[4];
int area = 0;
double per = 0.0;
int max_area = 240 * 320;
double Cameratheta = 0.0;
double x,y;
float xf,yf;
bool Cameraflag = false;
double Cameradist = 0;
int count30=0;
int count10=0;//コーンの角度を10度で割った値。この回数分10度回転する。
int sayucount=0;//コーンを認識するときに左右に交互に回るための値



//モータドライバ
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//GPS　Serialピンはtweliteで用いる
//Ticker ticker1;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);//受信するピン、送信するピン

////////////////////////////////////////////////////////////////////////////////////////////////////////



void trigger() {//HCSR04
  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite(TrigPin, LOW );
}

float GPScompass(){//GPSから方位角を算出するやつ
   delta_x = goal_x - now_x;           //ゴールまでのx,y座標の差分を読み込む
   delta_y = goal_y - now_y;

   //方角を出力する。
   delta_x = delta_x * 3.14/180;
   now_x = now_x * 3.14/180;
   goal_y_rad = goal_y * 3.14/180;
   now_y = now_y * 3.14/180;

   temp1 = cos(now_y)*tan(goal_y_rad);
   temp2 = sin(now_y)*cos(delta_x);

   GPSsita = atan2(sin(delta_x),(temp1-temp2));
   GPSsita = GPSsita * 180/3.14;  

   if(GPSsita<0){
    GPSsita = GPSsita + 360; //0~360 
   }
   GPSsitaf=(float)GPSsita;
   return GPSsitaf;

}

void Camera() {//カメラから距離とステアリング角度、面積を出す関数
  //Wire.begin();
  if( Cameraflag == false){
    Cameraflag = true;
    SendID(8);
    //delay(500);
    Serial.println("I2C:Start");
    
  }

  if(Cameraflag == true){
  getData(data);//240*320 QVGA
  delay(100);
  /*
  for(int i=0;i<4;i++){
    Serial.print(data[i]);
    Serial.print(",");
  }
  */
  area = abs(data[2]*data[3]);
  if(data[2] == -1 || data[3] == -1) area = 0;
  //Serial.print("area:");
  //Serial.println(area);
  Serial.print("面積:");
  Serial.println(area);
  Serial.print("距離:");
  Cameradist = cos((double)gyoukaku) * pow((double)area/31260,-1.0/1.966);
  if(Cameradist <= 1.0) Cameradist = -1;
  Serial.println(Cameradist);
  

  //per = (double)area / max_area; 
  //Serial.print("per:");
  //Serial.println(per);//int16_tからdoubleに変換するときor視野角を考慮した変換によって倍率が変化しているため1を超える

  //x:0~70,y:0~55
  
  x = (double)data[0];
  y = (double)data[1];
 
  Cameratheta = getAngleFromCenter(x,y);
  Serial.print("画像中心下部からの角度:");
  Serial.print(Cameratheta);
  Serial.println("度");
  }
}



void displayInfo(){//GPSが値を表示する関数
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
     delay(100);
    Serial.print(gps.location.lng(),6);
    Serial.print(F(","));
    Serial.print(gps.location.lat(),6);
    
    
    
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void GPSvalue()//GPSが値を返すだけのやつ。できればdisplayInfoと合体させたい。
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)//GPSが動作可能だったら続ける
    if (gps.encode(ss.read()))//GPSのデータをエンコードする
    delay(100);
      displayInfo();//情報を出力する。

  if (millis() > 5000 && gps.charsProcessed() < 10)//通信不可能
  {
    Serial.println(F("No GPS detected: check wiring."));
    //while(true);
  }

  
}



#include "FlightPin.h"
#include "LandingDetach.h"
#include "Magcari.h"


void Core0a(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    //サブで実行するプログラムを書く
    delay(2000);//1/1000秒待つ
    GPSvalue();   
  }
}




///////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  
   xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 1, &thp[0], 0); 

  
  

  Serial.begin(115200);//Serialはtxとrxでtweliteと通信するもの、mySerialは受信機と送信機の間。
  Serial.println("Start!");
  

  

  
  ss.begin(GPSBaud);//softwareserial専用のbaud
  //ticker1.attach_ms(1000, GPSvalue);
  servo.attach(SERVO);//サーボピン設定
    
  

  if (!IMU.begin()) {//9軸を動かすためのなにか
    Serial.println("Failed to initialize IMU!");
    
  }
  IMU.readGyroscope(gyrox, gyroy, gyroz);
  IMU.readAcceleration(acx, acy, acz);
  IMU.readMagneticField(magx, magy, magz);
  //Serial.printf(" %f,%f,%f\n", magx, magy, magz);

  
  pinMode(TrigPin, OUTPUT);//HCSRの設定
  pinMode(EchoPin, INPUT);


}

void loop(){

  
  
  
 

  ////////////////////////////////////////////////////////////////////////本制御
  flight_loop();//パラシュート放出確認
  Serial.println("mission3completed(parachuted)");
  landing_detect();//着地検知
  Serial.println("mission4completed(landed)");

  
  
  Serial.println("detaching the parachute...");//パラシュート分離
  int angle = 0;//サーボ
  servo.write(angle);
  delay(2000);
  angle=90;
  servo.write(angle);
  delay(2000);
  Serial.println("mission5completed(parachute detached)");

  //if(GPSflag=true){//タイマー都度処理
    //GPSvalue();
    //GPSflag=false;    
  //}


  
  Serial.println("10seconds forward");//10秒進んでパラシュートとかよけてもらう
  forward(motor1, motor2, 255);//前に進む
  delay(10000);

  //if(GPSflag=true){//タイマー都度処理
    //GPSvalue();
    //GPSflag=false;    
  //}

  Serial.println("Calibration Start!");//キャリブレーションスタート
  left(motor1, motor2, 255);
  fix();//地磁気を補正値を取得する。9秒間で1回転以上回るように。delayもこの中に入っている。
  Serial.println("Calibration finished");

  //if(GPSflag=true){//タイマー都度処理
    //GPSvalue();
    //GPSflag=false;    
  //}

  


  //GPS処理
   //GPSvalue();
    now_x=gps.location.lng();
    now_y=gps.location.lat();

   

   r = R * acos(sin(goal_y*3.14/180) * sin(now_y*3.14/180)+cos(now_y*3.14/180) * cos(goal_y*3.14/180)*cos((goal_x - now_x)*3.14/180));     
   r=abs(r);
   //Serial.print("toGoaldistance");
   //Serial.println(r);

   GPSsitaf=GPScompass();//GPSのコンパス(float)
   

   //Serial.print("toGoaltheta");
   //Serial.println(GPSsita);


   //地磁気計算
   IMU.readMagneticField(magx, magy, magz);
   delay(1000);
   magx=magx-fix_value[0];
   magy=magy-fix_value[1];
   Magtheta=compass(magx,magy);//地磁気コンパス

/////////////////////////////////////////////////mission5
   while(r>15){//もしゴールとの距離が15m以上なら
   //if(GPSflag=true){//タイマー都度処理
    //GPSvalue();
    //GPSflag=false;    
  //}
    rotate=GPSsitaf-Magtheta;//地磁気とGPSの角度差
    
    Serial.println("matching GPS and magnetic..");
    while(Magtheta>GPSsitaf+2 || Magtheta<GPSsitaf-2){//GPSと地磁気の角度を合わせる
      if( rotate>0){//もし正なら右側回転
        right(motor1, motor2, 255);
        delay(100);       
      }else{//もし負なら左側回転
        left(motor1, motor2, 255);
        delay(100);       
      }
      IMU.readMagneticField(magx, magy, magz);
      delay(100);
      magx=magx-fix_value[0];
      magy=magy-fix_value[1];
      Magtheta=compass(magx,magy); 
      //Serial.println(Magtheta); 
      delay(100);

     //if(GPSflag=true){//タイマー都度処理
     // GPSvalue();
     // GPSflag=false;    
      //} 
      



               
    }
    Serial.print("magtheta");
    Serial.println(Magtheta);
    Serial.println("thisway!");//地磁気とGPSを合わせ終わった
    
    Serial.println("10seconds forward");
    forward(motor1, motor2, 255);//10秒直進
    delay(10000);


    delay(1000);

    //if(GPSflag=true){//タイマー都度処理
      //GPSvalue();
      //GPSflag=false;    
    //} 
    Serial.println("why");
    
    delay(1000);


   
    now_x=gps.location.lng();
    now_y=gps.location.lat();
    delay(2000);
    r = R * acos(sin(goal_y*3.14/180) * sin(now_y*3.14/180)+cos(now_y*3.14/180) * cos(goal_y*3.14/180)*cos((goal_x - now_x)*3.14/180));     
    delay(2000);
    r=abs(r);
    Serial.print("toGoaldistance");
    Serial.println(r); 

    GPSsitaf=GPScompass();//GPSのコンパス
    delay(2000);

    Serial.print("toGoaltheta");
    Serial.println(GPSsita);
    //if(GPSflag=true){//タイマー都度処理
    
    delay(2000);

    //GPSflag=false;    
    //}
    
   }
   Serial.print("mission6completed(near the corn)");
   delay(1000);
////////////////////////////



   
   while(1){///////////////////////////////////////////////カメラの処理
    

//if(GPSflag=true){//タイマー都度処理
     // GPSvalue();
      //GPSflag=false;    
    //}
    //delay(1000);


    IMU.readAcceleration(acx, acy, acz);//仰角を出す。
    gyoukaku=asin(acy);//arcsinはradで渡される。
    //gyoukaku=gyoukaku*(180/M_PI);
    //Serial.println(gyoukaku);

    GPSvalue();
    now_x=gps.location.lng();
    now_y=gps.location.lat();
    

    Camera();
    Serial.println(Cameradist);
    while(Cameradist>30){//コーンが見つからないとき→15m以内にいるのに距離が30m以上
      if(sayucount==0){
        right(motor1, motor2, 255);//時計回りに微小回転させながらコーンを見つける。
        delay(200);       
      }else{
        left(motor1, motor2, 255);//時計回りに微小回転させながらコーンを見つける。
        delay(200);  
      }
      
      //仰角を出す。
      IMU.readAcceleration(acx, acy, acz);
      gyoukaku=asin(acy);//arcsinはradで渡される。
      //gyoukaku=gyoukaku*(180/M_PI);
      //Serial.println(gyoukaku);
      Camera();
      Serial.println(Cameradist);
      if(GPSflag=true){//タイマー都度処理
      GPSvalue();
      GPSflag=false;    
      }
    }

    if(sayucount==0){
      sayucount=1;
    }else{
      sayucount=0;
    }

    

    delay(1000); 
    

    //count30=abs((int)Cameratheta/30);//30度の倍数
    count10=abs((int)Cameratheta/10);//10度の倍数
    Serial.println(count10);

     if(Cameratheta<0){//角度が負のとき左方向に曲がり、時間回転
       left(motor1, motor2, 255);
       delay(time_10*(float)count10 * 1000);    
     }else{//角度が正のとき右方向に曲がり、時間回転
       right(motor1, motor2, 255);
       delay(time_10*(float)count10 * 1000);     
     }
     

     trigger();
     HCSRt = pulseIn(EchoPin, HIGH); // μS
     HCSRt = HCSRt / 2; //往復距離なので半分の時間
     Ultradistance = HCSRt * speedSound * 100 / 1000000; // 距離（cm）を計算
     Serial.print("Ultradistance");
     Serial.println(Ultradistance);
      if(Cameradist<=0.6){//もしカメラ距離が0.6以下なら終了
        SendID(9);
        Serial.print("cameradist");
        Serial.println(Cameradist);
        Serial.println("All missions are completed");//全ミッション終了
        brake(motor1, motor2);
        while(1);//無限ループ
      }else{
        forward(motor1, motor2, 255);//2秒直進
        delay(2000);  
      }
     
   
   }
   
   






   
   
   //if(r>10){

    
    
   //}



   

  

  


  
  


  
  //HCSR
  


  //9軸
  
   


//////////////////


//HCSRからの値
   
   //Serial.print("hcsr=");
   //Serial.println(distance);
//////////////


//GPS関連の操作
   //ゴールとの距離
   
////////////////



   





   //motor1.drive(255,1000);
   //motor2.drive(200,1000);
   //motor1.brake();

   //if(distance<10){
    //motor1.brake();
    //motor2.brake();       
   //}



}
