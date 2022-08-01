
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "Math.h"
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 34, TXPin = 0;//5,18成功。RXTXピンである必要はない。
static const uint32_t GPSBaud = 9600;
double delta_x, delta_y, now_x, now_y, sita, goal_y_rad,temp1,temp2;
double r; 
double GPSsita; 
const double R = 6371000.0;  

//const double goal_x = 139.460884;             //ゴールの経度(初期値は適当,35.657941,139.542798
//const double goal_y = 35.717222; 

const double goal_x = 139.54423149879;             //ゴールの経
const double goal_y = 35.65724408289;





// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


//////////////割り込み処理
hw_timer_t *timer1=NULL;//タイマー管理用の構造体
//votaile SemaphoreHandle_t timerSemaphore;//タイマー割り込みが発生したかどうかを表すセマフォ
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;//排他制御を行うための変数

volatile uint32_t timerCounter1 =0;//タイマー割込みが発生した回数を保存する変数。

void IRAM_ATTR onTimer1(){//割り込みが発生したときに呼び出される関数
  portENTER_CRITICAL_ISR(&timerMux);//排他制御開始
  timerCounter1++;//タイムカウンタを1
  portEXIT_CRITICAL_ISR(&timerMux);//排他制御終了

}


void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);//softwareserial専用のbaud

  //割り込み関係
  timer1=timerBegin(0,80,true);//タイマの初期設定、タイマー番号0、分周比80
  timerAttachInterrupt(timer1, &onTimer1, true);//タイマに割り込みハンドラを設定
  timerAlarmWrite(timer1, 1000000, true);//タイマに割り込みのタイミングを設定クロック周波数80MHz
  timerAlarmEnable(timer1);//タイマー開始
  Serial.println("Start!");
  
  
  

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); 
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop()
{

   if(timerCounter1>=0){
    portENTER_CRITICAL(&timerMux);
    timerCounter1++;
    portEXIT_CRITICAL(&timerMux);
    GPSvalue(); 
    
   }

   now_x=gps.location.lng();
   now_y=gps.location.lat();

   delta_x = goal_x - now_x;           //ゴールまでのx,y座標の差分を読み込む
   delta_y = goal_y - now_y;

   r = R * acos(sin(goal_y*3.14/180) * sin(now_y*3.14/180)+cos(now_y*3.14/180) * cos(goal_y*3.14/180)*cos((goal_x - now_x)*3.14/180));     

   r=abs(r);
   //Serial.println(r);

   //方角
   delta_x = delta_x * 3.14/180;
   now_x = now_x * 3.14/180;
   goal_y_rad = goal_y * 3.14/180;
   now_y = now_y * 3.14/180;

   temp1 = cos(now_y)*tan(goal_y_rad);
   temp2 = sin(now_y)*cos(delta_x);

   GPSsita = atan2(sin(delta_x),(temp1-temp2));
   GPSsita = GPSsita * 180/3.14;  

   Serial.println(GPSsita);
   

 

  
  

  

 
}

void GPSvalue()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)//GPSが動作可能だったら続ける
    if (gps.encode(ss.read()))//GPSのデータをエンコードする
      displayInfo();//情報を出力する。

  if (millis() > 5000 && gps.charsProcessed() < 10)//通信不可能
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
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
