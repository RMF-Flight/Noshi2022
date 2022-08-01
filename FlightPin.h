

#include <Arduino.h>

static const int flightPin = 35; 
int val = 0; 




void flight_loop() {

  while(analogRead(flightPin)){
    Serial.println("Connecting");
    delay(100);
    //if(GPSflag=true){
      //GPSvalue();
      //GPSflag=false;  
    
  //}
   
    
   

  }
  
Serial.println("hatched");  

     


}
