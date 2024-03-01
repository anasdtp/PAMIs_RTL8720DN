#include "Strategie.h"

volatile bool JACK = false, WaitForJack = false;
volatile int x_robot = 0, y_robot = 0, theta_robot = 0;//degree en dizieme de degree, distance en mm

void setup()
{
  Serial.begin(115200);
                   Serial.println("init_Capteurs();              begin");
  init_Capteurs(); Serial.println("init_Capteurs();              begin");


                Serial.println("initDriver();              begin");
  initDriver(); Serial.println("initDriver();              ended");

  // initStrategie();

                Serial.println("setup_Wifi();              begin");
  setup_Wifi(); Serial.println("setup_Wifi();              ended");

  Serial.println("void setup()              ended");
  delay(1000);
}

void loop(){
  WifiLoop();
  
  WifiProcessRx();

  StrategieLoop();  
}