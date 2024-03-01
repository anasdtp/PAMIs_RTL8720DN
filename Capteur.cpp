#include "Arduino.h"
#include "Capteur.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Stepper.h"
#include "math.h"

//Interrupteur, fdc, appellé moustaches, il y en a deux
const int PinMoustache1 = PA14, PinMoustache2 = PA13;

Ultrasonic ultrasonic(PB3);

volatile bool DANGER = false;

uint8_t etatMoustaches(){
    static bool both, one, MSTH1, MSTH2;
    MSTH1 = digitalRead(PinMoustache1); MSTH2 = digitalRead(PinMoustache2);
    // Serial.printf("MSTH1 : %d MSTH2 : %d\n", MSTH1, MSTH2);
    one = (MSTH1 || MSTH2 ? true : false);
    both= (MSTH1 && MSTH2 ? true : false);
    
    return (both ? 2 : one);
}

void Ultrason_task(void *pvParameters){
    (void) pvParameters;
    static const int DistanceEntreCapteurMoitiee = 30, tempsEchant = 50;//mm
    static int detection = 0,n=0, distance = 0;//,x_obs,y_obs;
    uint32_t start_time = 0;
    
    while(1){
        start_time = millis();

        distance = ultrasonic.read()*10 + DistanceEntreCapteurMoitiee; // two measurements should keep an interval
        
        // x_obs = distance * cos(theta_robot*PI/1800.) + x_robot;
        // y_obs = distance * sin(theta_robot*PI/1800.) + y_robot;

        // Serial.printf("distance %d mm ; x_obs %d ; y_obs %d\n",distance, x_obs,y_obs);//0~4000mm
        
        if(distance<250){detection+=1;}

        n+=1;
        if (n>30){ n=0; detection=0; DANGER = false;}
        if (detection>=2){DANGER = true; Serial.println("DANGER");}
        else{DANGER = false;}

        if((millis()-start_time)>tempsEchant){Serial.println("Erreur erreur temp calcul Capteur ultrason");}
        else{while((millis()-start_time)<tempsEchant);}
    }
}

bool UltrasonStatus(){
  return DANGER;
}

void init_Ultrason_task(){
    xTaskCreate(Ultrason_task, "Ultrason_task", 2048, NULL, tskIDLE_PRIORITY+2, NULL);
    
} 

void init_Capteurs(){
    pinMode(PinMoustache1, INPUT_PULLDOWN);//Pins qui doivent etre libre lors du boot
    pinMode(PinMoustache2, INPUT_PULLDOWN);

    //init_Ultrason_task();
    delay(100);
}
