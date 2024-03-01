#include "Stepper.h"
#include "Capteur.h"


// The X Stepper pins
#define STEPPER1_STEP_PIN PA27
#define STEPPER1_DIR_PIN PA30
// The Y stepper pins
#define STEPPER2_STEP_PIN PA15
#define STEPPER2_DIR_PIN PA26


#define MAX_SPEED 500.0
#define ACCELERATION 750.0

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);


double rayonRoue = RAYON_ROBOT, rayonRobot = LARGEUR_ROBOT;

//         /*(STEPpin, DIRpin, MOpin)*/
// STEPPER stepper1(PA27, PA30,0);
// STEPPER stepper2(PA26, PA15, 0);


void initDriver(){
  // pinMode(5, OUTPUT); pinMode(23, OUTPUT);
  // digitalWrite(5,HIGH); digitalWrite(23,HIGH); //micro step à 1/2
  rayonRoue = RAYON_ROBOT; rayonRobot = LARGEUR_ROBOT;
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(ACCELERATION);
}

void run(int position, AccelStepper::MOUVEMENT mvt){
  static int delta_distance = 0, x_origin = 0, y_origin = 0; //static double degreeParStep = DEGREPARSTEP;//Pour qu'il soit creé qu'une fois
  //uStep, 1/2 steps
  stepper1.setMouvement(mvt);

  stepper1.move(position);
  x_origin = x_robot;
  y_origin = y_robot;
  while (stepper1.run()){
       if(mvt != AccelStepper::MOUVEMENT::ROTATION){
          delta_distance = round(stepper1.currentPosition()*DEGREPARSTEP/(180.0/(rayonRoue*PI)));
          x_robot = x_origin + round(delta_distance * cos(theta_robot * PI / 1800.0));
          y_robot = y_origin + round(delta_distance * sin(theta_robot * PI / 1800.0));
        // Serial.printf("x_robot : %lf, y_robot : %lf, delta_distance : %d, cur : %d\n", x_robot, y_robot,delta_distance, stepper1.currentPosition());

      if(etatMoustaches()){
        stepper1.stop();
        stepper1.setCurrentPosition(0);//Remise à zéros pour les prochains
        break;
      }
    }
        // YIELD; // Let system housekeeping occur
  }
}

void rotation_robot_absolue(int degree){//en dizieme de degree 360° -> 3600 dizieme de degree
  //distance_totale = degree * PI/ 180 * rayonRoue;
  //Rotation d'une roue = distance_totale/rayonRoue * 360;
  //Step = degreeParStep * Rotation d'une roue ;
  //double degreeParStep = DEGREPARSTEP;
  //      m0 == 0 && m1 == 0 && m2 == 0
  //if     (stepper1.m0 == 1){degreeParStep /= 2.0;}//On verifie que pour un stepper car normalement c'est le meme pour les deux
  if (degree > 1800) {
        degree -= 3600; // Effectuer une rotation dans le sens opposé
  }else if(degree < -1800){
    degree += 3600;
  }
  
  double distance_totale = degree * PI/ 1800. * rayonRoue;
  int Step = (distance_totale*180.0/(rayonRoue*PI))/DEGREPARSTEP;
  //bothStepper((int)Step, dir, MOUVEMENT_ROTATION,1);
  
  stepper1.runToNewPosition(Step, stepper1.ROTATION);


  theta_robot += degree;
  if (theta_robot > 1800.0) {
        theta_robot -= 3600.0; 
  }else if(theta_robot < -1800.0){
    theta_robot += 3600.0;
  }

  stepper1.setCurrentPosition(0);
}//*/

// Fonction de rotation relative
void rotation_robot_relative(int degree) {//en dizieme de degree
  if(degree == theta_robot){return;}

    int degreeAparcourir = degree - theta_robot;
    // Appelez la fonction de rotation absolue avec le nouvel angle
    rotation_robot_absolue(degreeAparcourir);
    theta_robot = degree;
}



void Ligne_droite(int distance) {//En mm
    // static double accel = 0.2, ta = 0, tc = 0, td = 0;//m/s² //tc temps à vitesse constante, ta en acceleration, td en deceleration en ms
    // Calculate the number of steps required to move the given distance
    // Assuming DEGREEPARSTEP is the number of degrees moved per step
    // static double degreeParStep = DEGREPARSTEP;
    //if     (stepper1.m0 == 1){degreeParStep /= 2.0;}//On verifie que pour un stepper car normalement c'est le meme pour les deux

    int steps =  (distance*180.0/(rayonRoue*PI))/DEGREPARSTEP;
    
    run(steps, stepper1.LIGNE_DROITE);
    // stepper1.runToNewPosition(steps, stepper1.LIGNE_DROITE);
    // x_robot = x_robot + distance * cos(theta_robot * PI / 1800.0);
    // y_robot = y_robot + distance * sin(theta_robot * PI / 1800.0);

    stepper1.setCurrentPosition(0);
}

void Ligne_droite_sans_moustaches(int distance) {//En mm
    // static double accel = 0.2, ta = 0, tc = 0, td = 0;//m/s² //tc temps à vitesse constante, ta en acceleration, td en deceleration en ms
    // Calculate the number of steps required to move the given distance
    // Assuming DEGREEPARSTEP is the number of degrees moved per step
    // static double degreeParStep = DEGREPARSTEP;
    //if     (stepper1.m0 == 1){degreeParStep /= 2.0;}//On verifie que pour un stepper car normalement c'est le meme pour les deux

    int steps =  (distance*180.0/(rayonRoue*PI))/DEGREPARSTEP;
    
    // run(steps, stepper1.LIGNE_DROITE);
    stepper1.runToNewPosition(steps, stepper1.LIGNE_DROITE);
    x_robot = x_robot + distance * cos(theta_robot * PI / 1800.0);
    y_robot = y_robot + distance * sin(theta_robot * PI / 1800.0);

    stepper1.setCurrentPosition(0);
}

void XYT(int px, int py, int ptheta){
  static int dist = 0, ang1 = 0, ang2 = 0;


  // Son hypothénuse correspond à la distance à parcourir
  dist = sqrt((px - x_robot)*(px - x_robot)+(py - y_robot)*(py - y_robot));

  // la 1ere rotation correspond à l'angle du triangle, moins l'angle de la position de départ
  // C'est-à-dire la tangente du côté opposé sur l'angle adjacent
  // La fonction atan2 fait le calcul de la tangente en radians, entre Pi et -Pi
  // On rajoute des coefficients pour passer en degrés
  // On ajoute 7200 dixièmes de degrés pour être sûrs que le résultat soit positif
  // ang1 = (short)((atan2((double)(py - y_robot), (double)(px - x_robot)) * 1800 / PI) - theta_robot + 7200) % 3600;

  if((((py-y_robot)!=0)&&((px-x_robot)!=0))||((px-x_robot)!=0)){
    ang1 = (int)((atan2((double)(py - y_robot), (double)(px - x_robot)) * 1800 / PI) - theta_robot + 7200) % 3600;
  }   
  // On passe le résultat entre -1800 et 1800
  if(ang1 > 1800) {ang1 = (ang1 - 3600);}

  // La 2è rotation correspond à l'angle de destination, moins l'angle à la fin de la ligne droite,
  // donc le même qu'à la fin de la 1ère rotation, donc l'angle de départ plus la première rotation
  // On ajoute 3600 pour être sûr d'avoir un résultat positif
  ang2 = (int)(ptheta - ang1 - theta_robot + 3600) % 3600;
                    
  // On passe le résultat entre -1800 et 1800
  if(ang2 > 1800) ang2 = (ang2 - 3600);
  // //Serial.printf("X_Y_Theta : Dist : %f, ang1 : %hd, ang2 : %hd       ;       ", dist, ang1, ang2); 

  //ROTATION_X_Y_THETA_1 :
  rotation_robot_absolue(ang1);
  //Serial.printf("rotation_robot_absolue(ang1 : %lf);       ", ang1); 
  delay(150);
  // LIGNE_DROITE_X_Y_THETA :
  Ligne_droite(dist);
  //Serial.printf("Ligne_droite(dist : %lf);       ", dist); 
  delay(150);
  // ROTATION_X_Y_THETA_2 : 
  rotation_robot_absolue(ang2);
  //Serial.printf("rotation_robot_absolue(ang2 : %lf);\n", ang2); 

}


void recalage(int distance, short mode, int valRecalage)
{
    stepper1.move(distance);//relative
    
    while (stepper1.run()){
      if (etatMoustaches()==2){//Capteur interrupteur, alors on a bien touché le rebord, recalage fait
        stepper1.stop();

        if(mode == 1){//sur x
        x_robot = valRecalage;
                        // On met l'angle à jour en fonction de la position sur le terrain
                        // Si on est dans la partie haute ( > la moitié), on est dans un sens,
                        // Si on est dans la partie basse, on met à jour dans l'autre sens
                        // On prend aussi en compte le sens dans lequel on a fait la ligne droite
        if (valRecalage > 1000){
          if (distance >= 0){theta_robot = 0;}
          else{theta_robot = 1800;}
        }
        else{
          if (distance >= 0){theta_robot = 1800;}
          else{theta_robot = 0;}
        }
      }
      else{//sur y
        y_robot = valRecalage;

        if (valRecalage > 1500){
          if (distance >= 0){theta_robot = 900;}
          else{theta_robot = -900;}
        }
        else{
          if (distance >= 0){theta_robot = -900;}
          else{theta_robot = 900;}
        }
      }
      break;
      }
      YIELD; // Let system housekeeping occur
    }
}

