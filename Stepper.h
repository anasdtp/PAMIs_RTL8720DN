#include <Arduino.h>
#include "AccelStepper.h"

#define DEGREPARSTEP 1.8
#define LARGEUR_ROBOT 43.4 //mm
#define RAYON_ROBOT 29. //mm

#define MOITIEE_ROBOT 45 //mm

// extern STEPPER stepper1, stepper2;
extern volatile int x_robot, y_robot, theta_robot;
extern double rayonRoue, rayonRobot;//En variable pour qu'elle soit reglable via le Wifi

void initDriver();
void run(int position, AccelStepper::MOUVEMENT mvt);
void rotation_robot_absolue(int degree);
void rotation_robot_relative(int degree);
void Ligne_droite(int distance);
void Ligne_droite_sans_moustaches(int distance);
void XYT(int px, int py, int ptheta);
void recalage(int distance, short mode, int valRecalage);