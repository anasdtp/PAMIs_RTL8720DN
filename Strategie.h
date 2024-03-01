#include <Arduino.h>
#include "Capteur.h"
#include "Stepper.h"
#include "WifiPAMIS.h"
#include "ident_club_tech.h"

// #define SIZE_ACTION 10 + 1 + 2       +1 //Depart + Jack + jeu   +1 fin de match

//Phase de jeu : 
//Pour la phase du match choisir une couleur, couleur de base BLEU, mettre les coordonnées comme si on etait en BLEU meme si on est JAUNE
#define BLEU 0
#define JAUNE 1

typedef struct Etape
{
  /* data */
  int ACTION;
  int variable;
  int x,y,theta;// XYT

  int angle; //rotation

  int distance; uint8_t mode; int val_recalage; //Ligne droite ou recalage, Si mode<0 recalage sur y sinon sur x. Si mode = 0 alors ligne droite

}Etape;


extern int COULEUR_STRAT; //Bleu Ou JAUNE, de base il faut faire la strat comme si on etait en bleu et sa s'inverse tout seul
// IMPORTANT!  FAIRE LA STRAT de base EN BLEU

extern Etape strategie_pamis[SIZE_FIFO];
extern bool next_action;
//Cpateur ultrason : -----------------------------------------------------------------------------
extern char type_Evitement; //Mouvement ETAT_GAME_MVT_DANGER evitement sinon non 
extern int target_x,target_y,target_theta;//Pour aller là où on devait aller apres que le robot soit passé

extern volatile bool JACK, WaitForJack;
extern int ROBOT_ID, ACTION_ecriture;

//Construction liste strategie : ------------------------------------------------------

/* [TYPE] [ARG1] [ARG2] [ARG3] [ARG4]*/

/*  J : Jack ;;; Wait for jack
    X : XYT [x];[y];[t];
    L : Ligne droite [d];;;
    R : Rotation [a];;;
    O : Recalage [d];[x ou y?]si x mettre 1, si y -1;[Val recalage] valeur que prendra l'encodeur à la fin;

    A : Action [Type] [ARG1] [ARG2]
*/

// typedef struct CreatStrat{
//     char type;
//     int arg1,arg2,arg3;
// }CreatStrat;
// extern CreatStrat tab_action_brut[SIZE_ACTION];

void initStrategie();
void StrategieLoop();

// void remplirStructStrat();
void WifiProcessRx();