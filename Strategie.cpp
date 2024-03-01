#include "Strategie.h"

//Jack = depart Match

int ROBOT_ID = 0;
int COULEUR_STRAT = BLEU;

Etape strategie_pamis[SIZE_FIFO];//Faire la fonction qui interprete tab_action_brut. void initStrategie();

int ACTION_ecriture = 0;

bool next_action = true;

//Cpateur ultrason : -----------------------------------------------------------------------------
char type_Evitement = ETAT_GAME_PAS_DANGER; //Mouvement ETAT_GAME_MVT_DANGER evitement sinon non 
int target_x,target_y,target_theta;//Pour aller là où on devait aller apres que le robot soit passé

// CreatStrat tab_action_brut[SIZE_ACTION]={
//     //Phase d'initialisation (pendant les 3 min) :
//     // {'O', 1000,1,MOITIEE_ROBOT},//On avance doucement jusqu'a se plaquer contre le mur sur l'axe des x
//     // {'L', -150,1,0},//On recule sans prendre en compte les moustaches
//     // {'X', 150,1480,0},//Preparation garage debut match
//     // {'O', 1000,1,MOITIEE_ROBOT},//On se plaque contre le mur
//     // {'O', 1000,1,MOITIEE_ROBOT},//On avance doucement jusqu'a se plaquer contre le mur sur l'axe des x


//     {'L', 1000,0,0},
//     // {'L', 1550,0,0},
//     // {'R', 900,0,0},//On recule

//     {'J', 0,0,0},//Attente du jack
//     //Debut du match :
//     {'X', 250,450,-900},
//     // {'O', 1000,-1,MOITIEE_ROBOT},//On avance doucement jusqu'a touché soit le mur soit une plante
//     // {'X', 250,0,-900},
//     {'L', 450,0,0},
//     {'L', 10,1,0},

//     {'0', 0,0,0}//Fin de match
// };


void initStrategie(){
    // remplirStructStrat();
}

void StrategieLoop(){
    static int ETAPE_lecture=0,ETAPE_occupation=0,ETAPE_max_occupation=0;//, etat_evitement = 3;

    

    // if(WaitForJack){//On attend alors le debut du match
    //     if(JACK){
    //         Serial.println("Game start");
    //         JACK = false;
    //         WaitForJack = false;
    //         next_action = true;
    //     }
    //     else{
    //         return;
    //     }
    // }

    ETAPE_occupation=ACTION_ecriture-ETAPE_lecture;
    if(ETAPE_occupation<0){ETAPE_occupation=ETAPE_occupation+SIZE_FIFO;}
    if(ETAPE_max_occupation<ETAPE_occupation){ETAPE_max_occupation=ETAPE_occupation;}//Ajouter des conditions : attendre que l'action est été faite
    
  
    if(!next_action || WaitForJack){return;}
    next_action = false;
    Serial.print(ETAPE_lecture);  Serial.print(" - Action : ");  Serial.print(strategie_pamis[ETAPE_lecture].ACTION); 

    switch (strategie_pamis[ETAPE_lecture].ACTION)
    {
            case WAIT_FOR_JACK ://Debut du Match dés qu'on tire le Jack
                WaitForJack = true;
                //etat_evitement = 0;//On active le lidar durant le match
                Serial.println("WAIT_FOR_JACK");
            break;

            case ID_PAMIS_LIGNE_DROITE:
            {
                Serial.println(" - ID_PAMIS_LIGNE_DROITE");
                if (strategie_pamis[ETAPE_lecture].mode){
                    Ligne_droite_sans_moustaches(strategie_pamis[ETAPE_lecture].distance);
                }
                else{
                    Ligne_droite(strategie_pamis[ETAPE_lecture].distance);
                }
                
              next_action = true;
            }
            break;
            
            case ID_PAMIS_ROTATION://Absolue
            {
                Serial.println(" - ID_PAMIS_ROTATION");
              rotation_robot_absolue(strategie_pamis[ETAPE_lecture].angle);

              next_action = true;
            }
            break;
            case ID_PAMIS_XYT:
            {
                Serial.println(" - ID_PAMIS_XYT");
              int x = strategie_pamis[ETAPE_lecture].x;
              int y = strategie_pamis[ETAPE_lecture].y;
              int theta = strategie_pamis[ETAPE_lecture].theta;

              XYT(x, y, theta);
              
              next_action = true;
            }
            break;
            case ID_PAMIS_RECALAGE:
            {
                Serial.println(" - ID_PAMIS_RECALAGE");
              int distance = strategie_pamis[ETAPE_lecture].distance;
              uint8_t mode = strategie_pamis[ETAPE_lecture].mode;// si mode == 1, recalage sur x, sinon recalage sur y
              int valRecalage = strategie_pamis[ETAPE_lecture].val_recalage;//Valeur pris au final

              recalage(distance, mode, valRecalage);
              next_action = true;
            }
              
            // case ODOMETRIE_SMALL_POSITION:
            // {
            //     Odo_x = (strategie_pamis[ETAPE_lecture].dt[1] << 8) | strategie_pamis[ETAPE_lecture].dt[0];
            //     Odo_y = (strategie_pamis[ETAPE_lecture].dt[3] << 8) | strategie_pamis[ETAPE_lecture].dt[2];
            //     Odo_theta = (strategie_pamis[ETAPE_lecture].dt[5] << 8) | strategie_pamis[ETAPE_lecture].dt[4];
            // }
            // break;
           
            
            default :
            break;
    }
    // ETAPE_lecture ++;
    // ETAPE_lecture %= SIZE_ACTION;
    ETAPE_lecture=(ETAPE_lecture+1)%SIZE_FIFO;
    // Serial.println(ETAPE_lecture);
}   


bool machineEtatWifiStrategie(Message msg)
{
    bool output = false;
    switch (msg.order)
    {
    case ID_PAMIS_LIGNE_DROITE:
    {
        uint8_t mode = msg.arg2;
        int distance = msg.arg1;

        Serial.print(" - ID_PAMIS_LIGNE_DROITE : ");
        Serial.println(distance);
        // if (mode){Ligne_droite_sans_moustaches(distance);}
        // else{Ligne_droite(distance);}
        strategie_pamis[ACTION_ecriture].ACTION = ID_PAMIS_LIGNE_DROITE;
        strategie_pamis[ACTION_ecriture].distance = distance;
        strategie_pamis[ACTION_ecriture].mode = mode;

        ACTION_ecriture =(ACTION_ecriture+1)%SIZE_FIFO;
        next_action = true;
        output = true;
    }
    break;
    case ID_PAMIS_ROTATION: // Absolue
    {
        int angle = msg.arg1; // dixieme de degree

        Serial.print(" - ID_PAMIS_ROTATION  : ");
        Serial.println(angle);
        // rotation_robot_absolue(angle);
        strategie_pamis[ACTION_ecriture].ACTION = ID_PAMIS_ROTATION;
        if(COULEUR_STRAT == JAUNE){//Alors on inverse
            strategie_pamis[ACTION_ecriture].angle *= -1;
        }

        strategie_pamis[ACTION_ecriture].angle = angle;

        ACTION_ecriture =(ACTION_ecriture+1)%SIZE_FIFO;
        next_action = true;
        output = true;
    }
    break;
    case ID_PAMIS_XYT:
    {
        int x = msg.arg1;
        int y = msg.arg2;
        int theta = msg.arg3;
        if(COULEUR_STRAT == JAUNE){
            x     = x;
            y     = 3000 - y;
            theta = -theta;
        }
        Serial.print(" - ID_PAMIS_XYT xyt :");
        Serial.print(x);
        Serial.print(y);
        Serial.println(theta);
        // XYT(x, y, theta);
        strategie_pamis[ACTION_ecriture].ACTION = ID_PAMIS_XYT;
        strategie_pamis[ACTION_ecriture].x     = x;
        strategie_pamis[ACTION_ecriture].y     = y;
        strategie_pamis[ACTION_ecriture].theta = theta;

        ACTION_ecriture =(ACTION_ecriture+1)%SIZE_FIFO;
        next_action = true;
        output = true;
    }
    break;
    case ID_PAMIS_RECALAGE:
    {
        int distance = msg.arg1;
        uint8_t mode = msg.arg2;    // si mode == 1, recalage sur x, sinon recalage sur y
        int valRecalage = msg.arg3; // Valeur pris au final

        Serial.print(" - ID_PAMIS_RECALAGE : ");
        Serial.println(distance);
        // recalage(distance, mode, valRecalage);
        strategie_pamis[ACTION_ecriture].ACTION = ID_PAMIS_LIGNE_DROITE;
        strategie_pamis[ACTION_ecriture].distance = distance;
        strategie_pamis[ACTION_ecriture].mode = mode;
        strategie_pamis[ACTION_ecriture].val_recalage = valRecalage;

        ACTION_ecriture =(ACTION_ecriture+1)%SIZE_FIFO;
        next_action = true;
        output = true;
    }
    break;

    case ID_PAMIS_ODO: // Changement de la position du Pamis par le Bt
    {
        int x = msg.arg1;
        int y = msg.arg2;
        int theta = msg.arg3;

        if(COULEUR_STRAT == JAUNE){
            x     = x;
            y     = 3000 - y;
            theta = -theta;
        }

        x_robot = x;
        y_robot = y;
        theta_robot = theta;
        Serial.print(" - ID_PAMIS_ODO : xyt :");
        Serial.print(x);
        Serial.print(y);
        Serial.println(theta);
        output = true;
    }
    break;

    default:
        break;
    }
    return output;
}

bool machineEtatWifiReglage(Message msg)
{
    bool output = false;
    switch (msg.order)
    {
    case ID_DEPART_PAMIS:
    {
        // Lets go
        WaitForJack = false;
        JACK = true;
        next_action = true;
        Serial.println(" -JACK Lets go");
        output = true;
    }
    break;

    case WAIT_FOR_JACK:
    {
        // Lets go
        WaitForJack = true;
        next_action = false;
        Serial.println(" -WAIT_FOR_JACK");
        output = true;
    }
    break;

    case ID_REGLAGE_COEF: // Changement de la position du Pamis par le Bt
    {
        int largeurRobot = msg.arg1; // En
        int rayon_roue = msg.arg2;

        rayonRobot = (largeurRobot / 10.0) / 2.0;
        rayonRoue = rayon_roue / 10.0;

        Serial.print(" - ID_REGLAGE_COEF : val :");
        Serial.print(rayonRobot);
        Serial.println(rayonRoue);
        output = true;
    }
    break;

    case ID_REGLAGE_ROBOT_ID: // Changement de la position du Pamis par le Bt
    {
        int robotId = msg.arg1;

        ROBOT_ID = robotId;

        Serial.print(" - ID_REGLAGE_ROBOT_ID : ROBOT_ID :");
        Serial.println(ROBOT_ID);
        output = true;
    }
    break;

    case ID_REGLAGE_COULEUR: // Changement de la position du Pamis par le Bt
    {
        int couleur = msg.arg1;

        COULEUR_STRAT = couleur? JAUNE : BLEU;

        Serial.print(" - ID_REGLAGE_ROBOT_ID : ROBOT_ID :");
        Serial.println(ROBOT_ID);
        output = true;
    }
    break;


    default:
        break;
    }
    return output;
}


void WifiProcessRx()
{
    static signed char FIFO_lecture = 0, FIFO_occupation = 0, FIFO_max_occupation = 0;

    FIFO_occupation = FIFO_ecriture - FIFO_lecture;
    if(FIFO_occupation<0){FIFO_occupation=FIFO_occupation+SIZE_FIFO;}
    if(FIFO_max_occupation<FIFO_occupation){FIFO_max_occupation=FIFO_occupation;}
    if(!FIFO_occupation){return;}

    if(rxMsg[FIFO_lecture].robot_id == ROBOT_ID || rxMsg[FIFO_lecture].robot_id == 0){//ID = 0, parle à tous le monde
        Serial.println("*************************************************");
        Serial.print(FIFO_lecture);
        Serial.print(" - WIFI Action : ");
        Serial.print(strategie_pamis[FIFO_lecture].ACTION);

        machineEtatWifiStrategie(rxMsg[FIFO_lecture]);
        machineEtatWifiReglage(rxMsg[FIFO_lecture]);

        Serial.println("*************************************************");
    }
    

    FIFO_lecture = (FIFO_lecture + 1) % SIZE_FIFO;
    
}