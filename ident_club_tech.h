/*Fichiers regroupant la totalité des IDs que ce soit pour le bus CAN ou pour le BLE, c'est le meme protocole*/

//IDs pour les PAMIS pour le control par wifi, par page web:
#define ID_PAMIS_ROTATION       1
#define ID_PAMIS_LIGNE_DROITE   2
#define ID_PAMIS_XYT            3
#define ID_PAMIS_RECALAGE       4 
#define ID_PAMIS_ODO            5
#define ID_DEPART_PAMIS         6 //Jack
#define ID_REGLAGE_COEF         7
#define WAIT_FOR_JACK           8
#define ID_REGLAGE_ROBOT_ID     9
#define ID_REGLAGE_COULEUR      10
/*
Le recalage sera fait grace aux deux moustaches/interrupteurs devant les PAMIS
Pour le y, le PAMIS sera poser le precisement possible à la main sur une ligne
Pour le x, Le Pamis s'avancera jusqu'a que les deux interupteurs touchent le mur
L'angle initiale sera donc de -90°
*/


#define ESP32_RESTART 0x34

