#ifndef CONST_H
#define CONST_H

#include "vector.h"

//Anzahl der Beine und der Gelenke pro Bein
//für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3
#define A_GAIT 4

//////////////
// SONSTIGE //
//////////////

//Pi
extern const double Pi;

///////////////
// GEOMETRIE //
///////////////

//Längenangaben der Beinglieder
extern const int L_COX;     //Länge Coxae [mm]
extern const int L_FEM;     //Länge Femur [mm]
extern const int L_TIB;     //Länge Tibiae [mm]
//Radius der Gelenke Gama (Sechseck) vom Ursprung des Koordinatensystems in Roboterzentrum [mm]
extern const int R_BASIS;   

////////////
// SERVOS //
////////////

//Array mit den Servovariablen erzeugen
//0: Servo-Pin an Polulu Maestro Serveo Treiber (18 Kanäle)
//1: dP_90
//2: P_90
//3: Servorichtung
//4: P_Min
//5: P_Max 
extern const int SERVO_Array[A_BEINE][A_GELENKE][6];

//////////////////////
// NEUTRAL-POSITION //
//////////////////////

//Parameter für die Neutral-Position des Roboters
//Vektor für Basis-Offset [mm] 
//0:x, 1:y, 2:z
extern const Vec3d NTR_V_off;  
//Array mit den Eulerwinkeln im Bogenmass
//0: Phi (x), 1: Theta (y), 2: Psi (z)
extern const double NTR_Euler_Ang[3];
//Standweite = Abstand Gelenk Gamma zum TCP [mm]
extern const double NTR_dS;

//////////////////////////////
// Bewegungseinschränkungen //
//////////////////////////////

//Kontrolle der Eulerwinkel und des Offsets
//Min/Max-Werte für die Bewegung der Roboterbasis (+/- um die Neutralstellung)
//X-Offset [mm]
extern const double J_dX_Offs;
//Y-Offset [mm]
extern const double J_dY_Offs;
//Z-Offset (z_offs_ntr) [mm]
extern const double J_dZ_Offs_neg;
extern const double J_dZ_Offs_pos;
//Rotation um die X-Achse (Phi) [Grad]
extern const double J_dPhi;
//Rotation um die Y-Achse (Theta) [Grad]
extern const double J_dTheta;
//Rotation um die Z-Achse (Psi) [Grad]
extern const double J_dPsi;

///////////
// MOTOR //
///////////

//Anzahl Cycles für die Gaits
//0: wave, 1: ripple, 2: tripod
extern const int MOT_Gait_Cycles[A_GAIT];
//Return Stroke Cycle für jedes Gait für jedes Bein
//0: wave, 1: ripple, 2: tripod
extern const int MOT_Ret_Stroke[A_GAIT][A_BEINE];
//Konstante für die Anzahl an Schritten pro Zyklus (Steps per Cycle)
extern const int MOT_Cycle_Steps;
//Konstante für die Schrittlänge [mm]
extern const double MOT_dX;
//Konstante für die Schritthöhe [mm]
extern const double MOT_dZ;

#endif







