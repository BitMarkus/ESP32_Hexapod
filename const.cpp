#include "const.h"

//////////////
// SONSTIGE //
//////////////

//Pi
const double Pi = 3.14159265359;

///////////////
// GEOMETRIE //
///////////////

//Längenangaben der Beinglieder für alle Beine
const int L_COX = 28;     //Länge Coxae [mm]
const int L_FEM = 55;     //Länge Femur [mm]
const int L_TIB = 80;     //Länge Tibiae [mm]
//Radius der Gelenke Gama (Sechseck) vom Ursprung des Koordinatensystems im Roboterzentrum [mm]
const int R_BASIS = 65;   //[mm]
                            
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
//Kalibrierung am 27.04.2021 bei 6V
const int SERVO_Array[A_BEINE][A_GELENKE][6] =
{
  //Bein 0: vorne links
  { 
    //Gelenk Gamma 1
    {14, 888, 1206, -1, 1000, 2253},
    //Gelenk Alpha 2
    {13, 866, 820, -1, 800, 2044},
    //Gelenk Beta 3
    {12, 896, 1291, 1, 650, 2300}, 
  },
  //Bein 1: vorne rechts
  { 
    {17, 825, 1825, 1, 874, 2029},
    {16, 907, 2096, 1, 860, 2150},
    {15, 856, 1476, -1, 620, 2146}, 
  },
  //Bein 2: mitte links
  { 
    {11, 884, 666, -1, 900, 2282},
    {10, 880, 829, -1, 651, 2120},
    {9, 935, 1282, 1, 650, 2250}, 
  },
  //Bein 3: mitte rechts
  { 
    {8, 844, 2345, 1, 850, 2170},
    {7, 803, 2037, 1, 900, 2150},
    {6, 819, 1635, -1, 750, 2300}, 
  },
  //Bein 4: hinten links
  { 
    {5, 866, 128, -1, 870, 2200},
    {4, 928, 820, -1, 760, 2150},
    {3, 850, 1221, 1, 600, 2180}, 
  },
  //Bein 5: hinten rechts
  { 
    {2, 870, 2961, 1, 920, 2220},
    {1, 908, 2007, 1, 730, 2100},
    {0, 863, 1563, -1, 600, 2190} 
  }
};

//////////////////////
// NEUTRAL-POSITION //
//////////////////////

//Parameter für die Neutralstellung des Roboters
//Vektor für Basis-Offset [mm] 
//0:x, 1:y, 2:z
const Vec3d NTR_V_off = {0, 0, 60.0};
//Array mit den Eulerwinkeln im Bogenmass
//0: Phi (x), 1: Theta (y), 2: Psi (z)
const double NTR_Euler_Ang[3] = {0, 0, 0};
//Standweite = Abstand Gelenk Gamma zum TCP [mm]
const double NTR_dS = 70.0; //70.0

//////////////////////////////
// Bewegungseinschränkungen //
//////////////////////////////

//Kontrolle der Eulerwinkel und des Offsets
//Min/Max-Werte für die Bewegung der Roboterbasis (+/- um die Neutralstellung)
//X-Offset [mm]
const double J_dX_Offs = 30.0; //30.0
//Y-Offset [mm]
const double J_dY_Offs = 30.0; //30.0
//Z-Offset [mm]
//Soll in positive Richtung (nach oben) größer sein als in negative Richtung (Anstoßen der Basis am Boden)
const double J_dZ_Offs_neg = 20.0; //20.0
const double J_dZ_Offs_pos = 40.0; //40.0
//Rotation um die X-Achse (Phi) [Grad]
const double J_dPhi = 10.0; //10.0
//Rotation um die Y-Achse (Theta) [Grad]
const double J_dTheta = 10.0; //10.0
//Rotation um die Z-Achse (Psi) [Grad]
const double J_dPsi = 20.0; //20.0

///////////
// MOTOR //
///////////

//Anzahl Cycles für die Gaits
//0: wave, 1: quadpod, 2: ripple, 3: tripod
const int MOT_Gait_Cycles[A_GAIT] = {6, 4, 3, 2};
//Return Stroke Cycle für jedes Gait für jedes Bein
//0: wave, 1: quadpod, 2: ripple, 3: tripod
const int MOT_Ret_Stroke[A_GAIT][A_BEINE]
{
  //wave gait: Bein 0-5
  {5, 6, 3, 4, 1, 2},
  //quadpod gait
  {1, 3, 4, 2, 1, 3},    
  //ripple gait
  {1, 2, 3, 3, 2, 1},
  //tripod gait
  {1, 2, 2, 1, 1, 2}
  //quadpod gait
};
//Konstante für die Anzahl an Schritten pro Zyklus (Steps per Cycle)
const int MOT_Cycle_Steps = 20; //20
//Konstante für die (mittlere) Schrittlänge [mm]
const double MOT_dX = 60.0; //60.0
//Konstante für die Schritthöhe, bzw. die Auslenkung nach oben [mm]
//gemessen vom z-Wert der Neutralstellung, für jedes Bein gleich
const double MOT_dZ = 30.0; //30.0






