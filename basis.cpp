#include "basis.h"

///////////////////////
// Klassendefinition //
///////////////////////

//KONSTRUKTOREN

//Punkte P0_G im Konstruktor berechnen
Basis::Basis()
{
  P0_G[0] = { (R_BASIS * sin(2*Pi*(1.0/6.0))),  (R_BASIS * cos(2*Pi*(1.0/6.0))), 0 };   //Bein 0: i = 1
  P0_G[1] = { (R_BASIS * sin(2*Pi*(2.0/6.0))),  (R_BASIS * cos(2*Pi*(2.0/6.0))), 0 };   //Bein 1: i = 2
  P0_G[2] = { 0,                                 R_BASIS,                        0 };   //Bein 2: (i = 6)
  P0_G[3] = { 0,                                -R_BASIS,                        0 };   //Bein 3: (i = 3)
  P0_G[4] = { (R_BASIS * sin(2*Pi*(5.0/6.0))),  (R_BASIS * cos(2*Pi*(5.0/6.0))), 0 };   //Bein 4: i = 5 
  P0_G[5] = { (R_BASIS * sin(2*Pi*(4.0/6.0))),  (R_BASIS * cos(2*Pi*(4.0/6.0))), 0 };   //Bein 5: i = 4
} 
  
//KLASSENMETHODEN

//Methode zum Rotieren der Basis um die Eulerwinkel
//0: Phi, 1: Theta, 2: Psi im Bogenmass
//Zudem wird die rotierte Basis um das x,y,z-Offset verschoben
// TO DO: Berechnungen nur ausführen, wenn nötig (z.B. alle Eulerwinkel = 0)
//void Basis::basis_trans(const double Phi, const double Theta, const double Psi, const Vec3d &V_off)
void Basis::basisTrans(const double *Euler_Ang, const Vec3d &V_off)
{
  //Rotationsmatrizen
  //Rotation um positive Winkel in rechtshändigem Koordinatensystem
  //Mx (Phi): Rotation um die x-Achse
  Matr4x4d Mx(1,  0,                   0,                   0,
              0,  cos(Euler_Ang[0]),   -sin(Euler_Ang[0]),  0,
              0,  sin(Euler_Ang[0]),   cos(Euler_Ang[0]),   0,
              0,  0,                   0,                   1);
  //My (Theta): Rotation um die y-Achse
  Matr4x4d My(cos(Euler_Ang[1]),   0,  sin(Euler_Ang[1]),   0,
              0,           1,      0,                       0,
              -sin(Euler_Ang[1]),  0,  cos(Euler_Ang[1]),   0,
              0,                   0,  0,                   1);
  //Mz (Psi): Rotation um die z-Achse
  Matr4x4d Mz(cos(Euler_Ang[2]),   -sin(Euler_Ang[2]),  0,  0,
              sin(Euler_Ang[2]),   cos(Euler_Ang[2]),   0,  0,
              0,                   0,                   1,  0,
              0,                   0,                   0,  1);
  //Matrix-Multiplikation für Rotation um die drei körperfesten Raumachsen
  //Nur die erste Achse ist raumfest, die anderen Achsen entstehen bei der Rotation des Körpers
  Matr4x4d Mxyz = Mx * My * Mz;
  //Diese Version rotiert 3x um die raumfesten Achsen
  //Matr4x4d Mxyz = Mz * My * Mx;

  //Punkte P_G0 um xyz-Achse rotieren
  for(int i = 0; i < A_BEINE; i++)
  {
    Mxyz.matr_x_punkt(P0_G[i], P_G[i]);
  }

  //Rotierte Punkte um das x,y,z-Offset verschieben
  for(int i = 0; i < A_BEINE; i++)
  {
    P_G[i] = P_G[i] + V_off;
  }
  
  //Normalisierter Vektor, der vom Zentrum der Basis nach RECHTS zeigt und mit der Basis rotiert/translatiert wird
  //Wird als Nullwinkel für die Rotationsachsen Gamma verwendet
  //Verwendet wird ein Vektor von Punkt P_G[0] nach Punkt P_G[1] (parallel zur Vorderseite der Roboterbasis)
  Vec3d V_G0r = P_G[1] - P_G[0];
  Vn_G0r = V_G0r.normalisieren_neu();
  
  //Normalenvektor der Basisebene (Berechnet für P_G[0])
  //Normalenvektor zeigt in positive z-Richtung (nach oben)
  //Spannvektoren: Vektor P_G[0] - P_G[1] und Vektor P_G[0] - P_G[2]
  Vec3d V21 = P_G[0] - P_G[1];
  Vec3d V31 = P_G[0] - P_G[2];
  Vec3d V_B = V21.vektorprod(V31); 
  Vn_B = V_B.normalisieren_neu(); 
}

//Getter für die rotierten Pukte P_G
Vec3d Basis::getP_G(int nrleg) const
{
  return this->P_G[nrleg];
} 

//Getter für den Nullwinkel für die Rotationsachsen Gamma
Vec3d Basis::getVn_G0r() const
{
  return this->Vn_G0r;
}

//Getter für den Normalenvektor der Basisebene
Vec3d Basis::getVn_B() const
{
  return this->Vn_B;
} 

//Methode zur Ausgabe der Pukte P0_G
String Basis::printP0_G() const
{
  String strg_ausgabe = String("\nRoboterbasis (Zentrum in 0,0,0): P0_G:\n");
  String strg1 = String(", ");
  
  for(int i = 0; i < A_BEINE; i++)
  {  
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": (";
    strg_ausgabe += P0_G[i][0];
    strg_ausgabe += strg1;
    strg_ausgabe += P0_G[i][1];
    strg_ausgabe += strg1;
    strg_ausgabe += P0_G[i][2];
    strg_ausgabe += ")\n";
  }
  
  return strg_ausgabe;
} 
//Methode zur Ausgabe der Pukte P_G
String Basis::printP_G() const
{
  String strg_ausgabe = String("\nRoboterbasis (nach Translation): P_G:\n");
  String strg1 = String(", ");
  
  for(int i = 0; i < A_BEINE; i++)
  {  
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": (";
    strg_ausgabe += P_G[i][0];
    strg_ausgabe += strg1;
    strg_ausgabe += P_G[i][1];
    strg_ausgabe += strg1;
    strg_ausgabe += P_G[i][2];
    strg_ausgabe += ")\n";
  }
  
  return strg_ausgabe;
}
 
