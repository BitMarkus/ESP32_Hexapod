#ifndef BASIS_H
#define BASIS_H

#include "const.h"
#include "vector.h"
#include "matrix.h"

//Anzahl der Beine für die Initialisierung von Arrays
#define A_BEINE 6

////////////////////////
// Klassendeklaration //
////////////////////////

//Klasse für die Basis des Roboters
//Quadrat mit Rotationsachsen von Gelenk Gamma als Eckpunkte
class Basis
{
  //MEMBERVARIABEN

  private:
  
  //Wird innerhalb der Klasse benötigt
  //Array für die Punkte G der Sechseck-Basis mit Zentrum in (0, 0, 0)
  Vec3d P0_G[A_BEINE];
  //Array für die Punkte G der rotierten Basis
  Vec3d P_G[A_BEINE];
  //Normalisierter Vektor, der vom Zentrum der Basis nach RECHTS zeigt und mit der Basis rotiert/translatiert wird
  //Wird als Nullwinkel für die Rotationsachsen Gamma verwendet
  Vec3d Vn_G0r;
  //Normalenvektor der Basisebene (Berechnet für P_G[0])
  //Normalenvektor zeigt in positive z-Richtung (nach oben)
  Vec3d Vn_B;

  //KONSTRUKTOREN
  
  public:

  //Konstruktor ohne Parameter
  Basis();  

  //KLASSENMETHODEN

  //Methode zum Rotieren der Basis um die Eulerwinkel
  //0: Phi, 1: Theta, 2: Psi im Bogenmass
  //Zudem wird die rotierte Basis um das x,y,z-Offset verschoben
  void basisTrans(const double *Euler_Ang, const Vec3d &V_off);
  
  //Getter für die rotierten Pukte P_G
  Vec3d getP_G(int nrleg) const;    
  //Getter für den Nullwinkel für die Rotationsachsen Gamma
  Vec3d getVn_G0r() const; 
  //Getter für den Normalenvektor der Basisebene
  Vec3d getVn_B() const; 
  
  //Methoden zur Ausgabe der Pukte P_G und P0_G
  String printP0_G() const; 
  String printP_G() const;  
};

#endif
