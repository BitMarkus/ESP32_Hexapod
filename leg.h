#ifndef LEG_H
#define LEG_H

#include "functions.h"
#include <math.h>
#include "vector.h"
#include "matrix.h"
#include "basis.h"

//Anzahl der Beine für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3

////////////////////////
// Klassendeklaration //
////////////////////////

//Klasse für die Beine des Roboters
class Leg
{
  //MEMBERVARIABEN
  
  private:

  //Werden beim Start initialisiert
  //Beinnummer
  int Nr_Leg;  
  
  //Ausgabe der Kasse
  //Beinwinkel werden im Bogenmass in einem Array gespeichert  
  //0:Gamma: Hüftwinkel für Horizontalrotation des Beins
  //positive Winkel führen das Bein nach vorne (positives x)
  //Gamma ist Null, wenn das Bein in einer Linie mit der Vorder/-Rückseite der Basis ist (rotiert mit Basis)
  //1:Alpha: Winkel zwischen Coxa und Femur
  //Alpha ist Null, wenn das Coxa und Femur in einer Linie sind
  //positive Winkel führen das Bein über diese Linie
  //2:Beta: Winkel zwischen Femur und Tibia
  //0:Gamma, 1:Alpha, 2:Beta
  double Leg_Ang[A_GELENKE];
  //Status der Beine (für Bein-LEDs)
  //false: Vorgegebene Bewegung des Beins nicht möglich, zB. TCP zu weit weg (pro Bein) oder berechneter Winkel ist NAN/INF
  //true: Bewegung kann mathematisch ausgeführt werden
  bool Leg_Status;
  
  public:

  //KONSTRUKTOREN
  
  //Konstruktor ohne Parameter: mit 0 initialisieren
  Leg();
  //Beinnummer mit dem Konstruktor initialisieren
  Leg(const int Nr_Leg);

  //KLASSENMETHODEN

  //Methode berechnet die Winkel Gamma, Alpha und Beta für die Gelenke der Beine
  //TCP: angestrebter TCP für das Bein
  //P_G: Punkte P_G des Beins
  //Vn_G0r: Normalisierter Nullwinkel für die Rotationsachsen Gamma
  //Vn_B: Normalenvektor der Basisebene
  void calcLegAng(const Vec3d TCP, const Vec3d P_G, const Vec3d Vn_G0r, const Vec3d Vn_B);
  
  //Getter für die Beinwinkel
  double getLegAng(int nrjoint) const; 
  //Getter für den Bein-Status
  bool getLegStat() const;
  
  //Ausgabe der Beinwinkel als String
  String printLegAng() const;
};

#endif