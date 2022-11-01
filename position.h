#ifndef POSITION_H
#define POSITION_H

#include "basis.h"
#include "const.h"
#include "functions.h"
#include "vector.h"
#include "leg.h"
#include "servo.h"

//Anzahl der Beine für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3

////////////////////////
// Klassendeklaration //
////////////////////////

//Klasse zum Positionieren aller Servomotoren
class Position
{
  //MEMBERVARIABEN

  private:
  
  //Variablen für die Positionerung
  //Eulerwinkel der Basis im Bogemass
  //0: Phi (x), 1: Theta (y), 2: Psi (z)
  double Euler_Ang[3];
  //Vektor für Basis-Offset [mm]
  Vec3d V_off;
  //Standweite = Abstand Gelenk Gamma zum TCP in xy-Ebene [mm]
  double G_TCP;
  //Abstand (Radius in xy-Ebene) von der Roboterbasis Z = Zero zum TCP
  double Z_TCP;   
  //TCPs der Neutralposition für jedes Bein
  Vec3d TCP_ntr[A_BEINE]; 
  //TCPs der Position für jedes Bein
  Vec3d TCP[A_BEINE]; 
  
  //Output der Klasse
  //Gelenkwinkel der Position
  double Ang[A_BEINE][A_GELENKE];
  //Pulslängen der Position
  int Puls[A_BEINE][A_GELENKE]; 
  
  //Status der Beine (für Bein-LEDs)
  //false: Vorgegebene Bewegung des Beins nicht möglich, zB. TCP zu weit weg (pro Bein) oder berechneter Winkel ist NAN/INF
  //true: fehlerfrei
  bool Status_leg[A_BEINE];
  //Status der Gelenke (für Bein-LEDs)
  //false: Anschlag der Servos am Min-/Max-Puls
  //true: fehlerfrei
  bool Status_joint[A_BEINE][A_GELENKE];
  
  //Objekte
  //Basis-Objekt erzeugen
  Basis bas;
  //Array für Bein-Objekte deklarieren
  Leg leg[A_BEINE];
  //Array für ServoPWM-Objekte deklarieren
  ServoPWM servos[A_BEINE][A_GELENKE];
  //Maestro Servo Treiber Objekt am Serial2 Port initialisieren
  MiniMaestro maestro;

  //KONSTRUKTOREN
  
  public:

  //Konstruktor
  //Wird mit dem seriellen Port initialisiert, über den der Maestro Servo Treiber verbunden ist
  Position(HardwareSerial &serPort);

  //KLASSENMETHODEN
  
  private:
  
  //Methode berechnet die Beinwinkel (in Rad) und die Pulslängen für die Position
  void calcPos();

  public:
  
  //Setter
  //Methode setzt alle Positionsvariablen auf die Neutralposition
  void setNtrPos();
  //Methode setzen nur TCPs auf die Neutralposition
  void setNtrPosTcp();
  //Methode setzt alle Positionsvariablen auf die übergebene Position
  void setPos(const double *euang, const Vec3d& voff, const Vec3d *tcp);  
  //Methode setzen die TCPs auf die übergebene Position -> überladene Funktion
  //Bei Übergabe der Referenz auf ein Array von Vektor-Objekten werden die Punkte aller Beine gesetzt
  void setPosTcp(const Vec3d *tcp);
  //Bei Übergabe eines einzelnen Vektor-Objekts wird nur der Punkt für das entsprechende Bein gesetzt
  void setPosTcp(int index, const Vec3d tcp); 
  //Methode setzt das Basis Offset auf die übergebene Position -> überladene Funktion
  //Bei Übergabe der Referenz auf ein Vektor-Objekts wird der ganze Vektor gesetzt
  void setPosVoff(const Vec3d& voff);
  //Bei Angabe einer Zahl können die x, y und z Koordinaten des Vektors gesetzt werden
  void setPosVoff(int index, double value);
  //Methode setzt die Euler-Winkel auf die übergebene Position
  void setPosEuAng(int index, double value);
  
  
  //Methode setzt die Servos auf die Positionsvariablen
  void goToPos();
  
  //Getter für die TCPs der Neutralstellung -> überladene Funktion
  //Bei Angabe eines Parameters, Rückgabe des gesamten Vektors für ein Bein
  Vec3d getTCPntr(int index1) const;
  //Bei Angabe von zwei Parametern, Rückgabe der x, y und z Koordinaten eines Vektors als Double
  double getTCPntr(int index1, int index2) const;
  //Getter für den Status der Beine und der Gelenke (Servos) -> überladene Funktion
  //Bei Angabe eines Parameters, Rückgabe des Bein-Status
  bool getStatus(int index1) const;
  //Bei Angabe von zwei Parametern, Rückgabe des Gelenk(Servo)-Status
  bool getStatus(int index1, int index2) const;
  
  //Methode zur Ausgabe der Position als String
  String printPos(); 
  //Methode gibt die Servo-Konstaten aus
  //Von jedem Positions-Objekt können sämmtliche Servo-Konstaten als Tabelle abgerufen werden
  //PIN, dP90, P90, DIR, MIN_PULSE, MAX_PULSE
  String printServoConst() const;

};

#endif