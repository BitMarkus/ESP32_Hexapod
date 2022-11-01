#ifndef MOTOR_H
#define MOTOR_H

#include "const.h"
#include "vector.h"
#include "matrix.h"
#include "position.h"

//Anzahl der Beine für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3
#define A_GAIT 4

////////////////////////
// Klassendeklaration //
////////////////////////

//Klasse für die Bewegungsprogramme des Roboters
class Motor
{
  //MEMBERVARIABEN

  private:
  
  //Ausgabe der Klasse
  //TCPs der Position für jedes Bein
  Vec3d TCP[A_BEINE];  
  //Variablen für Zwischenergebnisse
  double TCP_X, TCP_Y, TCP_Z;
  
  //Referenz zu Positions-Objekt
  Position &pos;
  //TCPs der Neutralposition für jedes Bein
  Vec3d TCP_ntr[A_BEINE];  
  //Die Winkel Lambda [Rad] in der Neutralposition für jedes Bein
  double Lambda_ntr[A_BEINE];
  //Abstand (Radius in xy-Ebene) von der Roboterbasis Z = Zero zum TCP [mm]
  //Radius der Rotation um das Roboterzentrum
  double r_rot;
  //Die Winkel Lambda [Rad] in der aktuellen Position für jedes Bein
  double Lambda[A_BEINE];
  
  //Variablen für die Zyklus-Verwaltung  
  //Anzahl der Einelschritte pro Zyklus
  int cycle_steps;
  //Anzahl aller Schritte einer Runde für jedes Gait
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  int round_steps[A_GAIT];
  //Anzahl an Schritten im Return Stroke
  //Für alle Gaits gleich
  int ret_steps;
  //Anzahl an Schritten im Power Stroke für jedes Gait
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  int pow_steps[A_GAIT];
  //Letzten und ersten Schritt des Return Strokes für jedes Bein und für jedes Gait 
  //Innerhalb der gesamten Runde bestimmen
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  int ret_last_step[A_GAIT][A_BEINE];
  int ret_first_step[A_GAIT][A_BEINE];
    
  //KONSTRUKTOREN
  
  public:

  //Konstruktor: Übergeben wird eine Referenz zum Positionsobjekt
  //Dient primär zum Auslesen der Parameter für die Neutralstellung
  Motor(Position& pos);  

  //KLASSENMETHODEN
  
  private:
  
  //Methode zum berechnen der Z-Koordinaten für TCP IM RETURN STROKE!
  //Kann für mehrere Bewegungsprogramme verwendet werden
  //nr_leg: Beinnummer
  //step_in_ret: Aktueller Teilschritt im Return Stroke
  //step_hight: Schritthöhe [mm]
  double retZ(const int nr_leg, const int step_in_ret, const double step_hight); 
  
  public:
  
  //Methode zum berechnen der TCPs für die Translationsbewegung des Roboters 
  //Eingabe ist der Schritt innerhalb der Runde
  //Ausgabe sind die TCPs aller Beine für die Bewegungsprogramme
  //g = Gait: 0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod (wird durch PS4 Controller vorgegeben)
  //step_in_round: Schritt innerhalb der gesamten Runde
  //kappa: Winkel Kappa (in Rad) zur positiven Y-Achse = Richtung der Translation (wird durch PS4 Controller vorgegeben)
  //step_hight: Schritthöhe [mm] -> Für Methode retZ()
  //step_width: Schrittweite [mm] 
  void translRobot(const int g, const int step_in_round, const double kappa, const double step_hight, const double step_width);
  
  //Methode zum berechnen der TCPs für die Rotationsbewegung des Roboters (auf der Stelle)
  //g = Gait: 0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod (wird durch PS4 Controller vorgegeben)
  //step_in_round: Schritt innerhalb der gesamten Runde
  //dL: Winkeländerung in Rad für Lambda pro Runde (wird durch PS4 Controller vorgegeben)
  //dir: Richtung der Drehung, 0 = CW, 1 = CCW (wird durch PS4 Controller vorgegeben)
  //step_hight: Schritthöhe [mm] -> Für Methode retZ()
  void rotateRobot(const int g, const int step_in_round, const double dL, const int dir, const double step_hight);
  
  //Methode dreht den Roboter nach erfolgter Translation
  void turnRobot(const int g, const int step_in_round, const double dL, const int dir);
  
  //Setzt die TCPs im Positions-Objekt auf die berechneten TCPs im Motor-Objekt
  void setPosTcp();
  
  //Getter
  //Getter für die Rückgabe des TCP Vektor-Objekts
  Vec3d getTCP(int nr_leg) const;
  //Getter für die Anzahl aller Schritte einer Runde für ein übergebenes Gait
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  int getRoundSteps(int gait) const;
  
  //Methode zur Ausgabe der TCPs
  String printTCP(); 
};

#endif