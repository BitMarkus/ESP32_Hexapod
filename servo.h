#ifndef SERVO_H
#define SERVO_H

#include "const.h"
#include "functions.h"
#include <PololuMaestro.h>

//Anzahl der Beine für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3

////////////////////////
// Klassendeklaration //
////////////////////////

//Klasse übersetzt Winkel Gamma, Alpha und Beta der Beingelenke
//in Pulslängen für die entsprechenden Servo-Motoren
class ServoPWM
{
  private:
  
  //MEMBERVARIABEN

  //Werden beim Start initialisiert
  int SERVO_Pin;
  int SERVO_dP90;
  int SERVO_P90;
  int SERVO_Dir;
  int SERVO_Min_Pulse;
  int SERVO_Max_Pulse; 
 
  //Ausgabe der Klasse 
  //Angestrebte Pulslänge, die aus dem übergebenen Winkel berechnet wird  
  double dSERVO_End_Pulse;  
  int SERVO_End_Pulse;    
  //Status der Gelenke (für Bein-LEDs)
  //false: Anschlag der Servos am Min-/Max-Puls
  //true: Bewegung kann technisch ausgeführt werden
  bool SERVO_Status;

  public:  
  
  //KONSTRUKTOREN
  
  //Standardkonstruktor: Parameter mit 0 initialisieren
  ServoPWM();
  //Servo mit übergebenen Variablen initialisieren
  ServoPWM( const int Pin, 
            const int dP90, 
            const int P90, 
            const int Servo_Dir, 
            const int Min_Pulse, 
            const int Max_Pulse );
  
  //KLASSENMETHODEN
  
  private:
  
  //Methode berechnet die Pulslänge für den übergebenen Gelenkwinkel
  //SERVO_Winkel: Servo-Winkel wird in Rad übergeben
  //Wird nur innerhalb der Klasse benötigt
  void calcPuls(double SERVO_Winkel);

  public:
  
  //Methode setzt den Servo auf den übergebenen Winkel "SERVO_Ziel"
  //maestro: Referenz auf das Servo-Treiber Objekt
  void setServoToAng(const double SERVO_Winkel, MiniMaestro &maestro); 
  
  //Methode setzt den Servo auf die übergebene Pulslänge"SERVO_Puls"
  //Entspricht der Pulslänge, die tatsächlich an den Servo gesendet wird
  //= Pulslänge in der Maestro Software * 4
  //maestro: Referenz auf das Servo-Treiber Objekt
  void setServoToPuls(const int SERVO_Puls, MiniMaestro &maestro); 
  
  //Getter für die Pulslänge (int)
  int getPuls(const double SERVO_Winkel);
  //Getter für den Servo(Gelenk)-Status
  bool getServoStat() const;
};

#endif
