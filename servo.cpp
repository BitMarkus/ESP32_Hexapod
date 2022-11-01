#include "servo.h"

///////////////////////
// Klassendefinition //
///////////////////////

//KONSTRUKTOREN

//Standardkonstruktor: Servo-Objekt ohne Parameter erzeugen
//Alle Parameter werden als 0 initialisiert
ServoPWM::ServoPWM() {}
//Servo mit übergebenen Variablen initialisieren
ServoPWM::ServoPWM( const int Pin, 
                    const int dP90, 
                    const int P90, 
                    const int Servo_Dir, 
                    const int Min_Pulse, 
                    const int Max_Pulse)
                  {
                    SERVO_Pin = Pin;
                    SERVO_dP90 = dP90;
                    SERVO_P90 = P90;
                    SERVO_Dir = Servo_Dir;
                    SERVO_Min_Pulse = Min_Pulse;
                    SERVO_Max_Pulse = Max_Pulse;
                    dSERVO_End_Pulse = 0; 
                  }

//KLASSENMETHODEN

//Methode berechnet die Pulslänge für den übergebenen Gelenkwinkel
//SERVO_Winkel: Servo-Winkel wird in Rad übergeben
//Pulslänge für den Servo wird in der Klassenvariable "SERVO_End_Pulse" gespeichert
//Wird nur innerhalb der Klasse benötigt
void ServoPWM::calcPuls(double SERVO_Winkel)
{  
  //Den übergebenen Winkel in Pulslänge [µs] umrechnen
  dSERVO_End_Pulse = SERVO_P90 - (1 - rad_in_grad(SERVO_Winkel)/90) * SERVO_Dir * SERVO_dP90;
  
  //Bewegungsfreiheit als Anschlagsbremse einschränken: MIN_MAX_PULSE
  if(dSERVO_End_Pulse > SERVO_Max_Pulse)
  {
    dSERVO_End_Pulse = SERVO_Max_Pulse; 
    //Servo-Status auf false setzen
    SERVO_Status = false;
  }
  else if(dSERVO_End_Pulse < SERVO_Min_Pulse)
  {
    dSERVO_End_Pulse = SERVO_Min_Pulse;   
    //Servo-Status auf false setzen
    SERVO_Status = false;
  }
  else
  {
    //Servo-Status auf true setzen
    SERVO_Status = true;
  }
  
  //Die Pulslänge mit 4 multiplizieren
  //Die Pulslängen für die Funktion maestro.setTarget [1/4 µs] sind um das 4fache größer
  //als die Pulslängen, die über die Kalibrierung mit der Maestro Software erhalten werden [µs]
  //Info: setTarget takes the channel number you want to control, and the target position in units of 1/4 microseconds
  dSERVO_End_Pulse *= 4;
  
  //Pulslänge auf int beschneiden
  SERVO_End_Pulse = dSERVO_End_Pulse + 0.5;
} 

//Methode setzt den Servo auf den übergebenen Winkel "SERVO_Ziel"
//maestro: Referenz auf das Servo-Treiber Objekt
void ServoPWM::setServoToAng(const double SERVO_Winkel, MiniMaestro &maestro)
{
  //Pulslänge berechnen (Klassenmethode) und in der Klassenvariable "SERVO_End_Pulse" speichern
  calcPuls(SERVO_Winkel);
  //Die neue Position anfahren
  maestro.setTarget(SERVO_Pin, SERVO_End_Pulse);  
} 

//Methode setzt den Servo auf die übergebene Pulslänge "SERVO_Puls"
//Entspricht der Pulslänge, die tatsächlich an den Servo gesendet wird
//= Pulslänge in der Maestro Software * 4
//maestro: Referenz auf das Servo-Treiber Objekt
void ServoPWM::setServoToPuls(const int SERVO_Puls, MiniMaestro &maestro)
{
  //Die neue Position anfahren
  maestro.setTarget(SERVO_Pin, SERVO_Puls);
}

//Getter für die Pulslänge (int)
int ServoPWM::getPuls(const double SERVO_Winkel)
{
  //Pulslänge berechnen (Klassenmethode) und in der Klassenvariable "SERVO_End_Pulse" speichern
  calcPuls(SERVO_Winkel);
  //Pulslänge als int ausgeben
  return this->SERVO_End_Pulse;
}

//Getter für den Servo(Gelenk)-Status
bool ServoPWM::getServoStat() const
{
  return this->SERVO_Status;
}





