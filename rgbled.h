#ifndef RGBLED_H
#define RGBLED_H

#include <Arduino.h> //wegen map function

//#include <FastLED.h>

//Anzahl der Beine für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3

////////////////////////
// Klassendeklaration //
////////////////////////

//Klasse für die Status-LEDs
//Vorest eine RGB LED pro Bein
class RgbLed
{
  private:
  
  //MEMBERVARIABEN
  
  //Membervariable speichert die RGB-Werte für die LEDs
  //0: red, 1:green, 2:blue
  int rgb[A_BEINE][3];
  
  public:  
  
  //KONSTRUKTOREN
  
  //Standardkonstruktor
  RgbLed();
  
  //KLASSENMETHODEN
  
  //Setter setzten die RGB Werte der Bein LEDs auf eine bestimmte Farbe und Helligkeit
  
  //Bewegungs-Modus:
  //Grün: Vordere Beine (0,1)
  //Gelb: Mittlere Beine (2,3)
  //Rot: Hintere Beine (4,5)
  //Alle Beine werden gesetzt
  void setRgbMove(const int bright);
  
  //Debug-Modus:
  //Rot: Vorgegebene Bewegung des Beins mathematisch nicht möglich, zB. TCP zu weit weg (pro Bein) oder berechneter Winkel ist NAN/INF
  //Gelb:Anschlag der Servos am Min-/Max-Puls
  //Grün: Bewegung kann ausgeführt werden
  //nur ein Bein kann gesetzt werden
  //übergeben werden die:
  //nr_leg: Beinnummer
  //status_leg: Statusabfrage für das gesamte Beine
  //status_g,a,b: Statusabfrage für den einzelnen Servo (Gelenk), g=Gamma, a=Alpha, b=Beta
  //bright: Helligkeit in % (1-100)
  void setRgbDebug(const int nr_leg, const bool status_leg, const bool status_g, const bool status_a, const bool status_b, const int bright);
  
  //Getter
  //Methoden gibt die RGB-Werte für ein bestimmtes Bein aus
  int getR(const int nr_leg);
  int getG(const int nr_leg);
  int getB(const int nr_leg);
  
};

#endif