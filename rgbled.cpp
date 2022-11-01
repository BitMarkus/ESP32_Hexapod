#include "rgbled.h"

///////////////////////
// Klassendefinition //
///////////////////////

//KONSTRUKTOREN

//Standardkonstruktor
RgbLed::RgbLed() 
{
  //Array für die RGB-Werte auf Null setzen
  for(int i = 0; i < A_BEINE; i++)
  { 
    for(int j = 0; j < 3; j++)
    {
      rgb[i][j] = 0;
    }
  }           
}

//KLASSENMETHODEN

//Setter setzten die RGB Werte der Bein LEDs auf eine bestimmte Farbe und Helligkeit

//Bewegungs-Modus:
//Grün: Vordere Beine (0,1)
//Gelb: Mittlere Beine (2,3)
//Rot: Hintere Beine (4,5)
//Alle Beine werden gesetzt
void RgbLed::setRgbMove(const int bright)
{
  //Bein 0: Grün
  rgb[0][0] = 0; 
  rgb[0][1] = map(bright, 0, 100, 0, 255); 
  rgb[0][2] = 0;  
  //Bein 1: Grün
  rgb[1][0] = 0; 
  rgb[1][1] = map(bright, 0, 100, 0, 255); 
  rgb[1][2] = 0;
  //Bein 2: Gelb
  rgb[2][0] = map((bright*0.75), 0, 100, 0, 255); 
  rgb[2][1] = map((bright*0.55), 0, 100, 0, 255); 
  rgb[2][2] = 0;  
  //Bein 3: Gelb
  rgb[3][0] = map((bright*0.75), 0, 100, 0, 255); 
  rgb[3][1] = map((bright*0.55), 0, 100, 0, 255); 
  rgb[3][2] = 0; 
  //Bein 4: Rot
  rgb[4][0] = map(bright, 0, 100, 0, 255); 
  rgb[4][1] = 0; 
  rgb[4][2] = 0;  
  //Bein 5: Rot
  rgb[5][0] = map(bright, 0, 100, 0, 255); 
  rgb[5][1] = 0; 
  rgb[5][2] = 0;
}

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
void RgbLed::setRgbDebug(const int nr_leg, const bool status_leg, const bool status_g, const bool status_a, const bool status_b, const int bright)
{
  //Wenn KEINE Warnung für das gesamte Bein vorliegt
  if(status_leg)
  {
    //Überprüfen, ob eine Warnung für eines der Gelenke vorliegt
    if(status_g && status_a && status_b)
    {
      //LED des Beins leuchtet Grün auf
      rgb[nr_leg][0] = 0; 
      rgb[nr_leg][1] = map(bright, 0, 100, 0, 255); 
      rgb[nr_leg][2] = 0;
    }
    //Wenn eine Warnung für eines der Gelenke am Bein vorliegt
    else
    {
      //LED des Beins leuchtet Gelb auf
      rgb[nr_leg][0] = map((bright*0.75), 0, 100, 0, 255); 
      rgb[nr_leg][1] = map((bright*0.55), 0, 100, 0, 255); 
      rgb[nr_leg][2] = 0;
    }
  }
  //Wenn eine Warnung für das gesamte Bein vorliegt
  else
  {
    //LED des Beins leuchtet Rot auf
    rgb[nr_leg][0] = map(bright, 0, 100, 0, 255); 
    rgb[nr_leg][1] = 0; 
    rgb[nr_leg][2] = 0;
  }          
}

//Getter
//Methoden gibt die RGB-Werte für ein bestimmtes Bein aus
int RgbLed::getR(const int nr_leg)
{
  return this->rgb[nr_leg][0];
}
int RgbLed::getG(const int nr_leg)
{
  return this->rgb[nr_leg][1];
}
int RgbLed::getB(const int nr_leg)
{
  return this->rgb[nr_leg][2];
}