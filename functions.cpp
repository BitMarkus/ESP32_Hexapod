#include "functions.h"
#include "const.h"
#include <PS4Controller.h>

///////////////////////////
// Funktionsdefinitionen //
///////////////////////////

//Umrechnung von Rad in Grad und unmegehrt
//Funktion zur Umrechnung von Radians in Grad
double rad_in_grad(double rad)
{
    return rad * (180 / Pi);
}
//Funktion zur Umrechnung von Grad in Radians
double grad_in_rad(double grad)
{
    return grad * (Pi / 180);
}
//PS4-Controller Initialisierung beim drücken der PS-Taste
//It needs to follow a pre-defined signature: return void and receive no arguments
//Just prints the value of the isConnected method, to confirm that the controller is indeed connected to the ESP32
void onConnect()
{
  Serial.println("PS4-Controller connected!");
  //Returns 1 if connected
  //Serial.println(PS4.isConnected()); 
  //Serial.println(); 
}
//map() Funktion für double Values
//Normale map() Funktion gibt nur long Values zurück
double mapd(double val, double in_min, double in_max, double out_min, double out_max) 
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

