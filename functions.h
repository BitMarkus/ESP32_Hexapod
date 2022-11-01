#ifndef FUNCTIONS_H
#define FUNCTIONS_H

////////////////////////////
// Funktionsdeklarationen //
////////////////////////////

//Umrechnung von Rad in Grad und unmegehrt
double rad_in_grad(double rad);
double grad_in_rad(double grad);
//PS4-Controller Initialisierung beim drücken der PS-Taste
void onConnect();
//map() Funktion für double Values
//Normale map() Funktion gibt nur long Values zurück
double mapd(double val, double in_min, double in_max, double out_min, double out_max);


#endif

