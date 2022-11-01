#include "position.h"

///////////////////////
// Klassendefinition //
///////////////////////

//KONSTRUKTOREN

//Wird mit dem seriellen Port initialisiert, über den der Maestro Servo Treiber verbunden ist
Position::Position(HardwareSerial &serPort) : maestro(serPort) 
{
  //Bein-Objekte und ServoPWM-Objekte erstellen
  for(int i = 0; i < A_BEINE; i++)
  {
    leg[i] = Leg(i);
    //ServoPWM-Objekt für jeden Servo als Array erstellen
    for(int j = 0; j < A_GELENKE; j++)
    {
      //0=PIN, 1=dP90, 2=P90, 3=DIR, 4=MIN_PULSE, 6=MAX_PULSE
      servos[i][j] = ServoPWM( SERVO_Array[i][j][0],
                               SERVO_Array[i][j][1],
                               SERVO_Array[i][j][2],
                               SERVO_Array[i][j][3],
                               SERVO_Array[i][j][4],
                               SERVO_Array[i][j][5] );
    }
  }  
  //Parameter mit der Neutralstelling initialisieren
  //Eulerwinkel der Basis
  Euler_Ang[0] = NTR_Euler_Ang[0];
  Euler_Ang[1] = NTR_Euler_Ang[1];
  Euler_Ang[2] = NTR_Euler_Ang[2];
  //Vektor für Basis-Offset
  V_off = {NTR_V_off[0], NTR_V_off[1], NTR_V_off[2]};
  //Standweite = Abstand Gelenk Gamma zum TCP in xy-Ebene [mm]
  G_TCP = NTR_dS;
  //Abstand (Radius in xy-Ebene) von der Roboterbasis Z = Zero zum TCP
  Z_TCP = R_BASIS + G_TCP;
  //Die TCPs der Neutralstellung berechnen
  TCP_ntr[0] = { (Z_TCP * sin(2*Pi*(1.0/6.0))),  (Z_TCP * cos(2*Pi*(1.0/6.0))), 0 };   //Bein 0: i = 1
  TCP_ntr[1] = { (Z_TCP * sin(2*Pi*(2.0/6.0))),  (Z_TCP * cos(2*Pi*(2.0/6.0))), 0 };   //Bein 1: i = 2
  TCP_ntr[2] = { 0,                               Z_TCP,                        0 };   //Bein 2: (i = 6)
  TCP_ntr[3] = { 0,                              -Z_TCP,                        0 };   //Bein 3: (i = 3)
  TCP_ntr[4] = { (Z_TCP * sin(2*Pi*(5.0/6.0))),  (Z_TCP * cos(2*Pi*(5.0/6.0))), 0 };   //Bein 4: i = 5 
  TCP_ntr[5] = { (Z_TCP * sin(2*Pi*(4.0/6.0))),  (Z_TCP * cos(2*Pi*(4.0/6.0))), 0 };   //Bein 5: i = 4
  //Die TCPs auf die Neutralstellung setzen
  TCP[0] = TCP_ntr[0];
  TCP[1] = TCP_ntr[1];
  TCP[2] = TCP_ntr[2];
  TCP[3] = TCP_ntr[3];
  TCP[4] = TCP_ntr[4];
  TCP[5] = TCP_ntr[5];
  //Den Status der Beine und Gelenke auf "fehlerfrei" = true setzen (für Bein-LEDs)
  for(int i = 0; i < A_BEINE; i++)
  { 
    Status_leg[i] = true;
    for(int j = 0; j < A_GELENKE; j++)
    {
      Status_joint[i][j] = true;
    }
  }
} 

//KLASSENMETHODEN

//Methode setzt alle Positionsvariablen auf die Neutralposition
void Position::setNtrPos()
{
  //Eulerwinkel der Basis
  Euler_Ang[0] = NTR_Euler_Ang[0];
  Euler_Ang[1] = NTR_Euler_Ang[1];
  Euler_Ang[2] = NTR_Euler_Ang[2];
  //Vektor für Basis-Offset
  V_off = {NTR_V_off[0], NTR_V_off[1], NTR_V_off[2]};
  //Standweite = Abstand Gelenk Gamma zum TCP in xy-Ebene [mm]
  G_TCP = NTR_dS;
  //Abstand (Radius in xy-Ebene) von der Roboterbasis Z = Zero zum TCP
  Z_TCP = R_BASIS + G_TCP;
  //Die TCPs auf die Neutralstellung setzen
  TCP[0] = TCP_ntr[0];
  TCP[1] = TCP_ntr[1];
  TCP[2] = TCP_ntr[2];
  TCP[3] = TCP_ntr[3];
  TCP[4] = TCP_ntr[4];
  TCP[5] = TCP_ntr[5];
}

//Methode setzen nur TCPs auf die Neutralposition
void Position::setNtrPosTcp()
{
  //Die TCPs auf die Neutralstellung setzen
  TCP[0] = TCP_ntr[0];
  TCP[1] = TCP_ntr[1];
  TCP[2] = TCP_ntr[2];
  TCP[3] = TCP_ntr[3];
  TCP[4] = TCP_ntr[4];
  TCP[5] = TCP_ntr[5];
}

//Methode setzt alle Positionsvariablen auf die übergebene Position
void Position::setPos(const double *euang, const Vec3d& voff, const Vec3d *tcp)
{
  //Eulerwinkel der Basis
  Euler_Ang[0] = euang[0];
  Euler_Ang[1] = euang[1];
  Euler_Ang[2] = euang[2];
  //Vektor für Basis-Offset
  V_off = {voff[0], voff[1], voff[2]};
  //TCPs
  TCP[0] = tcp[0];
  TCP[1] = tcp[1];
  TCP[2] = tcp[2];
  TCP[3] = tcp[3];
  TCP[4] = tcp[4];
  TCP[5] = tcp[5];
}

//Methode setzen die TCPs auf die übergebene Position -> überladene Funktion
//Bei Übergabe der Referenz auf ein Array von Vektor-Objekten werden die Punkte aller Beine gesetzt
void Position::setPosTcp(const Vec3d *tcp)
{
  TCP[0] = tcp[0];
  TCP[1] = tcp[1];
  TCP[2] = tcp[2];
  TCP[3] = tcp[3];
  TCP[4] = tcp[4];
  TCP[5] = tcp[5];
}
//Bei Übergabe eines einzelnen Vektor-Objekts wird nur der Punkt für das entsprechende Bein gesetzt
void Position::setPosTcp(int index, const Vec3d tcp)
{
  TCP[index] = tcp;
}

//Methode setzt das Basis Offset auf die übergebene Position -> überladene Funktion
//Bei Übergabe der Referenz auf ein Vektor-Objekts wird der ganze Vektor gesetzt
void Position::setPosVoff(const Vec3d& voff)
{
  //Vektor für Basis-Offset
  V_off = {voff[0], voff[1], voff[2]};
}
//Bei Angabe einer Zahl können die x, y und z Koordinaten des Vektors gesetzt werden
void Position::setPosVoff(int index, double value)
{
  V_off[index] = value;
}
//Methode setzt die Euler-Winkel auf die übergebene Position
void Position::setPosEuAng(int index, double value)
{
  Euler_Ang[index] = value;
}

//Methode berechnet die Beinwinkel (in Rad) und die Pulslängen für die Position
void Position::calcPos()
{
  //Basis translatieren
  bas.basisTrans(Euler_Ang, V_off);
  //Beinwinkel berechnen
  for(int i = 0; i < A_BEINE; i++)
  { 
    //Beinwinkel (in Rad) berechnen
    leg[i].calcLegAng(TCP[i], bas.getP_G(i), bas.getVn_G0r(), bas.getVn_B());
    //Status des Beins speichern (für Bein-LEDs)
    Status_leg[i] = leg[i].getLegStat();
    //Beinwinkel und Pulslängen in den Klassenvariablen Ang und Puls speichern
    for(int j = 0; j < A_GELENKE; j++)
    {  
      Ang[i][j] = leg[i].getLegAng(j);
      Puls[i][j] = servos[i][j].getPuls(leg[i].getLegAng(j));
      //Status des Servos (Gelenks) speichern  (für Bein-LEDs)
      Status_joint[i][j] = servos[i][j].getServoStat();
    }
  }
}

//Methode setzt die Servos auf die Positionsvariablen
void Position::goToPos()
{
  //Beinwinkel (in Rad) und die Pulslängen für die Position berechnen: Methode
  calcPos();
  
  //Gelenke (Servos) des Beins auf die berechneten Winkel/Pulslängen bewegen
  for(int i = 0; i < A_BEINE; i++)
  { 
    for(int j = 0; j < A_GELENKE; j++)
    {     
      servos[i][j].setServoToPuls(Puls[i][j], maestro);
    }
  }
}

//Getter für die TCPs der Neutralstellung -> überladene Funktion
//Bei Angabe eines Parameters, Rückgabe des gesamten Vektors für ein Bein
Vec3d Position::getTCPntr(int index1) const
{
  return this->TCP_ntr[index1];
}
//Bei Angabe von zwei Parametern, Rückgabe der x, y und z Koordinaten eines Vektors
double Position::getTCPntr(int index1, int index2) const
{
  return this->TCP_ntr[index1][index2];
}

//Getter für den Status der Beine und der Gelenke (Servos) -> überladene Funktion
//Bei Angabe eines Parameters, Rückgabe des Bein-Status
bool Position::getStatus(int index1) const
{
  return this->Status_leg[index1];
}
//Bei Angabe von zwei Parametern, Rückgabe des Gelenk(Servo)-Status
bool Position::getStatus(int index1, int index2) const
{
  return this->Status_joint[index1][index2];
}

//Methode zur Ausgabe der Position
String Position::printPos()
{
  //Beinwinkel (in Rad) und die Pulslängen für die Position berechnen: Methode
  calcPos();
  
  //Ausgabe als String
  String strg_ausgabe = String("\n--- Position ---\n");
  String strg1 = String(", ");
  //Eulerwinkel
  strg_ausgabe += "\nEulerwinkel [Gad]:\n";
  strg_ausgabe += "Phi: ";
  strg_ausgabe += rad_in_grad(Euler_Ang[0]); 
  strg_ausgabe += strg1;
  strg_ausgabe += "Theta: ";
  strg_ausgabe += rad_in_grad(Euler_Ang[1]); 
  strg_ausgabe += strg1; 
  strg_ausgabe += "Psi: ";
  strg_ausgabe += rad_in_grad(Euler_Ang[2]); 
  strg_ausgabe += "\n";
  //Offset
  strg_ausgabe += "Offset [mm]:\n";
  strg_ausgabe += V_off.printVec3();
  strg_ausgabe += "\n";
  //Basisgeometrie
  strg_ausgabe += "Basisgeometrie:\n";
  //Radius der Roboterbasis zum Gelenk Gamma [mm]
  strg_ausgabe += "Basisradius [mm]: ";
  strg_ausgabe += R_BASIS;
  strg_ausgabe += "\n";
  //Standweite = Abstand Gelenk Gamma zum TCP [mm]
  strg_ausgabe += "Standweite G_TCP [mm]: ";
  strg_ausgabe += G_TCP;
  strg_ausgabe += "\n";     
  //Abstand (Radius) vom Ursprung der Roboterbasis zum TCP [mm]
  strg_ausgabe += "Z_TCP (Basisradius+Standweite) [mm]: ";
  strg_ausgabe += Z_TCP;
  strg_ausgabe += "\n"; 
  //TCPs der Position
  strg_ausgabe += "TCPs [mm]:\n";  
  for(int i = 0; i < A_BEINE; i++)
  {  
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": ";
    strg_ausgabe += TCP[i].printVec3();
    strg_ausgabe += "\n";
  }  
  //Beinwinkel in der Position
  strg_ausgabe += "Beinwinkel: Gamma, Alpha, Beta [Grad]:\n";
  for(int i = 0; i < A_BEINE; i++)
  { 
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": "; 
    strg_ausgabe += rad_in_grad(Ang[i][0]);
    strg_ausgabe += ", ";
    strg_ausgabe += rad_in_grad(Ang[i][1]);
    strg_ausgabe += ", ";
    strg_ausgabe += rad_in_grad(Ang[i][2]);
    strg_ausgabe += "\n";
  }
  //Pulslängen in der Position
  //Pulslängen müssen durch 4 geteilt werden, um wie in der Maestro Software angezeigt zu werden
  strg_ausgabe += "Pulslaengen: Gamma, Alpha, Beta\n";
  for(int i = 0; i < A_BEINE; i++)
  { 
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": "; 
    strg_ausgabe += (Puls[i][0]/4);
    strg_ausgabe += ", ";
    strg_ausgabe += (Puls[i][1]/4);
    strg_ausgabe += ", ";
    strg_ausgabe += (Puls[i][2]/4);
    strg_ausgabe += "\n";
  }

  return strg_ausgabe;
} 

//Methode gibt die Servo-Konstaten aus
//Von jedem Positions-Objekt können sämmtliche Servo-Konstaten als Tabelle abgerufen werden
//PIN, dP90, P90, DIR, MIN_PULSE, MAX_PULSE
String Position::printServoConst() const
{
  String strg_ausgabe = String("--- Servokonstanten ---\n\n");
  
  strg_ausgabe += "Leg, Joint, Pin, dP90, P90, DIR, MIN_P, tMAX_P:\n";
  
  for(int i = 0; i < A_BEINE; i++)
  { 
    for(int j = 0; j < A_GELENKE; j++)
    { 
      strg_ausgabe += i;
      strg_ausgabe += ", "; 
      strg_ausgabe += j;
      strg_ausgabe += ", ";     
      strg_ausgabe += SERVO_Array[i][j][0];
      strg_ausgabe += ", ";
      strg_ausgabe += SERVO_Array[i][j][1];
      strg_ausgabe += ", ";
      strg_ausgabe += SERVO_Array[i][j][2];
      strg_ausgabe += ", ";
      strg_ausgabe += SERVO_Array[i][j][3];
      strg_ausgabe += ", ";
      strg_ausgabe += SERVO_Array[i][j][4];
      strg_ausgabe += ", ";
      strg_ausgabe += SERVO_Array[i][j][5];
      strg_ausgabe += "\n";
    }
  }
  
  return strg_ausgabe;
}

