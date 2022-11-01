#include "leg.h"

///////////////////////
// Klassendefinition //
///////////////////////

//MEMBERVARIABEN

//Ausgabe der Kasse
//Beinwinkel werden im Bogenmass in einem Array gespeichert
//Gamma: Hüftwinkel für Horizontalrotation des Beins
//Alpha: Winkel zwischen Coxa und Femur
//Beta: Winkel zwischen Femur und Tibia
//0:Gamma, 1:Alpha, 2:Beta
double Leg_Ang[A_GELENKE] = {0,0,0};

//KONSTRUKTOREN

//Konstruktor ohne Parameter: mit 0 initialisieren
Leg::Leg() {Nr_Leg = 0;}
//Beinnummer mit dem Konstruktor initialisieren
Leg::Leg(const int nr) : Nr_Leg(nr) {} 

//KLASSENMETHODEN

//Methode berechnet die Winkel Gamma, Alpha und Beta für die Gelenke der Beine
//TCP: angestrebter TCP für das Bein
//P_G: Punkte P_G des Beins
//Vn_G0r: Normalisierter Nullwinkel für die Rotationsachsen Gamma
//Vn_B: Normalenvektor der Basisebene
void Leg::calcLegAng(const Vec3d TCP, const Vec3d P_G, const Vec3d Vn_G0r, const Vec3d Vn_B)
{
  //////////////////////////
  // Berechnung von Gamma //
  //////////////////////////

  //Vektor erzeugen, der vom Punkt P_G der rotierten Basis zum TCP zeigt
  Vec3d V_G_TCP = TCP - P_G;
  //L_G_TCP muss mindestens L_FEM lang sein, sonst ist der TCP zu nah am Gelenk G
  double L_G_TCP = V_G_TCP.betrag();
  if(L_G_TCP < L_FEM)
  {
    //Berechnung nicht moeglich: TCP zu nah am Gelenk Gamma!
    //Keine Änderung von Alpha, Beta und Gamma
    //Bein-Status auf false setzen
    Leg_Status = false;
  }
  else
  {
    //Vektor für Null-Position Gamma
    //Ist für jedes Bein unterschiedlich
    Vec3d V_G0;
    //Beine zeigen nach links
    if(Nr_Leg == 0 || Nr_Leg == 2 || Nr_Leg == 4)
    {
      V_G0 = Vn_G0r * -1;
    }
    //Beine zeigen nach rechts
    else if(Nr_Leg == 1 || Nr_Leg == 3 || Nr_Leg == 5)
    {
      V_G0 = Vn_G0r;
    }

    //Für die Berechnung von Gamma muss der Winkel zwischen Vektor V_G_TCP
    //und Vektor V_G01 (Verlängerung der Vorderseite nach links) berechnet werden
    //Nur die x und y-Koordinaten sind wichtig!
    Vec3d Vn_G_TCPxy(V_G_TCP[0], V_G_TCP[1], 0);
    Vn_G_TCPxy.normalisieren();
    Vec3d Vn_G0xy(V_G0[0], V_G0[1], 0);
    Vn_G0xy.normalisieren();

    //Gamma berechnen
    //Vorsicht! Gamma wird immer als positiver Winkel berechnet
    //d.h. das Vorzeichen muss noch bestimmt werden
    //Gamma = acos(Vn_G_TCPxy.skalarprod(Vn_G0xy));
    Leg_Ang[0] = acos(Vn_G_TCPxy.skalarprod(Vn_G0xy));

    //Vorzeichen von Gamma bestimmen
    //Wenn das Bein hinter die Nulllage geht, muss der Winkel negativ sein
    //Funktioniert so nur mit -45° <= Gamma <= 45°
    if(Vn_G_TCPxy[0] < Vn_G0xy[0])
    {
      Leg_Ang[0] *= -1;
    }

    /////////////////////////
    // Punkt P_A berechnen //
    /////////////////////////

    //Vn_B: Normalenvektor der Ebene (Berechnet für P_G[0], zeigt in positive z-Richtung)
    //Einen Normalenvektor Vn_G_A kreieren, der vom Punkt P_G[i] zum Punkt P_A[i] zeigt
    //Die x- und y-Richtung des Vektors zeigt von P_G[i] nach TCP[i] = Vektor V_G_TCP
    //Der Koeffizient für die z-Richtung muss über das Skalarprodukt (=0) mit dem
    //Normalenvektor der Basis berechnet werden und soll damit in der (gedrehten) Basisebene liegen
    double zGA = (-1 * ((V_G_TCP[0] * Vn_B[0]) + (V_G_TCP[1] * Vn_B[1]))) / Vn_B[2];
    Vec3d Vn_G_A(V_G_TCP[0], V_G_TCP[1], zGA);
    Vn_G_A.normalisieren();

    //Punkt P_A berechnen
    Vec3d P_A = P_G + (Vn_G_A * L_COX);

    /////////////////////////
    // Punkt P_B berechnen //
    /////////////////////////

    //Vektor V_TCP_A berechnen und normalisierten Vektor Vn_TCP_A erzeugen
    //Betrag L_TCP1_A ist die Länge von TCP1 zu P_A1
    Vec3d V_TCP_A = P_A - TCP;
    double L_TCP_A = V_TCP_A.betrag();
    // L_TCP1_A darf nicht größer als L_TIB + L_FEM werden, ansonsten ist das Ziel nicht erreichbar!
    if(L_TCP_A > L_TIB + L_FEM)
    {            
        //Berechnung nicht moeglich: TCP nicht erreichbar!
        //Keine Änderung von Alpha, Beta und Gamma
        //Bein-Status auf false setzen
        Leg_Status = false;
    }
    else
    {
      Vec3d Vn_TCP_A = V_TCP_A.normalisieren_neu();

      //Hilfswinkel Delta berechnen
      //Delta ist der Winkel zwischen V_TCP_A und V_TCP_B
      //Kosinussatz für Dreieck mit drei bekannten Seitenlängen
      //Delta soll immer positiv sein, damit das Bein nicht nach unten "durchschlägt"
      double Delta = acos( (pow(L_FEM, 2) - pow(L_TIB, 2) - pow(L_TCP_A, 2)) / (-2 * L_TIB * L_TCP_A) );

      //Hilfswinkel Epsilon berechnen
      //Epsilon ist der Winkel zwischen V_TCP_A und einem Vektor der parallel zur y-Achse verläuft
      //Winkel in x,y-Richtung wird berechnet ("von oben betrachtet")
      double Epsilon;
      Vec3d E1(V_TCP_A[0], V_TCP_A[1], 0);
      //Ist für jedes Bein unterschiedlich:
      Vec3d E2;
      //Beine zeigen nach links
      if(Nr_Leg == 0 || Nr_Leg == 2 || Nr_Leg == 4)
      {
        E2 = {0, 1, 0};
      }
      //Beine zeigen nach rechts
      else if(Nr_Leg == 1 || Nr_Leg == 3 || Nr_Leg == 5)
      {
        E2 = {0, -1, 0};
      }
      Epsilon = acos(E1.skalarprod(E2) / E1.betrag());
      //Vorzeichen von Epsilon berechnen
      if(Vn_TCP_A[0] > 0)
      {
        Epsilon *= -1;
      }

      //Zur Berechnung von P_B wird der normalisierte Vektor Vn_TCP_A:
      //1) um die z-Achse rotiert, damit er parallel zur y-Achse verläuft (-Epsilon)
      //2) um die x-Achse um den Winkel Delta rotiert
      //3) wieder zurück um die z-Achse rotiert (+Epsilon)
      //Muss für linke und rechte Beine getrennt gemacht werden!

      //Negative Rotation um die z-Achse
      Matr4x4d MzE1(cos(Epsilon),  sin(Epsilon),   0,  0,
                    -sin(Epsilon),  cos(Epsilon),  0,  0,
                    0,              0,             1,  0,
                    0,              0,             0,  1);
      //Positive Rotation um die z-Achse
      Matr4x4d MzE2(cos(Epsilon),  -sin(Epsilon),  0,  0,
                    sin(Epsilon),   cos(Epsilon),  0,  0,
                    0,              0,             1,  0,
                    0,              0,             0,  1);
      //Rotationsmatrix
      Matr4x4d Mzxz;
      //Beine zeigen nach links
      if(Nr_Leg == 0 || Nr_Leg == 2 || Nr_Leg == 4)
      {
        //Rotation um die x-Achse
        Matr4x4d MxD(1,  0,            0,            0,
                     0,  cos(Delta),   -sin(Delta),  0,
                     0,  sin(Delta),   cos(Delta),   0,
                     0,  0,            0,            1);
        //Rotationsmatrix
        Mzxz = MzE2 * MxD * MzE1;
      }
      //Beine zeigen nach rechts
      else if(Nr_Leg == 1 || Nr_Leg == 3 || Nr_Leg == 5)
      {
        //Rotation um die x-Achse
        Matr4x4d MxD(1,  0,            0,            0,
                     0,  cos(Delta),   sin(Delta),  0,
                     0,  -sin(Delta),   cos(Delta),   0,
                     0,  0,            0,            1);
        //Rotationsmatrix
        Mzxz = MzE1 * MxD * MzE2;
      }
      //Matr4x4d Mzxz = MzE2 * MxD * MzE1;
      //Vektor V_TCP_B berechnen und normalisieren
      Vec3d Vn_TCP_B;
      Mzxz.matr_x_vek(Vn_TCP_A, Vn_TCP_B);
      Vn_TCP_B.normalisieren();

      //Punkt P_B1 berechnen
      Vec3d P_B = TCP + (Vn_TCP_B * L_TIB);

      /////////////////////////
      // Berechnung von Beta //
      /////////////////////////

      //Kosinussatz für Dreieck mit drei bekannten Seitenlängen
      Leg_Ang[2] = acos( (pow(L_TCP_A, 2) - pow(L_TIB, 2) - pow(L_FEM, 2)) / (-2 * L_TIB * L_FEM) );

      //////////////////////////
      // Berechnung von Alpha //
      //////////////////////////

      //Alpha ist der Winkel zwischen dem Normalenvektor Vn_G_A (P_G - P_A)
      //und dem Normalenvektor Vn_A_B (P_A - P_B)
      //Normalenvektor Vn_A_B erzeugen
      Vec3d Vn_A_B = P_B - P_A;
      Vn_A_B.normalisieren();

      //Winkel berechnen
      //Alpha = acos(Vn_G_A.skalarprod(Vn_A_B));
      Leg_Ang[1] = acos(Vn_G_A.skalarprod(Vn_A_B));

      //Vorzeichen von Alpha bestimmen
      if(Vn_G_A[2] > Vn_A_B[2])
      {
        Leg_Ang[1] *= -1;
      }
      
      //Bein-Status auf true setzen
      Leg_Status = true;  
    }
  }
  
  //Wenn die Servowinkel keine Zahl sind oder unendliche Zahlen (Kalkulation falsch)
  //Dann sollen die Winkel auf den Nullwinkel gesetzt werden
  for(int i = 0; i < 3; i++)
  {
    if(isnan(Leg_Ang[i]) || isinf(Leg_Ang[i])) 
    {
      Leg_Ang[i] = 0;
      //Bein-Status auf false setzen
      Leg_Status = false;              
    }   
  } 
}

//Getter für die Beinwinkel
double Leg::getLegAng(int nrjoint) const 
{
  return this->Leg_Ang[nrjoint];
}
//Getter für den Bein-Status
bool Leg::getLegStat() const
{
  return this->Leg_Status;
}

//Ausgabe der Beinwinkel als String
String Leg::printLegAng() const
{
  String strg_ausgabe = String("Beinwinkel [Grad] ");
  strg_ausgabe += Nr_Leg;  
  strg_ausgabe += ": ";
  strg_ausgabe += rad_in_grad(Leg_Ang[0]);
  strg_ausgabe += ", ";
  strg_ausgabe += rad_in_grad(Leg_Ang[1]);
  strg_ausgabe += ", ";
  strg_ausgabe += rad_in_grad(Leg_Ang[2]);
  strg_ausgabe += "\n";
  
  return strg_ausgabe;
}
