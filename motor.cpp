#include "motor.h"

///////////////////////
// Klassendefinition //
///////////////////////

//KONSTRUKTOREN

//Konstruktor: Übergeben wird eine Referenz zum Positionsobjekt
Motor::Motor(Position& pos): pos(pos)
{
  //Wichtige Variablen initiieren
  
  //Die TCPs der Neutralstellung setzen
  TCP_ntr[0] = pos.getTCPntr(0);
  TCP_ntr[1] = pos.getTCPntr(1);
  TCP_ntr[2] = pos.getTCPntr(2);
  TCP_ntr[3] = pos.getTCPntr(3);
  TCP_ntr[4] = pos.getTCPntr(4);
  TCP_ntr[5] = pos.getTCPntr(5);
  //Die TCPs auf die Neutralstellung setzen
  TCP[0] = TCP_ntr[0];
  TCP[1] = TCP_ntr[1];
  TCP[2] = TCP_ntr[2];
  TCP[3] = TCP_ntr[3];
  TCP[4] = TCP_ntr[4];
  TCP[5] = TCP_ntr[5];
  //Die Winkel Lambda [Rad] in der Neutralposition für jedes Bein
  //Sollte besser berechnet werden aus der Roboterbasis und den TCPs
  Lambda_ntr[0] = grad_in_rad(30.0);
  Lambda_ntr[1] = grad_in_rad(330.0);
  Lambda_ntr[2] = grad_in_rad(90.0);
  Lambda_ntr[3] = grad_in_rad(270.0);
  Lambda_ntr[4] = grad_in_rad(150.0);
  Lambda_ntr[5] = grad_in_rad(210.0);  
  //Die Winkel Lambda [Rad] in der aktuellen Position für jedes Bein
  Lambda[0] = 0;
  Lambda[1] = 0;
  Lambda[2] = 0;
  Lambda[3] = 0;
  Lambda[4] = 0;
  Lambda[5] = 0;
  
  //Variablen für die Zyklus-Verwaltung
  //Anzahl der Einelschritte pro Zyklus
  cycle_steps = MOT_Cycle_Steps;
  //Anzahl aller Schritte einer Runde für jedes Gait
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  round_steps[0] = MOT_Gait_Cycles[0] * cycle_steps;
  round_steps[1] = MOT_Gait_Cycles[1] * cycle_steps;
  round_steps[2] = MOT_Gait_Cycles[2] * cycle_steps;
  round_steps[3] = MOT_Gait_Cycles[3] * cycle_steps;
  //Anzahl an Schritten im Return Stroke
  //Für alle Gaits gleich
  ret_steps = cycle_steps;
  //Anzahl an Schritten im Power Stroke für jedes Gait
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  pow_steps[0] = round_steps[0] - ret_steps;
  pow_steps[1] = round_steps[1] - ret_steps;
  pow_steps[2] = round_steps[2] - ret_steps;
  pow_steps[3] = round_steps[3] - ret_steps;

  //Letzten und ersten Schritt des Return Strokes für jedes Bein und für jedes Gait 
  //Innerhalb der gesamten Runde bestimmen
  //0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
  for(int g = 0; g < A_GAIT; g++)
  {
    for(int b = 0; b < A_BEINE; b++)
    {
      //letzter Schritt
      ret_last_step[g][b] = MOT_Ret_Stroke[g][b] * ret_steps;
      //erster Schritt
      ret_first_step[g][b] = ret_last_step[g][b] - (ret_steps - 1);
    }  
  } 
  
  //Abstand (Radius in xy-Ebene) von der Roboterbasis Z = Zero zum TCP [mm]
  //Radius der Rotation um das Roboterzentrum
  r_rot = R_BASIS + NTR_dS;
} 
  
//KLASSENMETHODEN

//Methode zum berechnen der Z-Koordinaten für TCP IM RETURN STROKE!
//Kann für mehrere Bewegungsprogramme verwendet werden
//nr_leg: Beinnummer
//step_in_ret: Aktueller Teilschritt im Return Stroke
//step_hight: Schritthöhe [mm]
double Motor::retZ(const int nr_leg, const int step_in_ret, const double step_hight)
{
  //Auf und ab des Beins als Sinuskurve
  return sin( (Pi/ret_steps) * (step_in_ret + 1) ) * (TCP_ntr[nr_leg][2] + step_hight);
} 

//Methode zum berechnen der TCPs für die Translationsbewegung des Roboters 
//Eingabe ist der Schritt innerhalb der Runde
//Ausgabe sind die TCPs aller Beine für die Bewegungsprogramme
//g = Gait: 0: wave, 1: ripple, 2: tripod (wird durch PS4 Controller vorgegeben) 
//step_in_round: Schritt innerhalb der gesamten Runde
//kappa: Winkel Kappa (in Rad) zur positiven Y-Achse = Richtung der Translation (wird durch PS4 Controller vorgegeben) 
//step_hight: Schritthöhe [mm] -> Für Methode retZ()
//step_width: Schrittweite [mm] 
void Motor::translRobot(const int g, const int step_in_round, const double kappa, const double step_hight, const double step_width)
{
  //TCPs für jedes Bein für den übergebenen Schritt (innerhalb der Runde)
  for(int b = 0; b < A_BEINE; b++)
  {
    //Variablen für die maximale und minimale Auslenkung der Beine in x-Richtung = Schrittweite
    //ist für jedes Gait gleich
    double x_max = TCP_ntr[b][0] + (step_width / 2.0);
    double x_min = x_max - step_width;
    //Variablen für die Positionsänderung in x-Richtung pro Schritt
    //Ist für Return Stroke vom Gait unabhängig 
    double dx_ret = step_width / ret_steps;
    //Ist für Power Stroke abhängig vom Gait
    double dx_pow = step_width / pow_steps[g];
    
    //Überprüfen, ob sich der aktuelle Schritt des Beins im Return Stroke befindet
    if(step_in_round >= ret_first_step[g][b] && step_in_round <= ret_last_step[g][b])
    {
      //Berechnen, wieviele Schritte man innerhalb des Return Strokes ist
      int step_in_ret = step_in_round - ret_first_step[g][b];      
      //TCP_X Berechnen
      TCP_X = x_min + (step_in_ret * dx_ret); 
      //TCP_Y (Neutralstellung)
      TCP_Y = TCP_ntr[b][1];
      //TCP_Z im Return Stroke berechnen -> Methode
      TCP_Z = retZ(b, step_in_ret, step_hight);
    }
    //Wenn nam sich im Power Stroke befindet
    else
    {
      int step_in_pow;
      //Berechnen, wieviele Schritte das Bein man vom Return Stroke entfernt ist
      if(step_in_round > ret_last_step[g][b] && step_in_round <= round_steps[g])
      {
        step_in_pow = step_in_round - ret_last_step[g][b] - 1;
      }
      else if(step_in_round < ret_first_step[g][b] && step_in_round >= 1)
      {
        step_in_pow = round_steps[g] - ret_last_step[g][b] - 1 + step_in_round;
      } 
      else
      {
        step_in_pow = 0;
      }
      //TCP_X Berechnen
      TCP_X = x_max - (step_in_pow * dx_pow);
      //TCP_Y (Neutralstellung)
      TCP_Y = TCP_ntr[b][1];
      //TCP_Z (Neutralstellung)
      TCP_Z = TCP_ntr[b][2];  
    } 
 
    //Rotation des TCPs um die Nutralposition um den Winkel Kappa (im Uhrzeigersinn)
    //Dadurch kann der Roboter ohne Rotation der Basis in jede beliebige Richtung laufen
    //kappa = 0° -> Vorwärts, kappa = 180° = Rückwärts, usw.

    //Vector, der von Null auf die Neutralstellung des Beins weist (nur x und y)
    Vec3d V_0_NTR(TCP_ntr[b][0], TCP_ntr[b][1], 0);
    //Vector, der von Null auf den berechneten TCP des Beins weist (nur x und y)
    Vec3d V_0_TCP(TCP_X, TCP_Y, 0);
    //Vector V_NTR_TCP erzeugen, der vom TCP der Neutralstellung auf den momentanen TCP des Beins weist
    //Nur X- und Y-Werte berücksichtigen
    Vec3d V_NTR_TCP = V_0_TCP - V_0_NTR;
    //Vector V_NTR_TCP um X Grad um die Z-Achse drehen (im Uhrzeigersinn) 
    Matr4x4d Mz(cos(kappa),  sin(kappa),  0,  0,
                -sin(kappa), cos(kappa),  0,  0,
                0,             0,         1,  0,
                0,             0,         0,  1); 
    Vec3d Vr_NTR_TCP;
    Mz.matr_x_vek(V_NTR_TCP, Vr_NTR_TCP);  
    //Rotierten Vector Vr_NTR_TCP mit Vector V_0_NTR addieren
    Vec3d Vr_0_NTR = V_0_NTR + Vr_NTR_TCP;  
  
    //Die XYZ-Werte für die TCPs des Beins setzen
    //Die X- und Y-Werte des Vectors Vr_0_NTR sind die neuen X- und Y-Wetre des TCPs
    TCP[b][0] = Vr_0_NTR[0];
    TCP[b][1] = Vr_0_NTR[1];
    TCP[b][2] = TCP_Z;    
  }
}

//Methode zum berechnen der TCPs für die Rotationsbewegung des Roboters (auf der Stelle)
//g = Gait: 0: wave, 1: ripple, 2: tripod (wird durch PS4 Controller vorgegeben)
//step_in_round: Schritt innerhalb der gesamten Runde
//dL: Winkeländerung in Rad für Lambda pro Runde (wird durch PS4 Controller vorgegeben)
//dir: Richtung der Drehung, 0 = CW, 1 = CCW (wird durch PS4 Controller vorgegeben)
//step_hight: Schritthöhe [mm] -> Für Methode retZ()
void Motor::rotateRobot(const int g, const int step_in_round, const double dL, const int dir, const double step_hight)
{
  //TCPs für jedes Bein für den übergebenen Schritt (innerhalb der Runde)
  for(int b = 0; b < A_BEINE; b++)
  {
    //Die minimale und maximale und minimale Auslenkung für den Winkel Lambda [Rad] für jedes Bein
    //Ist für jedes Gait gleich
    double L_min = Lambda_ntr[b] - (dL / 2.0);
    double L_max = L_min + dL;  
    //Die Winkeländerung pro Schritt 
    //für das übergebene Gait und für die übergebene Auslenkung berechnen
    //Return Stroke
    double dl_ret = dL / ret_steps;
    //Power Stroke
    double dl_pow = dL / pow_steps[g];
    
    //Überprüfen, ob sich der aktuelle Schritt des Beins im Return Stroke befindet
    double lambda;
    if(step_in_round >= ret_first_step[g][b] && step_in_round <= ret_last_step[g][b])
    {
      //Berechnen, wieviele Schritte man innerhalb des Return Strokes ist
      int step_in_ret = step_in_round - ret_first_step[g][b];
      //Den Winkel Lambda [Rad] berechnen -> von der Drehrichtung abhängig
      if(dir == 1)
      {
        lambda = L_min + (step_in_ret * dl_ret);
      }
      else
      {
        lambda = L_max - (step_in_ret * dl_ret);
      }

      //TCP_Z im Return Stroke berechnen -> Methode
      TCP_Z = retZ(b, step_in_ret, step_hight);      
    }
    //Wenn man sich im Power Stroke befindet
    else
    {
      int step_in_pow;
      //Berechnen, wieviele Schritte das Bein man vom Return Stroke entfernt ist
      if(step_in_round > ret_last_step[g][b] && step_in_round <= round_steps[g])
      {
        step_in_pow = step_in_round - ret_last_step[g][b] - 1;
      }
      else if(step_in_round < ret_first_step[g][b] && step_in_round >= 1)
      {
        step_in_pow = round_steps[g] - ret_last_step[g][b] - 1 + step_in_round;
      } 
      else
      {
        step_in_pow = 0;
      }
      
      //Den Winkel Lambda [Rad] berechnen -> von der Drehrichtung abhängig
      if(dir == 1)
      {
        lambda = L_max - (step_in_pow * dl_pow);
      }
      else
      {
        lambda = L_min + (step_in_pow * dl_pow);
      }

      //TCP_Z (Neutralstellung)
      TCP_Z = TCP_ntr[b][2];       
    }
   
    //TCP_X Berechnen
    TCP_X = cos(lambda) * r_rot;
    //TCP_Y Berechnen
    TCP_Y = sin(lambda) * r_rot; 
    
    //Die XYZ-Werte für die TCPs des Beins setzen
    //Die X- und Y-Werte des Vectors Vr_0_NTR sind die neuen X- und Y-Wetre des TCPs
    //TCP[b][0] = rad_in_grad(lambda);
    TCP[b][0] = TCP_X;    
    TCP[b][1] = TCP_Y;
    TCP[b][2] = TCP_Z;    
  }
}

//Methode dreht den Roboter nach erfolgter Translation um die Z-Achse
//Für Kurven
void Motor::turnRobot(const int g, const int step_in_round, const double dL, const int dir)
{
  //TCPs für jedes Bein für den übergebenen Schritt (innerhalb der Runde)
  for(int b = 0; b < A_BEINE; b++)
  {
    //Abstand des aktuellen TCPs des Beins zum Basiszentrum berechnen
    //Vektor vom Zentrum der Basis zum TCP
    Vec3d V_0_TCP(TCP[b][0], TCP[b][1], TCP[b][2]);
    //Betrag des Vektors bestimmen
    double L_0_TCP = V_0_TCP.betrag(); 
    
    //Beinwinkel Lambda für die aktuellen TCPs berechnen
    Lambda[b] = acos(V_0_TCP[0] / L_0_TCP);
    //Vorsicht!!! Geht nur bis 180°
    if(V_0_TCP[1] < 0)
    {
      Lambda[b] = Pi + (Pi - Lambda[b]);
    }    
    
    //Die maximale und minimale Auslenkung für den Winkel Lambda [Rad] für jedes Bein
    //Ist für jedes Gait gleich
    double L_min = Lambda[b] - (dL / 2.0);
    double L_max = L_min + dL;      
    //Die Winkeländerung pro Schritt 
    //für das übergebene Gait und für die übergebene Auslenkung berechnen
    //Return Stroke
    double dl_ret = dL / ret_steps;
    //Power Stroke
    double dl_pow = dL / pow_steps[g];  
  
    //Überprüfen, ob sich der aktuelle Schritt des Beins im Return Stroke befindet
    double Lambda_new;
    if(step_in_round >= ret_first_step[g][b] && step_in_round <= ret_last_step[g][b])
    {
      //Berechnen, wieviele Schritte man innerhalb des Return Strokes ist
      int step_in_ret = step_in_round - ret_first_step[g][b];
      
      //Den Winkel Lambda_new [Rad] berechnen -> von der Drehrichtung abhängig
      if(dir == 1)
      {
        Lambda_new = L_min + (step_in_ret * dl_ret);
      }
      else
      {
        Lambda_new = L_max - (step_in_ret * dl_ret);
      }    
    } 
    //Wenn man sich im Power Stroke befindet
    else
    {
      int step_in_pow;
      //Berechnen, wieviele Schritte das Bein man vom Return Stroke entfernt ist
      if(step_in_round > ret_last_step[g][b] && step_in_round <= round_steps[g])
      {
        step_in_pow = step_in_round - ret_last_step[g][b] - 1;
      }
      else if(step_in_round < ret_first_step[g][b] && step_in_round >= 1)
      {
        step_in_pow = round_steps[g] - ret_last_step[g][b] - 1 + step_in_round;
      } 
      else
      {
        step_in_pow = 0;
      }
      
      //Den Winkel Lambda [Rad] berechnen -> von der Drehrichtung abhängig
      if(dir == 1)
      {
        Lambda_new = L_max - (step_in_pow * dl_pow);
      }
      else
      {
        Lambda_new = L_min + (step_in_pow * dl_pow);
      }  
    }
    
    //TCP_X Berechnen
    TCP_X = cos(Lambda_new) * L_0_TCP;
    //TCP_Y Berechnen
    TCP_Y = sin(Lambda_new) * L_0_TCP; 
    
    //Die XY-Werte für die TCPs der Beine setzen
    //Die Z-Werte werden vom Translationsprogramm übernommen
    TCP[b][0] = TCP_X;    
    TCP[b][1] = TCP_Y;     
  }
}

//Setzt die TCPs im Positions-Objekt auf die berechneten TCPs im Motor-Objekt
void Motor::setPosTcp()
{
  pos.setPosTcp(this->TCP);
}

//Getter
//Getter für die Rückgabe des TCP Vektor-Objekts
Vec3d Motor::getTCP(int nr_leg) const 
{
  return this->TCP[nr_leg];
}
//Getter für die Anzahl aller Schritte einer Runde für ein übergebenes Gait
//0: wave, 1: ripple, 2: tripod
int Motor::getRoundSteps(int gait) const 
{
  return this->round_steps[gait];
}

//Methode zur Ausgabe von Membervariablen als String
String Motor::printTCP()
{
  //Ausgabe als String
  String strg_ausgabe = String("TCPs [mm]:\n");
  for(int i = 0; i < A_BEINE; i++)
  {  
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": ";
    strg_ausgabe += TCP[i].printVec3();
    strg_ausgabe += "\n";
  } 
  strg_ausgabe += "Lambda [Grad]:\n";
  for(int i = 0; i < A_BEINE; i++)
  {  
    strg_ausgabe += "Bein ";
    strg_ausgabe += i;  
    strg_ausgabe += ": ";
    strg_ausgabe += rad_in_grad(Lambda[i]);
    strg_ausgabe += "\n";
  }   
  return strg_ausgabe;
} 














