//#include "basis.h"
//#include "leg.h"
//#include "servo.h"
//#include <PololuMaestro.h>
//#include "matrix.h"
//#include "vector.h"
#include "const.h"
#include "functions.h"
#include "position.h"
#include "motor.h"
#include <PS4Controller.h>
#include <FastLED.h>
#include "rgbled.h"

//Anzahl der Beine und der Gelenke pro Bein
//für die Initialisierung von Arrays
#define A_BEINE 6
#define A_GELENKE 3
#define A_GAIT 4

//Pins für UART Kommunikation mit dem Maestro Servo Treiber
//U2UXD wird bei allen ESP32 Modellen verwendet (nicht U0UXD oder U1UXD)
//Da nur Daten vom ESP32 zum Maestro Servo Controller gesendet werden, ist nur der TX-Pin wichtig (transmit)
//Dieser befindet sich bei allen verwendeten ESP Modellen an Pin 17 (Master Hawk ESP32, AZ Delivery, Esp32 LoRa Wifi Kit V2)
//VORSICHT! Der ESP32 arbeitet mit 3,3V Logik, der Maestro Servo Controller mit 5V Logik
//Das Senden von Signalen vom ESP32 zum Maestro funktioniert, aber eigentlich ist die Schwelle zum HIGH bei 4,0V
//Hier sollte besser ein Logic Level Converter (3,3 nach 5V) eingesetzt werden
//Beim Senden von Daten vom Maestro zum ESP32 kann der ESP32 beschädigt werden!!!!
//Wenn das nötig sein sollte, dann MUSS ein Logic Level Converter (5V nach 3,3V) eingesetzt werden
//Wichtige Info:
/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/
#define RXD2 12  //Orange: Dummy-Port
#define TXD2 17  //Gelb: TX2 des ESP32 -> geht zu RX-Port des Maestro

//Pin für die Status-LEDs
#define LED_PIN 13

/////////////
// OBJEKTE //
/////////////

//Position-Objekt erstellen
Position pos(Serial2);
//Motor-Objekt erzeugen
Motor mot(pos);
//Array von CRGB-Objekten für die RGB-LEDs (der Beine) erstellen
//Zuordnunng der Objekt-Indices zu den Bein-Indices:
//led[0] = Bein 1
//led[1] = Bein 0
//led[2] = Bein 2
//led[3] = Bein 4
//led[4] = Bein 5
//led[5] = Bein 3
CRGB led[A_BEINE];
//RGBLED-Objekt erstellen
RgbLed rgb;

////////////////////////////
// Laufrhythmus-Variablen //
////////////////////////////

//Variable für die Rhythmus-Zeit
unsigned long RHY_time = 0;
//Schritt innerhalb des Zyklus
int RHY_step = 1; 
//Aktuelle Zeit
unsigned long RHY_NOW_Time;

////////////////////
// PS4-Controller //
////////////////////

//Bewegungsgeschwindigkeit
//Zeit in ms gibt an, wie lange ein Einzelschritt dauern soll
//Kann über den PS4 Controller zwischen min und max variiert werden
int RHY_delay_min = 20;
int RHY_delay_max = 50;
int RHY_delay = RHY_delay_max;
//Winkel Kappa (in Rad) zur positiven Y-Achse berechnen (= Laufrichtung)
double J_kappa = 0;
//Angabe der Joystick Deadzone in %
//Deadzone bei beiden Joysticks für x- und y-Achse gleich
double Jr_dz_proz = 10.0;
double Jl_dz_proz = 10.0;
//Umrechnen in 8-bit/2 (0-128) Schritte (halbe Joystickachse)
double J_dz_steps = 128.0;
//Runden: +0.5
int Jr_dz = ((J_dz_steps * Jr_dz_proz) / 100.0) + 0.5;
int Jl_dz = ((J_dz_steps * Jl_dz_proz) / 100.0) + 0.5; 
//Variable für die Bewegungsmodi des linken Joysticks
//0: X=Rotation um die x-Achse (Phi), Y= Rotation um die y-Achse (Theta)
//1: X=Verschiebung um die x-Achse (X-Offset), Y=Verschiebung um die y-Achse (Y-Offset) 
//2: X=Rotation um die z-Achse (Psi), Y= keine Zuordnung 
//3: X=Drehung des Roboters während des Laufens und Drehung um die Z-Achse in den Extrempositionen -> Standard
int Jl_menue = 3;
//Variable für das Gate
//0: wave -> Standard, 1: quadpod, 2: ripple, 3: tripod
int J_gait = 0;
//Rotationswinkel Lambda für die Drehung des Roboters [Rad]
//Winkeländerung pro Runde
double J_lam_min = grad_in_rad(1.0);     //Minimal 1 Grad
double J_lam_max = grad_in_rad(J_dPsi);  //Maximal die Konstante J_dPsi (z-Rotation)
double J_lam = J_lam_min;
//Richtung der Drehung, -1 = CW (mathemat. neg), 1 = CCW (mathemat. pos)
int J_lam_dir = -1;
//Schritthöhe der Beine beim Laufen
double J_step_hight_min = MOT_dZ;
double J_step_hight_max = MOT_dZ * 2.0;
double J_step_hight = MOT_dZ;
//Schrittweite der Beine beim Laufen
double J_step_width_min = MOT_dX + 30.0;  
double J_step_width_max = MOT_dX - 30.0; 
double J_step_width = MOT_dX; //mittlere Standatd-Schrittweite


///////////
// SETUP //
///////////

void setup()
{
  //RGB-LEDs (der Beine) initialisieren
  FastLED.addLeds<WS2812, LED_PIN, GRB>(led, A_BEINE); 
  led[0] = CRGB(0, 0, 100);    //Bein 1
  led[1] = CRGB(0, 0, 100);    //Bein 0
  led[2] = CRGB(0, 0, 100);    //Bein 2
  led[3] = CRGB(0, 0, 100);    //Bein 4
  led[4] = CRGB(0, 0, 100);    //Bein 5  
  led[5] = CRGB(0, 0, 100);    //Bein 3
  FastLED.show();
  
  //Serielle Kommunikation mit dem Maestro Servo Controller
  //ACHTUNG! Baudrate muss auch in der Maestro Software eingestellt werden
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  //Seriellen Monitor für Kommunikation mit dem PC initialisieren
  Serial.begin(115200);
  Serial.println("\n--- Start Serial Monitor ---\n");
  
  /*
  //Roboter auf Neutral-Position setzen
  pos.setNtrPos();
  //pos.goToPos();
  //Ausgabe der Neutral-Position und der Servokonstanten
  Serial.println(pos.printPos());   
  Serial.println(pos.printServoConst());  
  */
  
  //Kommunikation mit dem PS4 Controller starten
  //Übergabe der MAC-Adresse, welche auf dem Controller gespeichert ist
  PS4.begin("03:03:03:03:03:03");
  Serial.println("Press PS-Button to initialize PS4-Controller...");
  //This method receives as input our callback function
  PS4.attachOnConnect(onConnect); 
  
  //Delay 
  delay(100);
}

//////////
// LOOP //
//////////

void loop()
{
  //Wenn ein PS4-Controller verbunden ist, sind Bewegungen möglich
  if(PS4.isConnected()) 
  {
    ////////////////////
    // PS4-Controller //
    ////////////////////

    //Variable für die Bewegungsmodi des linken Joysticks
    //Viereck: X=Rotation um die x-Achse (Phi), Y= Rotation um die y-Achse (Theta)
    //Dreieck: X=Verschiebung um die x-Achse (X-Offset), Y=Verschiebung um die y-Achse (Y-Offset) 
    //Kreis: X=Rotation um die z-Achse (Psi), Y=keine Zuordnung 
    //Kreuz: X=Drehung des Roboters während des Laufens und Drehung um die Z-Achse in den Extrempositionen -> Standard, Y=Veränderung der Schrittweite
    if(PS4.Square())
    {
      Jl_menue = 0;
    }
    if(PS4.Triangle())
    {
      Jl_menue = 1;
    }
    if(PS4.Circle())
    {
      Jl_menue = 2;
    }
    if(PS4.Cross())
    {
      Jl_menue = 3;
    }    
  
    /// Linker Joystick ///
    
    switch(Jl_menue)
    {    
      case 0:  
        //Joystick links: x-Achse
        //Linker Anschlag: -128, Rechts: 127
        //Rotation um die x-Achse (Phi)
        double euler_ang_phi;
        if(PS4.LStickX())
        {
          euler_ang_phi = mapd(PS4.LStickX(), -128, 127, NTR_Euler_Ang[0] - grad_in_rad(J_dPhi), NTR_Euler_Ang[0] + grad_in_rad(J_dPhi));      
          pos.setPosEuAng(0, euler_ang_phi);
        }
        else
        {
          pos.setPosEuAng(0, NTR_Euler_Ang[0]);   
        }    
        //Joystick links: y-Achse
        //Unterer Anschlag: -128, Oben: 127 
        //Rotation um die y-Achse (Theta) 
        double euler_ang_theta;
        if(PS4.LStickY())
        { 
          euler_ang_theta = mapd(PS4.LStickY(), -128, 127, NTR_Euler_Ang[1] - grad_in_rad(J_dTheta), NTR_Euler_Ang[1] + grad_in_rad(J_dTheta));  
          pos.setPosEuAng(1, euler_ang_theta);  
        }
        else
        {
          pos.setPosEuAng(1, NTR_Euler_Ang[1]);    
        }
      break;
      
      case 1:  
        //Joystick links: x-Achse
        //Linker Anschlag: -128, Rechts: 127
        //Verschiebung um die y-Achse (Y-Offset)
        double y_offs;
        if(PS4.LStickX())
        {
          y_offs = mapd(PS4.LStickX(), -128, 127, NTR_V_off[1] + J_dY_Offs, NTR_V_off[1] - J_dY_Offs);      
          pos.setPosVoff(1, y_offs);
        }
        else
        {
          pos.setPosVoff(1, NTR_V_off[1]);   
        }    
        //Joystick links: y-Achse
        //Unterer Anschlag: -128, Oben: 127 
        //Verschiebung um die x-Achse (X-Offset)
        double x_offs;
        if(PS4.LStickY())
        { 
          x_offs = mapd(PS4.LStickY(), -128, 127, NTR_V_off[0] - J_dX_Offs, NTR_V_off[0] + J_dX_Offs);  
          pos.setPosVoff(0, x_offs); 
        }
        else
        {
          pos.setPosVoff(0, NTR_V_off[0]);    
        }
      break;
      
      case 2:  
        //Joystick links: x-Achse
        //Linker Anschlag: -128, Rechts: 127
        //Rotation um die z-Achse (Psi)
        double euler_ang_psi;
        if(PS4.LStickX())
        {
          euler_ang_psi = mapd(PS4.LStickX(), -128, 127, NTR_Euler_Ang[2] + grad_in_rad(J_dPsi), NTR_Euler_Ang[2] - grad_in_rad(J_dPsi));      
          pos.setPosEuAng(2, euler_ang_psi);
        }
        else
        {
          pos.setPosEuAng(2, NTR_Euler_Ang[2]); 
        }    
        //Joystick links: y-Achse
        //Unterer Anschlag: -128, Oben: 127 
        //Keine Zuordnung
      break;   
     
      case 3:   
        //Joystick links: x-Achse
        //Linker Anschlag: -128, Rechts: 127
        //Rotationswinkel Lambda pro Runde   
        if(PS4.LStickX() > Jl_dz)
        {    
          J_lam = mapd(PS4.LStickX(), Jl_dz, 127, J_lam_min, J_lam_max);
          //Rotationsrichtung
          J_lam_dir = -1;
        } 
        else if(PS4.LStickX() < -Jl_dz)
        {    
          J_lam = mapd(PS4.LStickX(), -Jl_dz, -128, J_lam_min, J_lam_max);
          //Rotationsrichtung
          J_lam_dir = 1;
        }  
        else
        {
          J_lam = J_lam_min;
          J_lam_dir = -1;
        }
        //Joystick links: y-Achse
        //Unterer Anschlag: -128, Oben: 127 
        //Veränderung der Schrittweite
        if(PS4.LStickY() > Jl_dz)
        {    
          J_step_width = mapd(PS4.LStickY(), Jl_dz, 127, MOT_dX, J_step_width_min);
        } 
        else if(PS4.LStickY() < -Jl_dz)
        {    
          J_step_width = mapd(PS4.LStickY(), -Jl_dz, -128, MOT_dX, J_step_width_max);
        }          
        else 
        {
          J_step_width = MOT_dX;
        }
      break;  
    }
    
    /// Rechter Joystick ///
    
    //Vector V_JR beschreibt die x- und y-Position des analogen Joysticks
    //in einem 2D Koordinatensystem (-128 bis 127)
    Vec3d V_JR(PS4.RStickX(), PS4.RStickY(), 0);
    //Betrag des Vektors bestimmen (= Laufgeschwindigkeit)
    double L_V_JR = V_JR.betrag(); 
    //Delaytime und damit Laufgeschwindigkeit einstellen
    RHY_delay = map(L_V_JR, Jr_dz, 127, RHY_delay_max, RHY_delay_min);
    //Werte kleiner oder größer min/max Werte abfangen: 
    //hier WICHTIG!, da sonst negative Werte entstehen können
    if(RHY_delay < RHY_delay_min)
    {
      RHY_delay = RHY_delay_min;
    }
    if(RHY_delay > RHY_delay_max)
    {
      RHY_delay = RHY_delay_max;
    }   
    //Winkel und Delay Time nur setzen, wenn L_V_JR > Deadzone Wert
    if(L_V_JR > Jr_dz)
    {    
      //Winkel Kappa (in Rad) zur positiven Y-Achse berechnen (= Laufrichtung)
      //Vorsicht!!! Geht nur bis 180°
      J_kappa = acos(V_JR[1] / L_V_JR);
      if(V_JR[0] < 0)
      {
        J_kappa = Pi + (Pi - J_kappa);
      }
    }
    else
    {
      //RHY_delay = RHY_delay_max;
      J_kappa = 0;
    }
    
    /// Analoge Schultertasten ///
    
    //linke analoge Schultertaste: 0-255
    //Verschiebt das z-Offset in die positive Richtung
    //Beim Drücken beider analoger Schultertasten überschreibt die positive Richtung immer die negative!
    double z_offs;
    if(PS4.L2())
    {
      z_offs = mapd(PS4.L2Value(), 0, 255, NTR_V_off[2], (NTR_V_off[2] + J_dZ_Offs_pos));
      pos.setPosVoff(2, z_offs); 
    }
    /*
    //rechte analoge Schultertaste: 0-255
    //Verschiebt das z-Offset in die negative Richtung
    else if(PS4.R2())
    {
      z_offs = mapd(PS4.R2Value(), 0, 255, NTR_V_off[2], (NTR_V_off[2] - J_dZ_Offs_neg));
      pos.setPosVoff(2, z_offs); 
    }
    */
    //z-Offset in die Neutralposition bringen
    else
    {
      pos.setPosVoff(2, NTR_V_off[2]);   
    } 
    
    //rechte analoge Schultertaste: 0-255
    //Erhöht die Schritthöhe beim Laufen
    if(PS4.R2())
    {
      J_step_hight = mapd(PS4.R2Value(), 0, 255, J_step_hight_min, J_step_hight_max);
    }
    else
    {
      J_step_hight = J_step_hight_min;   
    }
    
    /// Pfeiltasten ///
  
    //Variablen für das Durchschalten der Gates
    //Left = 0: wave -> Standard
    //Right = 3: tripod
    //Down = 1: quadpod
    //Up = 2: ripple    
    if(PS4.Left())
    {
      J_gait = 0;
    }
    if(PS4.Right())
    {
      J_gait = 3;
    }
    if(PS4.Up())
    {
      J_gait = 2;
    }
    if(PS4.Down())
    {
      J_gait = 1;
    }  
      
    ///////////////////////////////////
    // Bewegung von Basis und Beinen //
    ///////////////////////////////////
    
    RHY_NOW_Time = millis();   
    if(RHY_NOW_Time - RHY_time > RHY_delay)
    { 
      //Drehung auf der Stelle bei maximaler Auslenkung der x-Achse des linken Joysticks
      //Überschreibt die Translation des Roboters
      //Nur die Schrittgeschwindigkeit kann dabei vom rechten Joystick kontrolliert werden
      //Die Schrittweite ist jedoch immer maximal
      if(Jl_menue == 3 && (PS4.LStickX() == -128 || PS4.LStickX() == 127))
      {
        mot.rotateRobot(J_gait, RHY_step, J_lam, J_lam_dir, J_step_hight);
        mot.setPosTcp();
      }
      //Translation und eventuell Rotation des Roboters
      else if(L_V_JR > Jr_dz)
      {
        //Translation des Roboters
        mot.translRobot(J_gait, RHY_step, J_kappa, J_step_hight, J_step_width);
        //Rotoation des Roboters
        if((PS4.LStickX() < -Jl_dz || PS4.LStickX() > Jl_dz) && Jl_menue == 3)
        {
          mot.turnRobot(J_gait, RHY_step, J_lam, J_lam_dir);
        }
        mot.setPosTcp();
      }
      //Wenn die Eingabe des Bewegungs-Joysticks unterhalb der Deadzones liegt, 
      //dann soll der Roboter die Neutralposition einnehmen
      else
      {
        //TCPs auf die Neutralstellung setzen
        pos.setNtrPosTcp(); 
        //Rhythmus-Schritte resetten
        RHY_step = 1;  
      } 
      
      //Serial.print("Step: "); 
      //Serial.println(RHY_step); 
      //Serial.println(pos.printPos()); 
      //delay(300);   
      
      //Position setzen
      pos.goToPos(); 
      
      ///////////////////////////
      // Status-LEDs der Beine //
      ///////////////////////////
      
      //RGB-Werte für die Bein-LEDs ermitteln
      //Debug-Modus      
      for(int i = 0; i < A_BEINE; i++){rgb.setRgbDebug(i, pos.getStatus(i), pos.getStatus(i, 0), pos.getStatus(i, 1), pos.getStatus(i, 2), 5);}
      //Bewegungs-Modus 
      //rgb.setRgbMove(50);
      //RGB-Werte für die Bein-LEDs setzen
      led[0] = CRGB(rgb.getR(1), rgb.getG(1), rgb.getB(1));        //Bein 1
      led[1] = CRGB(rgb.getR(0), rgb.getG(0), rgb.getB(0));        //Bein 0
      led[2] = CRGB(rgb.getR(2), rgb.getG(2), rgb.getB(2));        //Bein 2
      led[3] = CRGB(rgb.getR(4), rgb.getG(4), rgb.getB(4));        //Bein 4
      led[4] = CRGB(rgb.getR(5), rgb.getG(5), rgb.getB(5));        //Bein 5
      led[5] = CRGB(rgb.getR(3), rgb.getG(3), rgb.getB(3));        //Bein 3
      //LEDs aktivieren
      FastLED.show();    

      //Zyklus-Zeit updaten
      RHY_time = RHY_NOW_Time;
      //Rhythmus-Schritte aktualisieren
      if(RHY_step == mot.getRoundSteps(J_gait))
      {RHY_step = 1;}
      else
      {RHY_step++;} 
    } 
  } 
  //Wenn kein PS4-Controller verbunden ist, bleibt der Roboter in der Neutralposition
  else
  {
    //Beine in Neutralposition bringen
    pos.setNtrPos();
    pos.goToPos();  
    delay(200);   
  }
}
