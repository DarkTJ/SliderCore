#include <Arduino.h>

#include <TMCStepper.h>
#include <math.h>


#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>
#include <OSCBundle.h>


//////////////////////////////////////////////////////
// WIFI Setup

const char *ssid = "Slider";                // Name des Netzwerks
const char *password = "slideslide";        // Passwort des Netzwerks

WiFiServer server(80);                      // Server Instanz -> ESP als Access Point
WiFiUDP Udp;                                // UDP Instanz um Pakete über UDP zu senden / empfangen

const IPAddress outIp(192, 168, 4, 2);      // IP des Clients
const unsigned int destPort = 9999;          // Incoming-Port des Clients
const unsigned int localPort = 8888;        // Incoming-Ports des Hosts für ankommende Nachrichten


//
///////////////////////////////////////////////////////


///////////////////////////////////////////////////////7
// Stepper Setup

#define STALL_VALUE      20 // [-64..63]

using namespace TMC2130_n;

#define EN_PIN           33 // Enable
#define DIR_PIN          25 // Direction
#define STEP_PIN         26 // Step
#define CS_PIN           15 // Chip select
#define SW_MOSI          13 // Software Master Out Slave In (MOSI)
#define SW_MISO          12 // Software Master In Slave Out (MISO)
#define SW_SCK           14 // Software Slave Clock (SCK)

#define R_SENSE 0.11f       // Match to your driver


//
//////////////////////////////////////


///////////////////////////////////////////////////////
// Global Variables

OSCErrorCode error;
//Variablen:



//Variablen für KeyFrame Toggle
bool KeyFrame_1;
bool KeyFrame_2;
bool KeyFrame_3;
bool KeyFrame_4;
bool KeyFrame_5;

//Variablen für KeyFrame Position
float KeyFramePosition_1;
float KeyFramePosition_2;
float KeyFramePosition_3;
float KeyFramePosition_4;
float KeyFramePosition_5;

//Variablen für KeyFrame Pan
float KeyFramePan_1;
float KeyFramePan_2;
float KeyFramePan_3;
float KeyFramePan_4;
float KeyFramePan_5;

//Variablen für KeyFrame Tilt
float KeyFrameTilt_1;
float KeyFrameTilt_2;
float KeyFrameTilt_3;
float KeyFrameTilt_4;
float KeyFrameTilt_5;

//Variablen für KeyFrame Beschleunigung
float KeyFrameBeschleunigung_1;
float KeyFrameBeschleunigung_2;
float KeyFrameBeschleunigung_3;
float KeyFrameBeschleunigung_4;
float KeyFrameBeschleunigung_5;

//Variablen für KeyFrame Beschleunigung
float KeyFrameGeschwindigkeit_1;
float KeyFrameGeschwindigkeit_2;
float KeyFrameGeschwindigkeit_3;
float KeyFrameGeschwindigkeit_4;
float KeyFrameGeschwindigkeit_5;

//Variablen für KeyFrame Pausendauer
float KeyFramePause_1;
float KeyFramePause_2;
float KeyFramePause_3;
float KeyFramePause_4;
float KeyFramePause_5;

//modus 3 Einlesen
int Runter;
int Hoch;
int Links;
int Rechts;
int Vor;
int Zuruck;

//Modus Weiter Variablen
int bilder;                 //Anzahl der Bilder
int Bildzahl;               //Anzahl der Bilder zwischen zwei Keyframes im modus2
int Wartedauer;             //Zeit zwischen den Bildern

//StepperMovement Variables
bool go;                    //ist der Slider an oder ist der Slider aus? (KANN der Slider sich bewegen) (im gegensatz zu MovingX(OB der Slider sich Bewegt))
bool shaft = true;          //Richtung des Sliders
int Geschwindigkeit;        //Geschwindikeit des Sliders
long aPX = 0;                     //AnfangspunktX
long zPX = 100;                   //ZielpunktX
bool rampingX = false;            //Ob Beschleuningt wird
bool movingX = false;             //Ob Gerade Bewegung stattfindet

//Slider
volatile long SliderPosition;      // volitile damit auch wärend critical vom timer geschrieben werden kann 
volatile float PankopfPosition;    //        
volatile float TiltkopfPosition;   //

long laengeSlider = -1;            // muss erst ausgemessen werden oder aus dem Speicher gelesen. Solange nichts gespeichert ist = -1

long accStep = 1;
int  accSteps = 1;

int speedRamp[6001];

long timeMove;
long timeRamp;

int modus = -2;

int speedX = 5;


//
/////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Stepper und Steppertimer initialisierung:

//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {        //macht steps und aktualisiert die Variable: SliderPosition
  portENTER_CRITICAL_ISR(&timerMux);      // damit auf eine Variable nicht geschrieben wird wärend wir sie ändern ; weil es noInterrupts(); nicht auf dem ESP gibt
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  if (shaft == true) {
    SliderPosition = SliderPosition - 1;
  } else {
    SliderPosition = SliderPosition + 1;
  }
  portEXIT_CRITICAL_ISR(&timerMux);       
 
}

//
///////////////////////////////////////////////////////////////////

//Zur Copy paste der timermodifikation!!
////////////////////
//timerAlarmEnable(timer);                      // start Timer
//timerAlarmDisable(timer);
//timerAlarmWrite(timer, speed, true);
////////////////////


//////////////////////////////////////////////////////////////////////////////////////7+
////
/////// Hardware Code


void moveMotorToLocation(long zielPos, int speed, int acceleration) {    // acceleration 0-10 // speed 0 - 10

  zPX = zielPos;
  aPX = SliderPosition;
  long movementX = zPX - aPX;
  int accMovementX = ((10/acceleration)*480)+600 ;
  accSteps = accMovementX;

  Serial.println(accMovementX);

  if (movementX > 0 ) {

    shaft = false;
    driver.shaft(shaft);
  }else {
    shaft = true;
    driver.shaft(shaft);
  }

  /*if (movementX < (accMovementX*5)){   // check ob die Bewegung lang genug ist für die ganze Bescheunigung 
    
    accMovementX = movementX/20;      // halbe 

  }*/
  
  Serial.println("START");

  timeRamp = 0;
  timeMove = 0;

    for (int i = 1; i <= accMovementX; i++) {

      float x;

      //Serial.println(accMovementX);
      x = i*PI;                                 // richtigen Schritt auswäähle
      //Serial.println(x);           
      x = x/accMovementX;                      //cosWelle in nötige Schritte aufteilen      
      //Serial.println(x);      
      x = 1-cos(x);                           //cos berechnen (dabei den Wert zu +2-0 und nicht -1-+1 machen)
      //Serial.println(x);
      x = cbrt(speed)*x*-65;                  // +2 langsamster Wert (festgelegt)  0* muss den gewollten Wert bringen

      x=int(x)+300;

      speedRamp[i] = int(x);

      timeRamp = timeRamp + int(x)*10;
    }

    timeMove = timeRamp + (abs(movementX)-(accMovementX*10))*speedRamp[accMovementX];

    Serial.print(timeMove);
    Serial.println(" ns Bewegungszeit"); 
      
      
      
      ///////////////////////////////////7
      //DEBUG
      
      /*static int looptimexff = 0;
      if (looptimexff < i) {

        Serial.println(speedRamp[i]);

        x= i+100;

      }*/

      



    Serial.println(speedRamp[1]);

    Serial.println("DONE");

    movingX = true;
    rampingX = true;

    digitalWrite(EN_PIN,LOW);

    timerAlarmEnable(timer);

    Serial.println("START");


}


void changeSpeed() {

  if (rampingX == true) {     //solange noch speed geändert werden muss

    static uint32_t last_time=0;

    uint32_t ns = micros();



    if((ns-last_time) > 2000) { //run every 0.0001s
      last_time = ns;


    //Serial.print(k);
    //Serial.print(" k " + String(shaft) + "  S  ");

    //Serial.println(speedRamp[accStep]);

      timerAlarmWrite(timer, speedRamp[accStep], true);

      accStep = accStep+1 ;

      if (accStep == accSteps) {    // wenn die Beschleinigungsphse vorbei ist

        rampingX = false;
      }

    }
    
  }

  

  


}

void messungX() {

  modus = -2; // auf Nullposition fahren
  driver.en_pwm_mode(false);   // disable stealthcop für stallguard
  shaft = true;
  timerAlarmWrite(timer, 30, true);
  driver.reset();
  digitalWrite(EN_PIN,LOW);
  timerAlarmEnable(timer);  

  //modus 1 Slider Position
  

}


void stallDetect(uint32_t ms)  {

  static uint32_t last_time=0;

  if((ms-last_time) > 50) { //run every 0.05s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();


    

    //Messung

    if (drv_status.sg_result == 0 && modus == -2) {   //zurückfahen (modus -2)
      timerAlarmDisable(timer);
      digitalWrite(EN_PIN,HIGH);
      SliderPosition = -2000;
      modus = -1;
      Serial.print(digitalRead(34));
      driver.reset();
      Serial.print(digitalRead(34));
      shaft = !shaft;
      driver.shaft(shaft);
      Serial.println(" F");
      drv_status.sg_result = 1;
      digitalWrite(EN_PIN,LOW);
      timerAlarmEnable(timer);

    } else if (drv_status.sg_result == 0 && modus == -1) {  // an maximum fahren  (modus -1)
      timerAlarmDisable(timer);
      digitalWrite(EN_PIN,HIGH);
      laengeSlider = SliderPosition - 2000;
      modus = 0;
      driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
      driver.reset();
      Serial.println(laengeSlider);
      shaft = !shaft;
      driver.shaft(shaft);
    
    }

    
  }
}

void detectZiel() {
  /////////////////////////////////
    //Motormovement

    if (modus >= 0) {       // check ob die Zielposition errreicht wurde
      if (movingX == true) {
        if (shaft == false) {
          if (zPX <= SliderPosition) {
            movingX = false;
            timerAlarmDisable(timer);
            digitalWrite(EN_PIN,HIGH);
            accStep = 1;
            Serial.print("STOP");
          }
        }else {
          if (zPX >= SliderPosition) {
            movingX = false;
            timerAlarmDisable(timer);
            digitalWrite(EN_PIN,HIGH);
            accStep = 1;
            Serial.print("STOP2");
          }
        }
      }     
    }

}

void checkGO() {
  if( movingX == true) {
    if (go == false) {

    movingX = false;
    rampingX = false;
    timerAlarmDisable(timer);
    digitalWrite(EN_PIN,HIGH);
    Serial.print("PANIC!!!!!!!");

  }

  }

  

}



void debugOutput(uint32_t ms) {

  static uint32_t last_time=0;

  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();
    
    if (modus < 0) {
      Serial.print(driver.TSTEP());
    Serial.print(" ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.print(driver.cs2rms(drv_status.cs_actual), DEC);
    Serial.print(" ");
    Serial.println(analogRead(34));

    }
    
    if (movingX == true) {
      Serial.print(driver.TSTEP());
      Serial.print(" ");
      Serial.print(drv_status.sg_result, DEC);
      Serial.print(" ");
      Serial.print(driver.cs2rms(drv_status.cs_actual), DEC);
     Serial.print(" ");
      Serial.print(analogRead(34));
     Serial.print(" ");



      //Change speed of Motor (keleinerse zahl = schneller )
    


    
      Serial.print("MovingX: ");
      Serial.print(movingX);
      Serial.print(" PositionX: ");
      Serial.print(SliderPosition);
      Serial.print(" ZP: ");
      Serial.println(zPX);

    }
    
  }
}






////////////7
/////////////
////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////7
//  OSC CODE

//mehrfach verwendete Funktionen-------------------------------------------------------------------------------------------------
/*void UIupdate{

  //modus 0 Slider Position
  OSCMessage msgOUT("/modus_0/slider_fader");
  msgOUT.add(Pankopf);      //Slider Position Variable
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message

  //modus 0 Pan Position
  OSCMessage msgOUT("/modus_0/pan_position");
  msgOUT.add(Pankopf);      //Oan Position Variable
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message

  //modus 0 Tiltkopf
  OSCMessage msgOUT("/modus_0/tilt_encoder");
  msgOUT.add(Pankopf);      //Tilt Position Variable
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message


  //modus 1 Slider Position
  OSCMessage msgOUT("/modus_1/slider_fader");
  msgOUT.add(Pankopf);      //Slider Position Variable
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message

  //modus 1 Pan Position
  OSCMessage msgOUT("/modus_1/pan_position");
  msgOUT.add(Pankopf);      //Oan Position Variable
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message

  //modus 1 Tiltkopf
  OSCMessage msgOUT("/modus_1/tilt_position");
  msgOUT.add(Pankopf);      //Tilt Position Variable
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}
*/



/*void modus(OSCMessage &msg, int addrOffset ) { // In welchem modus sind wir

  modus = msg.getFloat(0);

  if (modus == 1) {
    modus = 0;
  }
  if (modus == 3) {
    modus = 1;
  }
  if (modus == 5) {
    modus = 2;
  }
  if (modus == 7) {
    modus = 3;
  }
  Serial.print("modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);

}
*/

void start(OSCMessage &msg, int addrOffset ) { // Slider Position modus 1

  go = msg.getFloat(0);

  Serial.print("Start? "); //Hier einfügen, was gemacht werden soll
  Serial.println(go);

  OSCMessage msgOUT("/modus_1/slider_fader");
    msgOUT.add(float(SliderPosition)/float(laengeSlider)); 
    Serial.println(float(SliderPosition/laengeSlider));     //Slider Position Variable
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    msgOUT.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT.empty(); // free space occupied by message

    Serial.println(SliderPosition);

  

}

bool recSlider = false;

float recPos;
void checkRecSliderPos(bool check,float pos) {



  static uint32_t last_time=0;

  static float posSaved;

  uint32_t ms = millis();

  if (check == false) {

    last_time = ms;
    posSaved = pos;

  }else  {

    if (ms-last_time > 250 ) {

    long mappedZiel = posSaved*float(laengeSlider);
  
    moveMotorToLocation(mappedZiel,speedX,10);

    Serial.print("Gemappter Wert: ");
    Serial.println(mappedZiel);

    recSlider= false;


  }

  

  }
}

void SliderBewegung(OSCMessage &msg, int addrOffset ) { // Slider Position modus 1

  recSlider = true;
  
  float pos = msg.getFloat(0);

  //OSCMessage msgOUT("/modus_1/slider_fader");    //Hier Knopf Bezeichnung für Ziel einfügen

  Serial.print("Sliderposition = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(pos);

  checkRecSliderPos(false,pos);

}





void Pankopf_Bewegung(OSCMessage &msg, int addrOffset ) {    //Pankopf1

  Pankopf = msg.getFloat(0);
  if (modus == 1) {
    OSCMessage msgOUT("/modus_1/pan_position");
    msgOUT.add(Pankopf);
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    msgOUT.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT.empty(); // free space occupied by message
  }
  if (modus == 0) {
    OSCMessage msgOUT("/modus_0/pan_position");
    msgOUT.add(Pankopf);
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    msgOUT.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT.empty(); // free space occupied by message
  }

  Serial.print("Pankopf = : ");
  Serial.println(Pankopf);






}



void Tiltkopf_Bewegung (OSCMessage &msg, int addrOffset ) { // Slider Position modus 1

  Tiltkopf = msg.getFloat(0);
  //OSCMessage msgOUT("/modus_1/slider_fader");    //Hier Knopf Bezeichnung für Ziel einfügen

  Serial.print("Tiltkopf = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(Tiltkopf);

}



//modus 1 Keyframes----------------------------------------------------------------------------------------------
void Keyframe1(OSCMessage &msg, int addrOffset ) {
  int i = 0;
  KeyFrame_1 = msg.getFloat(0);
  //OSCMessage msgOUT("/modus 1/encoder2");

  Serial.print("Keyframe 1 set? = : ");
  Serial.println(KeyFrame_1);
  if (KeyFrame_1 == 1) {
    i++;
    if (KeyFramePosition_1 == 0 && KeyFramePan_1 == 0 && KeyFrameTilt_1 == 0 && i == 1) {
      KeyFramePosition_1 = SliderPosition;
      KeyFramePan_1 = Pankopf;
      KeyFrameTilt_1 = Tiltkopf;
      Serial.print("Keyframe 1 Position = : ");
      Serial.println(KeyFramePosition_1);
      Serial.print("Keyframe 1 Pan = : ");
      Serial.println(KeyFramePan_1);
      Serial.print("Keyframe 1 Tilt = : ");
      Serial.println(KeyFrameTilt_1);
    }
  }
  if (KeyFrame_1 == 0) {
    i = 0;
    KeyFramePosition_1 = 0;
    KeyFramePan_1 = 0;
    KeyFrameTilt_1 = 0;
    Serial.print("Keyframe 1 Position = : ");
    Serial.println(KeyFramePosition_1);
    Serial.print("Keyframe 1 Pan = : ");
    Serial.println(KeyFramePan_1);
    Serial.print("Keyframe 1 Tilt = : ");
    Serial.println(KeyFrameTilt_1);

  }
}

void ViewKeyFrame1(OSCMessage &msg, int addrOffset ) { //Anzeige des 1. Keyframes
  bool view = msg.getFloat(0);
  //OSCMessage msgOUT("/modus 1/encoder2");

  Serial.print("View Keyframe1?= : ");
  Serial.println(view); {
    if (view == 1) {
      SliderPosition = KeyFramePosition_1;
      Pankopf = KeyFramePan_1;
      Tiltkopf = KeyFrameTilt_1;
    }
  }
}

void BeschleunigungKeyFrame1(OSCMessage &msg, int addrOffset ) { //Beschleunigung des 1. KeyFrames
  KeyFrameBeschleunigung_1 = msg.getFloat(0);

  Serial.print("KeyFrameBeschleunigung_1?= : ");
  Serial.println(KeyFrameBeschleunigung_1);
}


void GeschwindigkeitKeyFrame1(OSCMessage &msg, int addrOffset ) { //Geschwindigkeit des 1. KeyFrames
  KeyFrameGeschwindigkeit_1 = msg.getFloat(0);

  Serial.print("KeyFrameGeschwindigkeit_1?= : ");
  Serial.println(KeyFrameGeschwindigkeit_1);
}


void PauseKeyFrame1(OSCMessage &msg, int addrOffset ) { //Pause des 1. KeyFrames
  KeyFramePause_1 = msg.getFloat(0);

  Serial.print("KeyFramePause_1?= : ");
  Serial.println(KeyFramePause_1);
}


//modus 1 Keyframe 2---------------------------------------------------------------------------------------------------
void Keyframe2(OSCMessage &msg, int addrOffset ) {
  int i = 0;
  KeyFrame_2 = msg.getFloat(0);


  Serial.print("Keyframe 2 set? = : ");
  Serial.println(KeyFrame_2);
  if (KeyFrame_2 == 1) {
    i++;
    if (KeyFramePosition_2 == 0 && KeyFramePan_2 == 0 && KeyFrameTilt_2 == 0 && i == 1) {
      KeyFramePosition_2 = SliderPosition;
      KeyFramePan_2 = Pankopf;
      KeyFrameTilt_2 = Tiltkopf;
      Serial.print("Keyframe 2 Position = : ");
      Serial.println(KeyFramePosition_2);
      Serial.print("Keyframe 2 Pan = : ");
      Serial.println(KeyFramePan_2);
      Serial.print("Keyframe 2 Tilt = : ");
      Serial.println(KeyFrameTilt_2);
    }
  }
  if (KeyFrame_2 == 0) {
    i = 0;
    KeyFramePosition_2 = 0;
    KeyFramePan_2 = 0;
    KeyFrameTilt_2 = 0;
    Serial.print("Keyframe 2 Position = : ");
    Serial.println(KeyFramePosition_2);
    Serial.print("Keyframe 2 Pan = : ");
    Serial.println(KeyFramePan_2);
    Serial.print("Keyframe 2 Tilt = : ");
    Serial.println(KeyFrameTilt_2);

  }
}

void ViewKeyFrame2(OSCMessage &msg, int addrOffset ) { //Anzeige des 2. Keyframes
  bool view = msg.getFloat(0);

  Serial.print("View Keyframe2?= : ");
  Serial.println(view);
  if (view == 1) {
    SliderPosition = KeyFramePosition_2;
    Pankopf = KeyFramePan_2;
    Tiltkopf = KeyFrameTilt_2;
  }
}

void BeschleunigungKeyFrame2(OSCMessage &msg, int addrOffset ) { //Beschleunigung des 2. KeyFrames
  KeyFrameBeschleunigung_2 = msg.getFloat(0);

  Serial.print("KeyFrameBeschleunigung_2?= : ");
  Serial.println(KeyFrameBeschleunigung_2);
}


void GeschwindigkeitKeyFrame2(OSCMessage &msg, int addrOffset ) { //Geschwindigkeit des 2. KeyFrames
  KeyFrameGeschwindigkeit_2 = msg.getFloat(0);

  Serial.print("KeyFrameGeschwindigkeit_2?= : ");
  Serial.println(KeyFrameGeschwindigkeit_2);
}


void PauseKeyFrame2(OSCMessage &msg, int addrOffset ) { //Pause des 2. KeyFrames
  KeyFramePause_2 = msg.getFloat(0);

  Serial.print("KeyFramePause_2?= : ");
  Serial.println(KeyFramePause_2);
}


//modus 1 Keyframe 3---------------------------------------------------------------------------------------------------
void Keyframe3(OSCMessage &msg, int addrOffset ) {
  int i = 0;
  KeyFrame_3 = msg.getFloat(0);


  Serial.print("Keyframe 3 set? = : ");
  Serial.println(KeyFrame_3);
  if (KeyFrame_3 == 1) {
    i++;
    if (KeyFramePosition_3 == 0 && KeyFramePan_3 == 0 && KeyFrameTilt_3 == 0 && i == 1) {
      KeyFramePosition_3 = SliderPosition;
      KeyFramePan_3 = Pankopf;
      KeyFrameTilt_3 = Tiltkopf;
      Serial.print("Keyframe 3 Position = : ");
      Serial.println(KeyFramePosition_3);
      Serial.print("Keyframe 3 Pan = : ");
      Serial.println(KeyFramePan_3);
      Serial.print("Keyframe 3 Tilt = : ");
      Serial.println(KeyFrameTilt_3);
    }
  }
  if (KeyFrame_3 == 0) {
    i = 0;
    KeyFramePosition_3 = 0;
    KeyFramePan_3 = 0;
    KeyFrameTilt_3 = 0;
    Serial.print("Keyframe 3 Position = : ");
    Serial.println(KeyFramePosition_3);
    Serial.print("Keyframe 3 Pan = : ");
    Serial.println(KeyFramePan_3);
    Serial.print("Keyframe 2 Tilt = : ");
    Serial.println(KeyFrameTilt_3);

  }
}

void ViewKeyFrame3(OSCMessage &msg, int addrOffset ) { //Anzeige des 3. Keyframes
  bool view = msg.getFloat(0);
  
  messungX();



  Serial.print("View Keyframe3?= : ");
  Serial.println(view);
  if (view == 1) {
    SliderPosition = KeyFramePosition_3;
    Pankopf = KeyFramePan_3;
    Tiltkopf = KeyFrameTilt_3;
  }
}

void BeschleunigungKeyFrame3(OSCMessage &msg, int addrOffset ) { //Beschleunigung des 3. KeyFrames
  KeyFrameBeschleunigung_3 = msg.getFloat(0);

  Serial.print("KeyFrameBeschleunigung_3?= : ");
  Serial.println(KeyFrameBeschleunigung_3);
}


void GeschwindigkeitKeyFrame3(OSCMessage &msg, int addrOffset ) { //Geschwindigkeit des 3. KeyFrames
  KeyFrameGeschwindigkeit_3 = msg.getFloat(0);

  Serial.print("KeyFrameGeschwindigkeit_3?= : ");
  Serial.println(KeyFrameGeschwindigkeit_3);

  speedX = int(KeyFrameGeschwindigkeit_3); 
}


void PauseKeyFrame3(OSCMessage &msg, int addrOffset ) { //Pause des 3. KeyFrames
  KeyFramePause_3 = msg.getFloat(0);

  Serial.print("KeyFramePause_3?= : ");
  Serial.println(KeyFramePause_3);
}

//modus 1 Keyframe 4---------------------------------------------------------------------------------------------------
void Keyframe4(OSCMessage &msg, int addrOffset ) {
  int i = 0;
  KeyFrame_4 = msg.getFloat(0);


  Serial.print("Keyframe 4 set? = : ");
  Serial.println(KeyFrame_4);
  if (KeyFrame_4 == 1) {
    i++;
    if (KeyFramePosition_4 == 0 && KeyFramePan_4 == 0 && KeyFrameTilt_4 == 0 && i == 1) {
      KeyFramePosition_4 = SliderPosition;
      KeyFramePan_4 = Pankopf;
      KeyFrameTilt_4 = Tiltkopf;
      Serial.print("Keyframe 4 Position = : ");
      Serial.println(KeyFramePosition_4);
      Serial.print("Keyframe 4 Pan = : ");
      Serial.println(KeyFramePan_4);
      Serial.print("Keyframe 4 Tilt = : ");
      Serial.println(KeyFrameTilt_4);
    }
  }
  if (KeyFrame_4 == 0) {
    i = 0;
    KeyFramePosition_4 = 0;
    KeyFramePan_4 = 0;
    KeyFrameTilt_4 = 0;
    Serial.print("Keyframe 4 Position = : ");
    Serial.println(KeyFramePosition_4);
    Serial.print("Keyframe 4 Pan = : ");
    Serial.println(KeyFramePan_3);
    Serial.print("Keyframe 4 Tilt = : ");
    Serial.println(KeyFrameTilt_4);

  }
}

void ViewKeyFrame4(OSCMessage &msg, int addrOffset ) { //Anzeige des 4. KeyFrames
  bool view = msg.getFloat(0);

  Serial.print("View Keyframe4?= : ");
  Serial.println(view);
  if (view == 1) {
    SliderPosition = KeyFramePosition_4;
    Pankopf = KeyFramePan_4;
    Tiltkopf = KeyFrameTilt_4;
  }
}

void BeschleunigungKeyFrame4(OSCMessage &msg, int addrOffset ) { //Beschleunigung des 4. KeyFrames
  KeyFrameBeschleunigung_4 = msg.getFloat(0);

  Serial.print("KeyFrameBeschleunigung_4?= : ");
  Serial.println(KeyFrameBeschleunigung_4);
}


void GeschwindigkeitKeyFrame4(OSCMessage &msg, int addrOffset ) { //Geschwindigkeit des 4. KeyFrames
  KeyFrameGeschwindigkeit_4 = msg.getFloat(0);

  Serial.print("KeyFrameGeschwindigkeit_4?= : ");
  Serial.println(KeyFrameGeschwindigkeit_4);
}


void PauseKeyFrame4(OSCMessage &msg, int addrOffset ) { //Pause des 4. KeyFrames
  KeyFramePause_4 = msg.getFloat(0);

  Serial.print("KeyFramePause_4?= : ");
  Serial.println(KeyFramePause_4);
}

//modus 1 Keyframe 5---------------------------------------------------------------------------------------------------
void Keyframe5(OSCMessage &msg, int addrOffset ) {
  int i = 0;
  KeyFrame_5 = msg.getFloat(0);


  Serial.print("Keyframe 5 set? = : ");
  Serial.println(KeyFrame_5);
  if (KeyFrame_5 == 1) {
    i++;
    if (KeyFramePosition_5 == 0 && KeyFramePan_5 == 0 && KeyFrameTilt_5 == 0 && i == 1) {
      KeyFramePosition_5 = SliderPosition;
      KeyFramePan_5 = Pankopf;
      KeyFrameTilt_5 = Tiltkopf;
      Serial.print("Keyframe 5 Position = : ");
      Serial.println(KeyFramePosition_5);
      Serial.print("Keyframe 5 Pan = : ");
      Serial.println(KeyFramePan_5);
      Serial.print("Keyframe 5 Tilt = : ");
      Serial.println(KeyFrameTilt_5);
    }
  }
  if (KeyFrame_5 == 0) {
    i = 0;
    KeyFramePosition_5 = 0;
    KeyFramePan_5 = 0;
    KeyFrameTilt_5 = 0;
    Serial.print("Keyframe 5 Position = : ");
    Serial.println(KeyFramePosition_5);
    Serial.print("Keyframe 5 Pan = : ");
    Serial.println(KeyFramePan_5);
    Serial.print("Keyframe 5 Tilt = : ");
    Serial.println(KeyFrameTilt_5);

  }
}

void ViewKeyFrame5(OSCMessage &msg, int addrOffset ) { //Anzeige des 5. KeyFrames
  bool view = msg.getFloat(0);

  Serial.print("View Keyframe5?= : ");
  Serial.println(view);
  if (view == 1) {
    SliderPosition = KeyFramePosition_5;
    Pankopf = KeyFramePan_5;
    Tiltkopf = KeyFrameTilt_5;
  }
}

void BeschleunigungKeyFrame5(OSCMessage &msg, int addrOffset ) { //Beschleunigung des 5. KeyFrames
  KeyFrameBeschleunigung_5 = msg.getFloat(0);

  Serial.print("KeyFrameBeschleunigung_5?= : ");
  Serial.println(KeyFrameBeschleunigung_5);
}


void GeschwindigkeitKeyFrame5(OSCMessage &msg, int addrOffset ) { //Geschwindigkeit des 5. KeyFrames
  KeyFrameGeschwindigkeit_5 = msg.getFloat(0);

  Serial.print("KeyFrameGeschwindigkeit_5?= : ");
  Serial.println(KeyFrameGeschwindigkeit_5);
}


void PauseKeyFrame5(OSCMessage &msg, int addrOffset ) { //Pause des 5. KeyFrames
  KeyFramePause_5 = msg.getFloat(0);

  Serial.print("KeyFramePause_5?= : ");
  Serial.println(KeyFramePause_5);
}


//modus 0------------------------------------------------------------------------------------------------

void Position1(OSCMessage &msg, int addrOffset ) { //Pankopf
  KeyFramePause_5 = msg.getFloat(0);

  Serial.print("KeyFramePause_5?= : ");
  Serial.println(KeyFramePause_5);
}


//modus 3-------------------------------------------------------------------------------------------------------------------------------------------------------
void runter(OSCMessage &msg, int addrOffset ) { //runter
  Runter = msg.getFloat(0);

  if (Runter == 1) {
    Serial.println("runter");
  }
}

void hoch(OSCMessage &msg, int addrOffset ) { //hoch
  Hoch = msg.getFloat(0);

  if (Hoch == 1) {
    Serial.println("hoch");
  }
}

void links(OSCMessage &msg, int addrOffset ) { //links
  Links = msg.getFloat(0);

  if (Links == 1) {
    Serial.println("links");
  }
}

void rechts(OSCMessage &msg, int addrOffset ) { //rechts
  Rechts = msg.getFloat(0);

  if (Rechts == 1) {
    Serial.println("rechts");
  }
}

void vor(OSCMessage &msg, int addrOffset ) { //vor
  Vor = msg.getFloat(0);

  if (Vor == 1) {
    Serial.println("vor");
  }
}

void zuruck(OSCMessage &msg, int addrOffset ) { //zuruck
  Zuruck = msg.getFloat(0);

  if (Zuruck == 1) {
    Serial.println("zuruck");
  }
}

//modus_2---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Bilder(OSCMessage &msg, int addrOffset ) { //Bilder
  bilder = msg.getFloat(0);
  
  Serial.print("Bilder: ");
  Serial.println(bilder);
  OSCMessage msgOUT("/modus_2/BilderLabel");
  msgOUT.add(bilder);
  Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}

// MESSAGE RECIVE -------------------------------------------------------------------------------------
void OSCMsgReceive() {

  
  OSCMessage msgIN;
  int size;
  if ((size = Udp.parsePacket()) > 0) {
    Serial.println("MessageREcived");
    while (size--) 
      msgIN.fill(Udp.read());
      if (!msgIN.hasError()) {
/*      msgIN.route("/modus_0", modus); // die 4 Modi mit alle der gleichen Funktion
        msgIN.route("/modus_1", modus);
        msgIN.route("/modus_2", modus);
        msgIN.route("/modus_3", modus);
*/

        msgIN.route("/modus_1/start", start); //Startknöpfe
        msgIN.route("/modus_0/start", start);

        msgIN.route("/modus_1/slider_fader", SliderBewegung); //Positionseinstellungen
        msgIN.route("/modus_1/pan_encoder", Pankopf_Bewegung);
        msgIN.route("/modus_1/tilt_position", Tiltkopf_Bewegung);

        msgIN.route("/modus_0/slider_fader", SliderBewegung); //modus 0 ruft gleiche Funktionen auf wie modus 1
        msgIN.route("/modus_0/pan_encoder", Pankopf_Bewegung);
        msgIN.route("/modus_0/tilt_encoder", Tiltkopf_Bewegung);

        msgIN.route("/modus_0/keyframe_1", Keyframe1);                    //KeyFrame 1, modus 0
        msgIN.route("/modus_0/View_1", ViewKeyFrame1);
        msgIN.route("/modus_0/Geschwindigkeit_1", GeschwindigkeitKeyFrame1);
        msgIN.route("/modus_0/Pause_1", PauseKeyFrame1);

        msgIN.route("/modus_0/keyframe_2", Keyframe2);                    //KeyFrame 2, modus 0
        msgIN.route("/modus_0/View_2", ViewKeyFrame2);
        msgIN.route("/modus_0/Geschwindigkeit_2", GeschwindigkeitKeyFrame2);
        msgIN.route("/modus_0/Pause_2", PauseKeyFrame2);

        //modus_1
        msgIN.route("/modus_1/keyframe_1", Keyframe1);                    //KeyFrame 1, modus 1
        msgIN.route("/modus_1/View_1", ViewKeyFrame1);
        msgIN.route("/modus_1/Geschwindigkeit_1", GeschwindigkeitKeyFrame1);
        msgIN.route("/modus_1/Beschleunigung_1", BeschleunigungKeyFrame1);
        msgIN.route("/modus_1/Pause_1", PauseKeyFrame1);

        msgIN.route("/modus_1/keyframe_2", Keyframe2);                    //KeyFrame 2, modus 1
        msgIN.route("/modus_1/View_2", ViewKeyFrame2);
        msgIN.route("/modus_1/Geschwindigkeit_2", GeschwindigkeitKeyFrame2);
        msgIN.route("/modus_1/Beschleunigung_2", BeschleunigungKeyFrame2);
        msgIN.route("/modus_1/Pause_2", PauseKeyFrame2);

        msgIN.route("/modus_1/keyframe_3", Keyframe3);                    //KeyFrame 3
        msgIN.route("/modus_1/View_3", ViewKeyFrame3);
        msgIN.route("/modus_1/Geschwindigkeit_3", GeschwindigkeitKeyFrame3);
        msgIN.route("/modus_1/Beschleunigung_3", BeschleunigungKeyFrame3);
        msgIN.route("/modus_1/Pause_3", PauseKeyFrame3);

        msgIN.route("/modus_1/keyframe_4", Keyframe4);                    //KeyFrame 4
        msgIN.route("/modus_1/View_4", ViewKeyFrame4);
        msgIN.route("/modus_1/Geschwindigkeit_4", GeschwindigkeitKeyFrame4);
        msgIN.route("/modus_1/Beschleunigung_4", BeschleunigungKeyFrame4);
        msgIN.route("/modus_1/Pause_4", PauseKeyFrame4);

        msgIN.route("/modus_1/keyframe_5", Keyframe5);                    //KeyFrame 5
        msgIN.route("/modus_1/View_5", ViewKeyFrame5);
        msgIN.route("/modus_1/Geschwindigkeit_5", GeschwindigkeitKeyFrame5);
        msgIN.route("/modus_1/Beschleunigung_5", BeschleunigungKeyFrame5);
        msgIN.route("/modus_1/Pause_5", PauseKeyFrame5);


        //modus_2
        msgIN.route("/modus_2/Bilder", Bilder);
        
        //modus_3
        msgIN.route("/modus_3/runter", runter);
        msgIN.route("/modus_3/hoch", hoch);
        msgIN.route("/modus_3/links", links);
        msgIN.route("/modus_3/rechts", rechts);
        msgIN.route("/modus_3/vor", vor);
        msgIN.route("/modus_3/zuruck", zuruck);
      }
    }
  
}







void setup() {
  
  Serial.begin(115200);

  // Server starten + Feedback
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP address: ");
  Serial.println(myIP);
  server.begin();

  // UDP starten + Feedback
  Serial.println("Starting UDP...");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

  


  //timer Setup
  timer = timerBegin(0, 80, true);      //timer, der alle 80ticks der Frequenz tickt = 1ns 
  timerAttachInterrupt(timer, &onTimer, true);  // wenn getriggert wird onTimer() ausgeführt
  timerAlarmWrite(timer, 100, true);            //alle 100 (ns) wird der "Alarm" (also die funktion) ausgeführt

  //Stepper Setup
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  pinMode(34,INPUT);

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables d