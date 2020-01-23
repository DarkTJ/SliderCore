#include <Arduino.h>

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>

#include <OSCBundle.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>

#include <Variablen.h>

const char *ssid = "SliderMasterVersion2";         // Name des Netzwerks
const char *password = "slideslide"; // Passwort des Netzwerks

WiFiServer server(80); // Server Instanz -> ESP als Access Point
WiFiUDP Udp;           // UDP Instanz um Pakete über UDP zu senden / empfangen

const IPAddress outIp(192, 168, 4, 2); // IP des Clients
const unsigned int destPort = 9999;    // Incoming-Port des Clients
const unsigned int localPort = 8888;   // Incoming-Ports des Hosts für ankommende Nachrichten

OSCErrorCode error;



////////////////////////////////////////////////////////////////////////////7
// Stepper setup


#define EN_PINX          33   // Enable Slider Driver
#define EN_PINY          22   // Enable Pan Driver
#define EN_PINZ          21   // Enable  Tilt Driver
#define CS_PINX          15   // Chip select X Achse (Slider)
#define CS_PINY          2    // Chip select Y Achse (Pan)
#define CS_PINZ          4    // Chip select Z Achse (Tilt)

#define SW_MOSI          13   // Software Master Out Slave In (MOSI)
#define SW_MISO          12   // Software Master In Slave Out (MISO)
#define SW_SCK           23   // Software Slave Clock (SCK)

#define STEPX            26
#define DIRX             25

#define STEPY            5
#define DIRY             18

#define STEPZ            27
#define DIRZ             14


using namespace TMC2130_n;

#define STALL_VALUE 20
#define R_SENSE 0.11f 

// Stepper und Steppertimer initialisierung:

//TMC2130Stepper driverX(CS_PINX, R_SENSE);                           // Hardware SPI
TMC2130Stepper driverX(CS_PINX, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driverY(CS_PINY, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverZ(CS_PINZ, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

AccelStepper stepperX(AccelStepper::DRIVER,STEPX,DIRX);  //Slider
AccelStepper stepperY(AccelStepper::DRIVER,STEPY,DIRY);  //Pan
AccelStepper stepperZ(AccelStepper::DRIVER,STEPZ,DIRZ);  //Tilt

void setup()
{
  Serial.begin(115200);

  pinMode(EN_PINX, OUTPUT);
  pinMode(STEPX, OUTPUT);
  pinMode(DIRX, OUTPUT);
  digitalWrite(EN_PINX, LOW);      // Enable driver in hardware
  pinMode(EN_PINY, OUTPUT);
  pinMode(STEPY, OUTPUT);
  pinMode(DIRY, OUTPUT);
  digitalWrite(EN_PINY, LOW);      // Enable driver in hardware
  pinMode(EN_PINZ, OUTPUT);
  pinMode(STEPZ, OUTPUT);
  pinMode(DIRZ, OUTPUT);
  digitalWrite(EN_PINZ, LOW);      // Enable driver in hardware

  // Server starten + Feedback
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP address: ");
  Serial.println(myIP);
  server.begin();

  // UDP starten + Feedback
  Serial.println("Starting UDP...");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

 delay(2500);

 // driverStartup (SPI sollte bis dahin funktionieren)
  driverX.begin(); 
  driverX.toff(5);                 // Enables driver in software
  driverX.rms_current(1000);        // Set motor RMS current
  driverX.microsteps(16);          // Set microsteps to 1/16th
  driverX.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driverX.pwm_autoscale(true);     // Needed for stealthChop

  driverY.begin(); 
  driverY.toff(5);                 // Enables driver in software
  driverY.rms_current(600);        // Set motor RMS current
  driverY.microsteps(16);          // Set microsteps to 1/16th
  driverY.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driverY.pwm_autoscale(true);     // Needed for stealthChop

  driverZ.begin(); 
  driverZ.toff(5);                 // Enables driver in software
  driverZ.rms_current(600);        // Set motor RMS current
  driverZ.microsteps(16);          // Set microsteps to 1/16th
  driverZ.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driverZ.pwm_autoscale(true);     // Needed for stealthChop








  delay(2500);



  MaxSliderPosition = 60000;
  MaxTiltkopf = 30000;
  MaxPankopf = 15000;
        stepperX.setMaxSpeed(50000);
      stepperX.setAcceleration(2500);
            stepperY.setMaxSpeed(50000);
      stepperY.setAcceleration(2500);
            stepperZ.setMaxSpeed(50000);
      stepperZ.setAcceleration(2500);
}

#pragma region Stepper

void sliderAusmessen() {
  modusSlider = -2; // auf Nullposition fahren
  driverX.en_pwm_mode(false);   // disable stealthcop für stallguard  
  driverX.reset();
  stepperX.setAcceleration(5000);
  stepperX.setMaxSpeed(50000);
  stepperX.move(-100000000);

}

void stallDetect(uint32_t ms)  {

  static uint32_t last_time=0;

  if((ms-last_time) > 50) { //run every 0.05s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driverX.DRV_STATUS();


    

    //Messung

    if (drv_status.sg_result == 0 && modus == -2) {   //zurückfahen (modus -2)
      

      stepperX.setCurrentPosition(0);

      modus = -1;
      driverX.reset();
      Serial.println(" F ");
      drv_status.sg_result = 1;
      stepperX.moveTo(100000);

    } else if (drv_status.sg_result == 0 && modus == -1) {  // an maximum fahren  (modus -1)
      
      stepperX.stop();
      MaxSliderPosition = stepperX.currentPosition();
      modus = 0;
      driverX.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
      driverX.reset();
      Serial.println(MaxSliderPosition);
    
    }

    
  }
}

void debugOutput() {

  static uint32_t last_time=0;

  if((millis()-last_time) > 100) { //run every 0.1s
    last_time = millis();

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driverX.DRV_STATUS();
    
    
    
    if (true) {
      Serial.print(driverX.TSTEP());
      Serial.print(" ");
      Serial.print(drv_status.sg_result, DEC);
      Serial.print(" ");
      Serial.print(driverX.cs2rms(drv_status.cs_actual), DEC);
      Serial.print(" ");
      Serial.print(driverX.IOIN(), BIN);
      Serial.print(" ");
      Serial.print(driverX.sfilt());
      Serial.print(" ");
      Serial.println(stepperX.distanceToGo());



      //Change speed of Motor (keleinerse zahl = schneller )
    

    }
    
  }
}


#pragma endregion Stepper


#pragma region 


void SendOSCMessage(OSCMessage msgOUT, float wert){
    msgOUT.add(wert);
//    Serial.println("Send Float");
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    msgOUT.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT.empty(); // free space occupied by message
}
void SendOSCMessage(OSCMessage msgOUT, String wert){
    OSCBundle bundle;
    bundle.add(msgOUT).add(wert);
    Serial.println("Send String");
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    bundle.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    bundle.empty(); // free space occupied by message
    msgOUT.empty();
}
void SendOSCMessage(OSCMessage msgOUT, int wert){
    msgOUT.add(wert);
//    Serial.println("Send Int");
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    msgOUT.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT.empty(); // free space occupied by message
}


void Modus0(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 0;
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}
void Modus1(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 1;
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}
void Modus2(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 2;
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}
void Modus3(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 3;
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}

void start(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  bool go_ = msg.getFloat(0);

  if (go_ == 1) {
    goModus = modus;
    go = 1;
    SendOSCMessage("/modus_0/start_label/color","red");
    //stepperStart();
  } else {
    go = 0;
    SendOSCMessage("/modus_0/start_label/color","green");
    //stepperStop();
  }

  Serial.print("Start? "); 
  Serial.println(go);
}

void Reverse(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  reversemsg = msg.getFloat(0);
  Serial.print("reverse? "); 
  Serial.println(reversemsg);
}

void SliderBewegung(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1

  stepperX.moveTo(MaxSliderPosition*msg.getFloat(0));
  Serial.print("Sliderposition = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(SliderPosition);
}

void Pankopf_Bewegung(OSCMessage &msg, int addrOffset)
{ //Pankopf1

  stepperY.move(10*msg.getFloat(0));

  Serial.print("Pankopf = : ");
  Serial.println(Pankopf);
}

void Tiltkopf_Bewegung(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1

  Tiltkopf = msg.getFloat(0);

  stepperZ.moveTo(MaxTiltkopf*msg.getFloat(0));
  //OSCMessage msgOUT("/modus_1/slider_fader");    //Hier Knopf Bezeichnung für Ziel einfügen

  Serial.print("Tiltkopf = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(Tiltkopf);
}

//Keyframes 1----------------------------------------------------------------------------------------------
void Keyframe1(OSCMessage &msg, int addrOffset)
{
  int i = 0;
  if (modus == 0)
  {
    KeyFrame_0_1 = msg.getFloat(0);
    Serial.print("Keyframe 1 set? = : ");
    Serial.println(KeyFrame_0_1);
    if (KeyFrame_0_1 == 1)
    {
      i++;
      if (KeyFramePosition_0_1 == 0 && KeyFramePan_0_1 == 0 && KeyFrameTilt_0_1 == 0 && i == 1)
      {
        KeyFramePosition_0_1 = SliderPosition;
        KeyFramePan_0_1 = Pankopf;
        KeyFrameTilt_0_1 = Tiltkopf;
        Serial.print("Keyframe 0 1 Position = : ");
        Serial.println(KeyFramePosition_0_1);
        Serial.print("Keyframe 0 1 Pan = : ");
        Serial.println(KeyFramePan_0_1);
        Serial.print("Keyframe 0 1 Tilt = : ");
        Serial.println(KeyFrameTilt_0_1);
      }
    }
  }
  if (KeyFrame_0_1 == 0)
  {
    i = 0;
    KeyFramePosition_0_1 = 0;
    KeyFramePan_0_1 = 0;
    KeyFrameTilt_0_1 = 0;
    Serial.print("Keyframe 0 1 Position = : ");
    Serial.println(KeyFramePosition_0_1);
    Serial.print("Keyframe 0 1 Pan = : ");
    Serial.println(KeyFramePan_0_1);
    Serial.print("Keyframe 0 1 Tilt = : ");
    Serial.println(KeyFrameTilt_0_1);
  }

  if (modus == 1)
  {
    KeyFrame_1_1 = msg.getFloat(0);
    Serial.print("Keyframe 1 set? = : ");
    Serial.println(KeyFrame_1_1);
    if (KeyFrame_1_1 == 1)
    {
      i++;
      if (KeyFramePosition_1_1 == 0 && KeyFramePan_1_1 == 0 && KeyFrameTilt_1_1 == 0 && i == 1)
      {
        KeyFramePosition_1_1 = SliderPosition;
        KeyFramePan_1_1 = Pankopf;
        KeyFrameTilt_1_1 = Tiltkopf;
        Serial.print("Keyframe 1 1 Position = : ");
        Serial.println(KeyFramePosition_1_1);
        Serial.print("Keyframe 1 1 Pan = : ");
        Serial.println(KeyFramePan_1_1);
        Serial.print("Keyframe 1 1 Tilt = : ");
        Serial.println(KeyFrameTilt_1_1);
      }
    }
    if (KeyFrame_1_1 == 0)
    {
      i = 0;
      KeyFramePosition_1_1 = 0;
      KeyFramePan_1_1 = 0;
      KeyFrameTilt_1_1 = 0;
      Serial.print("Keyframe 1 1 Position = : ");
      Serial.println(KeyFramePosition_1_1);
      Serial.print("Keyframe 1 1 Pan = : ");
      Serial.println(KeyFramePan_1_1);
      Serial.print("Keyframe 1 1 Tilt = : ");
      Serial.println(KeyFrameTilt_1_1);
    }
  }

  if (modus == 2)
  {
    KeyFrame_2_1 = msg.getFloat(0);
    Serial.print("Keyframe 2 1 set? = : ");
    Serial.println(KeyFrame_2_1);
    if (KeyFrame_2_1 == 1)
    {
      i++;
      if (KeyFramePosition_2_1 == 0 && KeyFramePan_2_1 == 0 && KeyFrameTilt_2_1 == 0 && i == 1)
      {
        KeyFramePosition_2_1 = SliderPosition;
        KeyFramePan_2_1 = Pankopf;
        KeyFrameTilt_2_1 = Tiltkopf;
        Serial.print("Keyframe 2 1 Position = : ");
        Serial.println(KeyFramePosition_2_1);
        Serial.print("Keyframe 2 1 Pan = : ");
        Serial.println(KeyFramePan_2_1);
        Serial.print("Keyframe 2 1 Tilt = : ");
        Serial.println(KeyFrameTilt_2_1);
      }
    }
    if (KeyFrame_2_1 == 0)
    {
      i = 0;
      KeyFramePosition_2_1 = 0;
      KeyFramePan_2_1 = 0;
      KeyFrameTilt_2_1 = 0;
      Serial.print("Keyframe 2 1 Position = : ");
      Serial.println(KeyFramePosition_2_1);
      Serial.print("Keyframe 2 1 Pan = : ");
      Serial.println(KeyFramePan_2_1);
      Serial.print("Keyframe 2 1 Tilt = : ");
      Serial.println(KeyFrameTilt_2_1);
    }
  }
}

void ViewKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Anzeige des 1. Keyframes
  bool view = msg.getFloat(0);
  //OSCMessage msgOUT("/Modus 1/encoder2");

  if (modus == 0)
  {
    Serial.print("View Keyframe 0 1?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_0_1);
      stepperY.moveTo(KeyFramePan_0_1);
      stepperZ.moveTo(KeyFrameTilt_0_1);
    }
  }

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 1?= : ");
    Serial.println(view);
    if (view == 1)
    {
      SliderPosition = KeyFramePosition_1_1;
      Pankopf = KeyFramePan_1_1;
      Tiltkopf = KeyFrameTilt_1_1;
    }
  }

  if (modus == 2)
  {
    Serial.print("View Keyframe 2 1?= : ");
    Serial.println(view);
    if (view == 1)
    {
      SliderPosition = KeyFramePosition_2_1;
      Pankopf = KeyFramePan_2_1;
      Tiltkopf = KeyFrameTilt_2_1;
    }
  }
}

void BeschleunigungKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 1. KeyFrames

  if (modus == 0)
  {
    KeyFrameBeschleunigung_0_1 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_0_1?= : ");
    Serial.println(KeyFrameBeschleunigung_0_1);
  }

  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_1 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_1_1?= : ");
    Serial.println(KeyFrameBeschleunigung_1_1);
  }

  if (modus == 2)
  {
    KeyFrameBeschleunigung_2_1 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_2_1?= : ");
    Serial.println(KeyFrameBeschleunigung_2_1);
  }
}

void DauerKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 1. KeyFrames
  if (modus == 0)
  {
    KeyFrameDauer_0_1 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 0 1?= : ");
    Serial.println(KeyFrameDauer_0_1);
    SendOSCMessage("/modus_0/dauer_anzeige_k1",String(KeyFrameDauer_0_1));
  }
  if (modus == 1)
  {
    KeyFrameDauer_1_1 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 1 1?= : ");
    Serial.println(KeyFrameDauer_1_1);
    SendOSCMessage("/modus_1/dauer_anzeige_1",String(KeyFrameDauer_1_1));  
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_1 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 2 1?= : ");
    Serial.println(KeyFrameDauer_2_1);
    SendOSCMessage("/modus_2/dauer_anzeige_1",String(KeyFrameDauer_2_1));  
  }
}

void PauseKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Pause des 1. KeyFrames
  if (modus == 0)
  {
    KeyFramePause_0_1 = msg.getFloat(0);
    Serial.print("KeyFramePause 0 1?= : ");
    Serial.println(KeyFramePause_0_1);
    SendOSCMessage("/modus_0/pause_anzeige_k2",String(KeyFrameDauer_0_1));  
  }
  if (modus == 1)
  {
    KeyFramePause_1_1 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 1?= : ");
    Serial.println(KeyFramePause_1_1);
    SendOSCMessage("/modus_1/pause_anzeige_2",String(KeyFrameDauer_1_1));  
  }
  if (modus == 2)
  {
    KeyFramePause_2_1 = msg.getFloat(0);
    Serial.print("KeyFramePause 2 1?= : ");
    Serial.println(KeyFramePause_2_1);
    SendOSCMessage("/modus_2/pause_anzeige_2",String(KeyFrameDauer_2_1));  
  }
}

// Keyframe 2---------------------------------------------------------------------------------------------------
void Keyframe2(OSCMessage &msg, int addrOffset)
{
  int i = 0;
  if (modus == 0)
  {
    KeyFrame_0_2 = msg.getFloat(0);
    Serial.print("Keyframe 0 2 set? = : ");
    Serial.println(KeyFrame_0_2);
    if (KeyFrame_0_2 == 1)
    {
      i++;
      if (KeyFramePosition_0_2 == 0 && KeyFramePan_0_2 == 0 && KeyFrameTilt_0_2 == 0 && i == 1)
      {
        KeyFramePosition_0_2 = SliderPosition;
        KeyFramePan_0_2 = Pankopf;
        KeyFrameTilt_0_2 = Tiltkopf;
        Serial.print("Keyframe 0 2 Position = : ");
        Serial.println(KeyFramePosition_0_2);
        Serial.print("Keyframe 0 2 Pan = : ");
        Serial.println(KeyFramePan_0_2);
        Serial.print("Keyframe 0 2 Tilt = : ");
        Serial.println(KeyFrameTilt_0_2);
      }
    }
    if (KeyFrame_0_2 == 0)
    {
      i = 0;
      KeyFramePosition_0_2 = 0;
      KeyFramePan_0_2 = 0;
      KeyFrameTilt_0_2 = 0;
      Serial.print("Keyframe 0 2 Position = : ");
      Serial.println(KeyFramePosition_0_2);
      Serial.print("Keyframe 0 2 Pan = : ");
      Serial.println(KeyFramePan_0_2);
      Serial.print("Keyframe 0 2 Tilt = : ");
      Serial.println(KeyFrameTilt_0_2);
    }
  }

  if (modus == 1)
  {
    KeyFrame_1_2 = msg.getFloat(0);
    Serial.print("Keyframe 1  2 set? = : ");
    Serial.println(KeyFrame_1_2);
    if (KeyFrame_1_2 == 1)
    {
      i++;
      if (KeyFramePosition_1_2 == 0 && KeyFramePan_1_2 == 0 && KeyFrameTilt_1_2 == 0 && i == 1)
      {
        KeyFramePosition_1_2 = SliderPosition;
        KeyFramePan_1_2 = Pankopf;
        KeyFrameTilt_1_2 = Tiltkopf;
        Serial.print("Keyframe 1 2 Position = : ");
        Serial.println(KeyFramePosition_1_2);
        Serial.print("Keyframe 1 2 Pan = : ");
        Serial.println(KeyFramePan_1_2);
        Serial.print("Keyframe 1 2 Tilt = : ");
        Serial.println(KeyFrameTilt_1_2);
      }
    }

    if (KeyFrame_1_2 == 0)
    {
      i = 0;
      KeyFramePosition_1_2 = 0;
      KeyFramePan_1_2 = 0;
      KeyFrameTilt_1_2 = 0;
      Serial.print("Keyframe 1 2 Position = : ");
      Serial.println(KeyFramePosition_1_2);
      Serial.print("Keyframe 1 2 Pan = : ");
      Serial.println(KeyFramePan_1_2);
      Serial.print("Keyframe 1 2 Tilt = : ");
      Serial.println(KeyFrameTilt_1_2);
    }
  }

  if (modus == 2)
  {
    KeyFrame_2_2 = msg.getFloat(0);
    Serial.print("Keyframe 2 2 set? = : ");
    Serial.println(KeyFrame_2_2);
    if (KeyFrame_2_2 == 1)
    {
      i++;
      if (KeyFramePosition_2_2 == 0 && KeyFramePan_2_2 == 0 && KeyFrameTilt_2_2 == 0 && i == 1)
      {
        KeyFramePosition_2_2 = SliderPosition;
        KeyFramePan_2_2 = Pankopf;
        KeyFrameTilt_2_2 = Tiltkopf;
        Serial.print("Keyframe 2 2 Position = : ");
        Serial.println(KeyFramePosition_2_2);
        Serial.print("Keyframe 2 2 Pan = : ");
        Serial.println(KeyFramePan_2_2);
        Serial.print("Keyframe 2 2 Tilt = : ");
        Serial.println(KeyFrameTilt_2_2);
      }
    }
    if (KeyFrame_2_2 == 0)
    {
      i = 0;
      KeyFramePosition_2_2 = 0;
      KeyFramePan_2_2 = 0;
      KeyFrameTilt_2_2 = 0;
      Serial.print("Keyframe 2 2 Position = : ");
      Serial.println(KeyFramePosition_2_2);
      Serial.print("Keyframe 2 2 Pan = : ");
      Serial.println(KeyFramePan_2_2);
      Serial.print("Keyframe 2 2 Tilt = : ");
      Serial.println(KeyFrameTilt_2_2);
    }
  }
}

void ViewKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Anzeige des 2. Keyframes
  bool view = msg.getFloat(0);

  if (modus == 0)
  {
    Serial.print("View Keyframe 0 2?= : ");
    Serial.println(view);
    if (view == 1)
    {
      SliderPosition = KeyFramePosition_0_2;
      Pankopf = KeyFramePan_0_2;
      Tiltkopf = KeyFrameTilt_0_2;
    }
  }

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 2?= : ");
    Serial.println(view);
    {
      if (view == 1)
      {
        SliderPosition = KeyFramePosition_1_2;
        Pankopf = KeyFramePan_1_2;
        Tiltkopf = KeyFrameTilt_1_2;
      }
    }
  }

  if (modus == 2)
  {
    Serial.print("View Keyframe 2 2?= : ");
    Serial.println(view);
    {
      if (view == 1)
      {
        SliderPosition = KeyFramePosition_2_2;
        Pankopf = KeyFramePan_2_2;
        Tiltkopf = KeyFrameTilt_2_2;
      }
    }
  }
}

void BeschleunigungKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 2. KeyFrames
  if (modus == 0)
  {
    KeyFrameBeschleunigung_0_2 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_0_2?= : ");
    Serial.println(KeyFrameBeschleunigung_0_2);
  }

  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_2 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_1_2?= : ");
    Serial.println(KeyFrameBeschleunigung_1_2);
  }

  if (modus == 2)
  {
    KeyFrameBeschleunigung_2_2 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_2_2?= : ");
    Serial.println(KeyFrameBeschleunigung_2_2);
  }
}

void DauerKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 2. KeyFrames
  if (modus == 0)
  {
    KeyFrameDauer_0_2 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 0 2?= : ");
    Serial.println(KeyFrameDauer_0_2);
    SendOSCMessage("/modus_0/dauer_anzeige_k2",String(KeyFrameDauer_0_2));  
    
  }
  if (modus == 1)
  {
    KeyFrameDauer_1_2 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 1 2?= : ");
    Serial.println(KeyFrameDauer_1_2);
    SendOSCMessage("/modus_1/dauer_anzeige_2",String(KeyFrameDauer_1_2));  
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_2 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 2 2?= : ");
    Serial.println(KeyFrameDauer_2_2);
    SendOSCMessage("/modus_2/dauer_anzeige_2",String(KeyFrameDauer_2_2));  
  }
}

void PauseKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Pause des 2. KeyFrames
  if (modus == 0)
  {
    KeyFramePause_0_2 = msg.getFloat(0);
    Serial.print("KeyFramePause 0 2?= : ");
    Serial.println(KeyFramePause_0_2);
    SendOSCMessage("/modus_0/pause_anzeige_k2",String(KeyFramePause_0_2));  
  }
  if (modus == 1)
  {
    KeyFramePause_1_2 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 2?= : ");
    Serial.println(KeyFramePause_1_2);
    SendOSCMessage("/modus_1/pause_anzeige_2",String(KeyFramePause_1_2));  
  }
  if (modus == 2)
  {
    KeyFramePause_2_2 = msg.getFloat(0);
    Serial.print("KeyFramePause 2 2?= : ");
    Serial.println(KeyFramePause_2_2);
    SendOSCMessage("/modus_2/pause_anzeige_2",String(KeyFramePause_2_2));  
  }
}

//Modus 1 Keyframe 3---------------------------------------------------------------------------------------------------
void Keyframe3(OSCMessage &msg, int addrOffset)
{
  int i = 0;

  if (modus == 1)
  {
    KeyFrame_1_3 = msg.getFloat(0);
    Serial.print("Keyframe 1  3 set? = : ");
    Serial.println(KeyFrame_1_3);
    if (KeyFrame_1_3 == 1)
    {
      i++;
      if (KeyFramePosition_1_3 == 0 && KeyFramePan_1_3 == 0 && KeyFrameTilt_1_3 == 0 && i == 1)
      {
        KeyFramePosition_1_3 = SliderPosition;
        KeyFramePan_1_3 = Pankopf;
        KeyFrameTilt_1_3 = Tiltkopf;
        Serial.print("Keyframe 1 3 Position = : ");
        Serial.println(KeyFramePosition_1_3);
        Serial.print("Keyframe 1 3 Pan = : ");
        Serial.println(KeyFramePan_1_3);
        Serial.print("Keyframe 1 3 Tilt = : ");
        Serial.println(KeyFrameTilt_1_3);
      }
    }
    if (KeyFrame_1_3 == 0)
    {
      i = 0;
      KeyFramePosition_1_3 = 0;
      KeyFramePan_1_3 = 0;
      KeyFrameTilt_1_3 = 0;
      Serial.print("Keyframe 1 3 Position = : ");
      Serial.println(KeyFramePosition_1_3);
      Serial.print("Keyframe 1 3 Pan = : ");
      Serial.println(KeyFramePan_1_3);
      Serial.print("Keyframe 1 3 Tilt = : ");
      Serial.println(KeyFrameTilt_1_3);
    }
  }

  if (modus == 2)
  {
    KeyFrame_2_3 = msg.getFloat(0);
    Serial.print("Keyframe 2 3 set? = : ");
    Serial.println(KeyFrame_2_3);
    if (KeyFrame_2_3 == 1)
    {
      i++;
      if (KeyFramePosition_2_3 == 0 && KeyFramePan_2_3 == 0 && KeyFrameTilt_2_3 == 0 && i == 1)
      {
        KeyFramePosition_2_3 = SliderPosition;
        KeyFramePan_2_3 = Pankopf;
        KeyFrameTilt_2_3 = Tiltkopf;
        Serial.print("Keyframe 2 3 Position = : ");
        Serial.println(KeyFramePosition_2_3);
        Serial.print("Keyframe 2 3 Pan = : ");
        Serial.println(KeyFramePan_2_3);
        Serial.print("Keyframe 2 3 Tilt = : ");
        Serial.println(KeyFrameTilt_2_3);
      }
    }
    if (KeyFrame_2_3 == 0)
    {
      i = 0;
      KeyFramePosition_2_3 = 0;
      KeyFramePan_2_3 = 0;
      KeyFrameTilt_2_3 = 0;
      Serial.print("Keyframe 2 3 Position = : ");
      Serial.println(KeyFramePosition_2_3);
      Serial.print("Keyframe 2 3 Pan = : ");
      Serial.println(KeyFramePan_2_3);
      Serial.print("Keyframe 2 3 Tilt = : ");
      Serial.println(KeyFrameTilt_2_3);
    }
  }
}

void ViewKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Anzeige des 3. Keyframes
  bool view = msg.getFloat(0);

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 3?= : ");
    Serial.println(view);
    if (view == 1)
    {
      SliderPosition = KeyFramePosition_1_3;
      Pankopf = KeyFramePan_1_3;
      Tiltkopf = KeyFrameTilt_1_3;
    }
  }

  if (modus == 2)
  {
    Serial.print("View Keyframe 2 3?= : ");
    Serial.println(view);
    if (view == 1)
    {
      SliderPosition = KeyFramePosition_2_3;
      Pankopf = KeyFramePan_2_3;
      Tiltkopf = KeyFrameTilt_2_3;
    }
  }
}

void BeschleunigungKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 3. KeyFrames

  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_3 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_1_3?= : ");
    Serial.println(KeyFrameBeschleunigung_1_3);
  }

  if (modus == 2)
  {
    KeyFrameBeschleunigung_2_3 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_2_3?= : ");
    Serial.println(KeyFrameBeschleunigung_2_3);
  }
}

void DauerKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 3. KeyFrames
  if (modus == 1)
  {
    KeyFrameDauer_1_3 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 1 3?= : ");
    Serial.println(KeyFrameDauer_1_3);
    SendOSCMessage("/modus_1/dauer_anzeige_3",int(KeyFrameDauer_1_3));  
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_3 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 2 3?= : ");
    Serial.println(KeyFrameDauer_2_3);
    SendOSCMessage("/modus_2/dauer_anzeige_3",int(KeyFrameDauer_2_3));  
  }
  }


void PauseKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Pause des 3. KeyFrames
  if (modus == 1)
  {
    KeyFramePause_1_3 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 3?= : ");
    Serial.println(KeyFramePause_1_3);
    SendOSCMessage("/modus_1/pause_anzeige_3",String(KeyFramePause_1_3));  
  }
  
  if (modus == 2)
  {
    KeyFramePause_2_3 = msg.getFloat(0);
    Serial.print("KeyFramePause 2 3?= : ");
    Serial.println(KeyFramePause_2_3);
    SendOSCMessage("/modus_2/pause_anzeige_3",String(KeyFramePause_2_3));  
  }
}

//Modus 1 Keyframe 4---------------------------------------------------------------------------------------------------
void Keyframe4(OSCMessage &msg, int addrOffset)
{
  int i = 0;
  KeyFrame_1_4 = msg.getFloat(0);

  Serial.print("Keyframe 1 4 set? = : ");
  Serial.println(KeyFrame_1_4);
  if (KeyFrame_1_4 == 1)
  {
    i++;
    if (KeyFramePosition_1_4 == 0 && KeyFramePan_1_4 == 0 && KeyFrameTilt_1_4 == 0 && i == 1)
    {
      KeyFramePosition_1_4 = SliderPosition;
      KeyFramePan_1_4 = Pankopf;
      KeyFrameTilt_1_4 = Tiltkopf;
      Serial.print("Keyframe 1 4 Position = : ");
      Serial.println(KeyFramePosition_1_4);
      Serial.print("Keyframe 1 4 Pan = : ");
      Serial.println(KeyFramePan_1_4);
      Serial.print("Keyframe 1 4 Tilt = : ");
      Serial.println(KeyFrameTilt_1_4);
    }
  }
  if (KeyFrame_1_4 == 0)
  {
    i = 0;
    KeyFramePosition_1_4 = 0;
    KeyFramePan_1_4 = 0;
    KeyFrameTilt_1_4 = 0;
    Serial.print("Keyframe 1 4 Position = : ");
    Serial.println(KeyFramePosition_1_4);
    Serial.print("Keyframe 1 4 Pan = : ");
    Serial.println(KeyFramePan_1_4);
    Serial.print("Keyframe 1 4 Tilt = : ");
    Serial.println(KeyFrameTilt_1_4);
  }
}

void ViewKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Anzeige des 4. KeyFrames
  bool view = msg.getFloat(0);

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 4?= : ");
    Serial.println(view);
    if (view == 1)
    {
      SliderPosition = KeyFramePosition_1_4;
      Pankopf = KeyFramePan_1_4;
      Tiltkopf = KeyFrameTilt_1_4;
    }
  }
}

void BeschleunigungKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 4. KeyFrames
  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_4 = msg.getFloat(0);
    Serial.print("KeyFrameBeschleunigung_1_4?= : ");
    Serial.println(KeyFrameBeschleunigung_1_4);
  }
}

void DauerKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 4. KeyFrames
  if (modus == 1)
  {
    KeyFrameDauer_1_4 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 1 4?= : ");
    Serial.println(KeyFrameDauer_1_4);
  }
}

void PauseKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Pause des 4. KeyFrames
  if (modus == 1)
  {
    KeyFramePause_1_4 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 4?= : ");
    Serial.println(KeyFramePause_1_4);
    SendOSCMessage("/modus_1/pause_anzeige_4",String(KeyFramePause_1_4));  
  }
}
//Modus_2---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Bilder(OSCMessage &msg, int addrOffset)
{ //Bilder
  bilder = msg.getFloat(0);

  Serial.print("Bilder: ");
  Serial.println(bilder);
  SendOSCMessage("/modus_2/BilderLabel",bilder);
}

void FPS(OSCMessage &msg, int addrOffset)
{ 
  fps = msg.getFloat(0);

  Serial.print("fps: ");
  Serial.println(fps);
  SendOSCMessage("/modus_2/fps_anzeige",fps);
}

void Sequenzdauer1(OSCMessage &msg, int addrOffset) //Schickt Variable Sequenzdauer1  
{ 
   //!!!Hier Rechnung einfügen
  Serial.print("sequenzdauer1 : ");
  Serial.println(sequenzdauer1);
  SendOSCMessage("/modus_2/sequenzdauer_anzeige_1","lol");
}

void Sequenzdauer2(OSCMessage &msg, int addrOffset)  //Schickt Variable Sequenzdauer1  
{ 
  //!!!Hier Rechnung einfügen
  Serial.print("sequenzdauer2 : ");
  Serial.println(sequenzdauer2);
  SendOSCMessage("/modus_2/sequenzdauer_anzeige_2","lol" );
}

void Intervall1(OSCMessage &msg, int addrOffset){
    intervall1 = msg.getFloat(0);
    Serial.print("Intervall1 1?= : ");
    Serial.println(intervall1);
    SendOSCMessage("/modus_2/intervall_anzeige_1","lol"); 
}

void Intervall2(OSCMessage &msg, int addrOffset){
    intervall2 = msg.getFloat(0);
    Serial.print("Intervall2 1?= : ");
    Serial.println(intervall2);
    SendOSCMessage("/modus_2/intervall_anzeige_2","lol"); 
}

//Modus 3----------------------------------------------------------------------------------------------------------------------------------------------------
void Rotation(OSCMessage &msg, int addrOffset){
    rotation = msg.getFloat(0);
    Serial.print("Rotation?= : ");
    Serial.println(rotation);

}


// Settings--------------------------------------------------------------------------------------------------------------------------------------------

void SliderEnable(OSCMessage &msg, int addrOffset){
    slider_enable = msg.getFloat(0);
    digitalWrite(EN_PINX,slider_enable);
    Serial.print("Slider Enable?= : ");
    Serial.println(slider_enable);
}

void PanEnable(OSCMessage &msg, int addrOffset){
    pan_enable = msg.getFloat(0);
    digitalWrite(EN_PINY,pan_enable);
    Serial.print("Pan Enable?= : ");
    Serial.println(pan_enable);
}

void TiltEnable(OSCMessage &msg, int addrOffset){
    tilt_enable = msg.getFloat(0);
    digitalWrite(EN_PINZ,tilt_enable);
    Serial.print("Tilt Enable?= : ");
    Serial.println(tilt_enable);
}

void Kalibrieren(OSCMessage &msg, int addrOffset){
    kalibrieren = msg.getFloat(0);
    Serial.print("Kalibrieren?= : ");
    Serial.println(kalibrieren);
}

void Reset(OSCMessage &msg, int addrOffset){
    reset = msg.getFloat(0);
    Serial.print("Reset= : ");
    Serial.println(reset);
}

void OSCMsgReceive()
{
  OSCMessage msgIN;
  int size;
  if ((size = Udp.parsePacket()) > 0)
  {
    while (size--)
      msgIN.fill(Udp.read());
    if (!msgIN.hasError())
    {
      msgIN.route("/modus_0/s", Modus0); // die 4 Modi mit alle der gleichen Funktion
      msgIN.route("/modus_1/s", Modus1);
      msgIN.route("/modus_2/s", Modus2);
      msgIN.route("/modus_3/s", Modus3);



      msgIN.route("/start", start); //Startknöpfe

       msgIN.route("/slider_fader", SliderBewegung); //modus 0 ruft gleiche Funktionen auf wie modus 1
    msgIN.route("/pan_encoder", Pankopf_Bewegung);
      msgIN.route("/tilt_position", Tiltkopf_Bewegung);


      //Modus_0
      msgIN.route("/modus_0/keyframe_1", Keyframe1); //KeyFrame 1, Modus 0
      msgIN.route("/modus_0/view_1", ViewKeyFrame1);
      msgIN.route("/modus_0/dauer_fader_k1", DauerKeyFrame1);
      msgIN.route("/modus_0/pause_fader_k1", PauseKeyFrame1);

      msgIN.route("/modus_0/keyframe_2", Keyframe2); //KeyFrame 2, Modus 0
      msgIN.route("/modus_0/view_2", ViewKeyFrame2);
      msgIN.route("/modus_0/dauer_fader_k2", DauerKeyFrame2);
      msgIN.route("/modus_0/pause_fader_k2", PauseKeyFrame2);

      //Modus_1
      msgIN.route("/modus_1/reverse_toggle", Reverse);
      msgIN.route("/modus_1/keyframe_1", Keyframe1); //KeyFrame 1, Modus 1
      msgIN.route("/modus_1/view_1", ViewKeyFrame1);
      msgIN.route("/modus_1/dauer_fader_1", DauerKeyFrame1);
      msgIN.route("/modus_1/beschleunigung_fader_1", BeschleunigungKeyFrame1);
      msgIN.route("/modus_1/pause_fader_1", PauseKeyFrame1);

      msgIN.route("/modus_1/keyframe_2", Keyframe2); //KeyFrame 2, Modus 1
      msgIN.route("/modus_1/view_2", ViewKeyFrame2);
      msgIN.route("/modus_1/dauer_fader_2", DauerKeyFrame2);
      msgIN.route("/modus_1/beschleunigung_fader_2", BeschleunigungKeyFrame2);
      msgIN.route("/modus_1/pause_fader_2", PauseKeyFrame2);

      msgIN.route("/modus_1/keyframe_3", Keyframe3); //KeyFrame 3, Modus 1
      msgIN.route("/modus_1/view_3", ViewKeyFrame3);
      msgIN.route("/modus_1/dauer_fader_3", DauerKeyFrame3);
      msgIN.route("/modus_1/beschleunigung_fader_3", BeschleunigungKeyFrame3);
      msgIN.route("/modus_1/pause_fader_3", PauseKeyFrame3);

      msgIN.route("/modus_1/keyframe_4", Keyframe4); //KeyFrame 4, Modus 1
      msgIN.route("/modus_1/view_4", ViewKeyFrame4);
      msgIN.route("/modus_1/pause_fader_4", PauseKeyFrame4);

      //Modus_2
      msgIN.route("/modus_2/fps_fader", FPS);

      msgIN.route("/modus_2/keyframe_1", Keyframe1); //KeyFrame 1, Modus 2
      msgIN.route("/modus_2/View_1", ViewKeyFrame1);
      msgIN.route("/modus_2/dauer_fader_1", DauerKeyFrame1);
      msgIN.route("/modus_2/intervall_fader_1", Intervall1);

      msgIN.route("/modus_2/keyframe_2", Keyframe2); //KeyFrame 2, Modus 2
      msgIN.route("/modus_2/View_2", ViewKeyFrame2);
      msgIN.route("/modus_2/dauer_fader_2", DauerKeyFrame2);
      msgIN.route("/modus_2/intervall_fader_2", Intervall2);

      msgIN.route("/modus_2/keyframe_3", Keyframe3); //KeyFrame 3, Modus 2
      msgIN.route("/modus_2/View_3", ViewKeyFrame3);

      //Modus_3
      msgIN.route("/modus_3/pan_tilt", Rotation);


      //Settings
      msgIN.route("/settings/pan_enable", PanEnable);
      msgIN.route("/settings/slider_enable", SliderEnable);
      msgIN.route("/settings/tilt_enable", TiltEnable);
      msgIN.route("/settings/slider_kalibrieren", Kalibrieren);
      msgIN.route("/settings/resetten", Reset);
      
    }
  }
}

#pragma endregion

void updateUI() {

  SendOSCMessage("/slider_fader",float(float(SliderPosition)/float(MaxSliderPosition)));
  SendOSCMessage("/pan_position",float(Pankopf)/float(MaxPankopf));
  SendOSCMessage("/tilt_position",float(Tiltkopf)/float(MaxPankopf));

}

void loop()
{
  OSCMsgReceive();
  
  //randomMovement();
  



  static uint32_t last_time;
  if (millis() - last_time >500) {    //only run every half second
  
    updateUI();
    last_time = millis();

    debugOutput();
  }

  if (go == 1) {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
    SliderPosition = stepperX.currentPosition();
    Tiltkopf = stepperZ.currentPosition();
    Pankopf = stepperY.currentPosition();
  }
  
  
  

  
}


void randomMovement() {

  if (stepperX.distanceToGo() == 0)
  {
      stepperX.moveTo(random(500,MaxSliderPosition));
      stepperX.setMaxSpeed(50000);
      stepperX.setAcceleration(2500);
      Serial.println("Random");
  }
  if (stepperY.distanceToGo() == 0)
  {
      stepperY.moveTo(random(500,MaxPankopf));
      stepperY.setMaxSpeed(50000);
      stepperY.setAcceleration(2500);
      Serial.println("Random");
  }
  if (stepperZ.distanceToGo() == 0)
  {
      stepperZ.moveTo(random(500,MaxTiltkopf));
      stepperZ.setMaxSpeed(50000);
      stepperZ.setAcceleration(2500);
      Serial.println("Random");
  }
}