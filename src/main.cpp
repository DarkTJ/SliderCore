#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>

#include <OSCBundle.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>

#include <Variablen.h>

const char *ssid = "Slider";         // Name des Netzwerks
const char *password = "slideslide"; // Passwort des Netzwerks

WiFiServer server(80); // Server Instanz -> ESP als Access Point
WiFiUDP Udp;           // UDP Instanz um Pakete über UDP zu senden / empfangen

const IPAddress outIp(192, 168, 4, 2); // IP des Clients
const unsigned int destPort = 9999;    // Incoming-Port des Clients
const unsigned int localPort = 8888;   // Incoming-Ports des Hosts für ankommende Nachrichten

OSCErrorCode error;


void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);

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
}

void SendOSCMessage(OSCMessage msgOUT, float wert){
    msgOUT.add(Pankopf);
    Udp.beginPacket(Udp.remoteIP(), destPort);    //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
    msgOUT.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT.empty(); // free space occupied by message
}
void Modus(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir

  modus = msg.getFloat(0);

  if (modus == 1)
  {
    modus = 0;
  }
  if (modus == 3)
  {
    modus = 1;
  }
  if (modus == 5)
  {
    modus = 2;
  }
  if (modus == 7)
  {
    modus = 3;
  }
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}

void start(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  go = msg.getFloat(0);
  Serial.print("Start? "); //Hier einfügen, was gemacht werden soll
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

  SliderPositon = msg.getFloat(0);
  Serial.print("Sliderposition = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(SliderPosition);
}

void Pankopf_Bewegung(OSCMessage &msg, int addrOffset)
{ //Pankopf1

  Pankopf = msg.getFloat(0);
  if (modus == 1)
  {
    SendOSCMessage("/modus_1/pan_position",Pankopf);
  }
  if (modus == 0)
  {
    SendOSCMessage("/modus_0/pan_position",Pankopf);
  }

  Serial.print("Pankopf = : ");
  Serial.println(Pankopf);
}

void Tiltkopf_Bewegung(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1

  Tiltkopf = msg.getFloat(0);
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
      SliderPosition = KeyFramePosition_0_1;
      Pankopf = KeyFramePan_0_1;
      Tiltkopf = KeyFrameTilt_0_1;
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
    SendOSCMessage("/modus_0/dauer_anzeige_k1",KeyFrameDauer_0_1);
  }
  if (modus == 1)
  {
    KeyFrameDauer_1_1 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 1 1?= : ");
    Serial.println(KeyFrameDauer_1_1);
    SendOSCMessage("/modus_1/dauer_anzeige_1",KeyFrameDauer_1_1);  
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_1 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 2 1?= : ");
    Serial.println(KeyFrameDauer_2_1);
    SendOSCMessage("/modus_2/dauer_anzeige_1",KeyFrameDauer_2_1);  
  }
}

void PauseKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Pause des 1. KeyFrames
  if (modus == 0)
  {
    KeyFramePause_0_1 = msg.getFloat(0);
    Serial.print("KeyFramePause 0 1?= : ");
    Serial.println(KeyFramePause_0_1);
    SendOSCMessage("/modus_0/pause_anzeige_k2",KeyFramePause_0_1);  
  }
  if (modus == 1)
  {
    KeyFramePause_1_1 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 1?= : ");
    Serial.println(KeyFramePause_1_1);
    SendOSCMessage("/modus_1/pause_anzeige_2",KeyFramePause_1_1);  
  }
  if (modus == 2)
  {
    KeyFramePause_2_1 = msg.getFloat(0);
    Serial.print("KeyFramePause 2 1?= : ");
    Serial.println(KeyFramePause_2_1);
    SendOSCMessage("/modus_2/pause_anzeige_2",KeyFramePause_2_1);  
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
    SendOSCMessage("/modus_0/dauer_anzeige_k2",KeyFrameDauer_0_2);  
    
  }
  if (modus == 1)
  {
    KeyFrameDauer_1_2 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 1 2?= : ");
    Serial.println(KeyFrameDauer_1_2);
    SendOSCMessage("/modus_1/dauer_anzeige_2",KeyFrameDauer_1_2);  
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_2 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 2 2?= : ");
    Serial.println(KeyFrameDauer_2_2);
    SendOSCMessage("/modus_2/dauer_anzeige_2",KeyFrameDauer_2_2);  
  }
}

void PauseKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Pause des 2. KeyFrames
  if (modus == 0)
  {
    KeyFramePause_0_2 = msg.getFloat(0);
    Serial.print("KeyFramePause 0 2?= : ");
    Serial.println(KeyFramePause_0_2);
    SendOSCMessage("/modus_0/pause_anzeige_k2",KeyFramePause_0_2);  
  }
  if (modus == 1)
  {
    KeyFramePause_1_2 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 2?= : ");
    Serial.println(KeyFramePause_1_2);
    SendOSCMessage("/modus_1/pause_anzeige_2",KeyFramePause_1_2);  
  }
  if (modus == 2)
  {
    KeyFramePause_2_2 = msg.getFloat(0);
    Serial.print("KeyFramePause 2 2?= : ");
    Serial.println(KeyFramePause_2_2);
    SendOSCMessage("/modus_2/pause_anzeige_2",KeyFramePause_2_2);  
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
    SendOSCMessage("/modus_1/dauer_anzeige_3",KeyFrameDauer_1_3);  
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_3 = msg.getFloat(0);
    Serial.print("KeyFrameDauer 2 3?= : ");
    Serial.println(KeyFrameDauer_2_3);
    SendOSCMessage("/modus_2/dauer_anzeige_3",KeyFrameDauer_2_3);  
  }
  }


void PauseKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Pause des 3. KeyFrames
  if (modus == 1)
  {
    KeyFramePause_1_3 = msg.getFloat(0);
    Serial.print("KeyFramePause 1 3?= : ");
    Serial.println(KeyFramePause_1_3);
    SendOSCMessage("/modus_1/pause_anzeige_3",KeyFramePause_1_3);  
  }
  
  if (modus == 2)
  {
    KeyFramePause_2_3 = msg.getFloat(0);
    Serial.print("KeyFramePause 2 3?= : ");
    Serial.println(KeyFramePause_2_3);
    SendOSCMessage("/modus_2/pause_anzeige_3",KeyFramePause_2_3);  
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
    SendOSCMessage("/modus_1/pause_anzeige_4",KeyFramePause_1_4);  
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
  SendOSCMessage("/modus_2/sequenzdauer_anzeige_1",sequenzdauer1);
}

void Sequenzdauer2(OSCMessage &msg, int addrOffset)  //Schickt Variable Sequenzdauer1  
{ 
  //!!!Hier Rechnung einfügen
  Serial.print("sequenzdauer2 : ");
  Serial.println(sequenzdauer2);
  SendOSCMessage("/modus_2/sequenzdauer_anzeige_2",sequenzdauer2 );
}

void Intervall1(OSCMessage &msg, int addrOffset){
    intervall1 = msg.getFloat(0);
    Serial.print("Intervall1 1?= : ");
    Serial.println(intervall1);
    SendOSCMessage("/modus_2/intervall_anzeige_1",intervall1); 
}

void Intervall2(OSCMessage &msg, int addrOffset){
    intervall2 = msg.getFloat(0);
    Serial.print("Intervall2 1?= : ");
    Serial.println(intervall2);
    SendOSCMessage("/modus_2/intervall_anzeige_2",intervall2); 
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
    Serial.print("Slider Enable?= : ");
    Serial.println(slider_enable);
}

void PanEnable(OSCMessage &msg, int addrOffset){
    pan_enable = msg.getFloat(0);
    Serial.print("Pan Enable?= : ");
    Serial.println(pan_enable);
}

void TiltEnable(OSCMessage &msg, int addrOffset){
    tilt_enable = msg.getFloat(0);
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
      /*        msgIN.route("/modus_0", Modus); // die 4 Modi mit alle der gleichen Funktion
              msgIN.route("/modus_1", Modus);
              msgIN.route("/modus_2", Modus);
              msgIN.route("/modus_3", Modus);
      */

      msgIN.route("/modus_0/start", start); //Startknöpfe
      msgIN.route("/modus_1/start", start);
      msgIN.route("/modus_2/start", start);

    msgIN.route("/modus_0/slider_fader", SliderBewegung); //modus 0 ruft gleiche Funktionen auf wie modus 1
    msgIN.route("/modus_0/pan_encoder", Pankopf_Bewegung);
    msgIN.route("/modus_0/tilt_position", Tiltkopf_Bewegung);

      msgIN.route("/modus_1/slider_fader", SliderBewegung); //Positionseinstellungen
      msgIN.route("/modus_1/pan_encoder", Pankopf_Bewegung);
      msgIN.route("/modus_1/tilt_position", Tiltkopf_Bewegung);

    msgIN.route("/modus_2/slider_fader", SliderBewegung); //Positionseinstellungen
    msgIN.route("/modus_2/pan_encoder", Pankopf_Bewegung);
    msgIN.route("/modus_2/tilt_position", Tiltkopf_Bewegung);

      

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
      msgIN.route("/modus_3/slider_fader", SliderBewegung);
      msgIN.route("/modus_3/start", start);
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




void loop()
{
  OSCMsgReceive();
}