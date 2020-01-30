#include <Arduino.h>

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>

#include <OSCBundle.h>
#include <OSCMessage.h>
#include <OSCData.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>

#include <Variablen.h>

const char *ssid = "SliderMasterVersion2"; // Name des Netzwerks
const char *password = "slideslide";       // Passwort des Netzwerks

WiFiServer server(80); // Server Instanz -> ESP als Access Point
WiFiUDP Udp;           // UDP Instanz um Pakete über UDP zu senden / empfangen

const IPAddress outIp(192, 168, 4, 2); // IP des Clients
const unsigned int destPort = 9999;    // Incoming-Port des Clients
const unsigned int localPort = 8888;   // Incoming-Ports des Hosts für ankommende Nachrichten

OSCErrorCode error;

////////////////////////////////////////////////////////////////////////////7
// Stepper setup

#define EN_PINX 33 // Enable Slider Driver
#define EN_PINY 22 // Enable Pan Driver
#define EN_PINZ 21 // Enable  Tilt Driver
#define CS_PINX 15 // Chip select X Achse (Slider)
#define CS_PINY 2  // Chip select Y Achse (Pan)
#define CS_PINZ 4  // Chip select Z Achse (Tilt)

#define SW_MOSI 13 // Software Master Out Slave In (MOSI)
#define SW_MISO 12 // Software Master In Slave Out (MISO)
#define SW_SCK 23  // Software Slave Clock (SCK)

#define STEPX 26
#define DIRX 25

#define STEPY 5
#define DIRY 18

#define STEPZ 27
#define DIRZ 14

using namespace TMC2130_n;

#define STALL_VALUE 20
#define R_SENSE 0.11f

#define StandartAcc 6000
#define StandartSpeed 10000
// Stepper und Steppertimer initialisierung:

//TMC2130Stepper driverX(CS_PINX, R_SENSE);                           // Hardware SPI
TMC2130Stepper driverX(CS_PINX, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driverY(CS_PINY, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverZ(CS_PINZ, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

AccelStepper stepperX(AccelStepper::DRIVER, STEPX, DIRX); //Slider
AccelStepper stepperY(AccelStepper::DRIVER, STEPY, DIRY); //Pan
AccelStepper stepperZ(AccelStepper::DRIVER, STEPZ, DIRZ); //Tilt

void setup()
{
  Serial.begin(115200);

  pinMode(EN_PINX, OUTPUT);
  pinMode(STEPX, OUTPUT);
  pinMode(DIRX, OUTPUT);
  digitalWrite(EN_PINX, LOW); // Enable driver in hardware
  pinMode(EN_PINY, OUTPUT);
  pinMode(STEPY, OUTPUT);
  pinMode(DIRY, OUTPUT);
  digitalWrite(EN_PINY, LOW); // Enable driver in hardware
  pinMode(EN_PINZ, OUTPUT);
  pinMode(STEPZ, OUTPUT);
  pinMode(DIRZ, OUTPUT);
  digitalWrite(EN_PINZ, LOW); // Enable driver in hardware

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

  // driverStartup X = Slider
  //               Y = Pan
  //               Z = Tilt
  driverX.begin();
  driverX.toff(5);             // Enables driver in software
  driverX.rms_current(1000);   // Set motor RMS current
  driverX.microsteps(16);      // Set microsteps to 1/16th
  driverX.en_pwm_mode(true);   // Toggle stealthChop on TMC2130/2160/5130/5160
  driverX.pwm_autoscale(true); // Needed for stealthChop

  //stallguard config
  /*
  driverX.TCOOLTHRS(0xFFFFF); // 20bit max
  driverX.THIGH(0);
  driverX.semin(5);
  driverX.semax(2);
  driverX.sedn(0b01);
  driverX.sgt(STALL_VALUE);*/

  driverY.begin();
  driverY.toff(5);             // Enables driver in software
  driverY.rms_current(300);    // Set motor RMS current
  driverY.microsteps(16);      // Set microsteps to 1/16th
  driverY.en_pwm_mode(true);   // Toggle stealthChop on TMC2130/2160/5130/5160
  driverY.pwm_autoscale(true); // Needed for stealthChop

  driverZ.begin();
  driverZ.toff(5);             // Enables driver in software
  driverZ.rms_current(300);    // Set motor RMS current
  driverZ.microsteps(16);      // Set microsteps to 1/16th
  driverZ.en_pwm_mode(true);   // Toggle stealthChop on TMC2130/2160/5130/5160
  driverZ.pwm_autoscale(true); // Needed for stealthChop

  delay(2500); //delay, um den Treibern zeit zum starten zu geben

  //setzen von Standartwerten, um Modus 0 sofort nutzbar zu machen, theoretisch auch ohne App.
  MaxSliderPosition = 120000;
  MaxPankopf = 15000;
  MaxTiltkopf = 20000;
  KeyFrame_0_1 = true;
  KeyFrame_0_2 = true;
  KeyFramePosition_0_2 = MaxSliderPosition-10000;
  KeyFramePosition_0_1 = 10000;
  KeyFrameBeschleunigung_0_1 = StandartAcc;
  KeyFrameBeschleunigung_0_2 = StandartAcc;
  KeyFrameDauer_0_1 = 2000;
  KeyFrameDauer_0_2 = 2000;
  KeyFramePause_0_1 = 8;
  KeyFramePause_0_2 = 8;
  KeyFrameTilt_0_1 = MaxTiltkopf / 4;
  KeyFrameTilt_0_2 = MaxTiltkopf / 4;
  KeyFramePan_0_1 = (MaxPankopf / 2) - 1000;
  KeyFramePan_0_2 = (MaxPankopf / 2) + 1000;

  stepperX.setMaxSpeed(30000);
  stepperX.setAcceleration(2500);
  stepperY.setMaxSpeed(30000);
  stepperY.setAcceleration(2500);
  stepperZ.setMaxSpeed(30000);
  stepperZ.setAcceleration(2500);

  stepperX.setCurrentPosition(MaxSliderPosition / 2);
  stepperY.setCurrentPosition(MaxPankopf / 2);
  stepperZ.setCurrentPosition(MaxTiltkopf / 4);
}

//constructors weil ich zu faul bin den Code anders zu sortieren
void SendOSCMessage(OSCMessage msgOUT, int wert);
void SendOSCMessage(OSCMessage msgOUT, float wert);
void SendOSCMessage(OSCMessage msgOUT, char *wert);
void updateSequenzDauer();

#pragma region Stepper

float zeitdesMoves(float MaxSpeed, float Acc, float distance)
{ //Berechnet mithilfe des Weg-Zeit-Gesetzes
  if (distance == 0)
  {
    return MaxPankopf / 4; //Notfall, wenn XAchse sich nicht bewegt (Panik) TO-BE DONE RIGHT
  }
  else
  {
    float gesZeit;
    float timeAcc = MaxSpeed / Acc; //zeit der beschleunigung
    Serial.println(timeAcc);
    float distAcc = 0.5f * Acc * (timeAcc * timeAcc); //s = 0,5 · a · t²      distanz der beschleinigung
    Serial.println(distAcc);
    float timeMaxSpeed = (distance - 2 * distAcc) / MaxSpeed; //zeit zwischen den beiden beschleunigungs/bremsteilen
    if (timeMaxSpeed < 0)                                     //wenn zeit negativ == maximale geschwindigkeit wird nie erreicht!!
    {
      timeMaxSpeed = 0;
      distAcc = distance / 2;                  // da die hälfte der Zeit beschl. wird.
      timeAcc = sqrtf(distAcc / (0.5f * Acc)); //obrige fromel nach t umgeformt
      Serial.print("TimeAccNeu: ");
      Serial.println(timeAcc);
    }
    Serial.println(timeMaxSpeed);
    gesZeit = 2 * timeAcc + timeMaxSpeed;
    Serial.println(gesZeit);
    return gesZeit;
  }
}
void calcSpeed(float MaxSpeed, float Acc, float distancetoMatch, float distancetoDo, float timeToMatch)
{ //Berechnet Geschwindigkeit, um die eingegebene Strecke in der angegebenen Zeit zu erreichen
  accGlobal = Acc;
  if (distancetoMatch == 0)
  {                          //wenn XAchse sich nicht bewegt
    distancetoMatch = 12000; //dont know
    Serial.println("Xachse Steht...");
  }
  else if (distancetoDo == 0)
  { //wenn die eigene Achse sich nicht bewegt, irgendwelche werte um ekin error zu erzeugen.
    speedGlobal = StandartSpeed;
    accGlobal = StandartAcc;
    Serial.println("Eine Achse Steht");
  }
  else
  {
    float timeAcc = MaxSpeed / Acc;
    if (timeToMatch / 2 < timeAcc)
    { // wenn die maxmimale speed nie erriecht wird. neuberechnen aber MEHR Acceleration;
      Serial.println("neueAcc");
      calcSpeed(MaxSpeed, 5 * (Acc / 4), distancetoMatch, distancetoDo, timeToMatch);
    }
    else
    {
      float timefullSpeed = timeToMatch - 2 * timeAcc;
      float distAcc = 0.5f * Acc * (timeAcc * timeAcc);
      float speed = (distancetoDo - 2 * distAcc) / timefullSpeed;
      if (abs(speed - speedGlobal) < 250)
      {
        speedGlobal = speed;
        Serial.print("Speed: ");
        Serial.println(speedGlobal);
      }
      else
      {
        Serial.print("We Try Again: ");
        Serial.println(speed);
        speedGlobal = speed;
        calcSpeed(speed, Acc, distancetoMatch, distancetoDo, timeToMatch);
      }
    }
  }
}

// wichtigste Funktion, es wird der goModus, und der angestrebte Keyframe als Input erwartet, und damit den Motoren (nach berechnung der Geschwindigkeit+Beschleunigung) der .moveTo() befehl gegeben.

void gotoKeyframe(int Keyframe)
{

  if (goModus == 0)
  { // if abfrage zur übersichtlichkeit, case switch wird schnell unübersichtlich
    if (Keyframe == 0)
    {
      if (KeyFrame_0_1 == 1)
      {
        stepperX.setAcceleration(StandartAcc);
        stepperX.setMaxSpeed(KeyFrameDauer_0_1);

        calcSpeed(KeyFrameDauer_0_1, StandartAcc, abs(KeyFramePosition_0_1 - stepperX.currentPosition()), abs(KeyFramePan_0_1 - stepperY.currentPosition()), zeitdesMoves(KeyFrameDauer_0_1, StandartAcc, abs(KeyFramePosition_0_1 - stepperX.currentPosition())));
        stepperY.setMaxSpeed(speedGlobal);
        stepperY.setAcceleration(accGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        calcSpeed(KeyFrameDauer_0_1, StandartAcc, abs(KeyFramePosition_0_1 - stepperX.currentPosition()), abs(KeyFrameTilt_0_1 - stepperZ.currentPosition()), zeitdesMoves(KeyFrameDauer_0_1, StandartAcc, abs(KeyFramePosition_0_1 - stepperX.currentPosition())));
        stepperZ.setAcceleration(accGlobal);
        stepperZ.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        stepperX.moveTo(KeyFramePosition_0_1);
        stepperY.moveTo(KeyFramePan_0_1);
        stepperZ.moveTo(KeyFrameTilt_0_1);

        pause_timeKeyframe = 1000 * KeyFramePause_0_1;
      }
      else
      { //wenn kein KeyFrame gesetzt ist.
        go = 0;
        SendOSCMessage("/start_label/color", "orange");
        SendOSCMessage("/start_label", "No Keyframe");
        SendOSCMessage("/start/color", "green");
        SendOSCMessage("/start", 0);
      }
    }
    else if (Keyframe == 1)
    {
      if (KeyFrame_0_2 == 1)
      {
        stepperX.setAcceleration(StandartAcc);
        stepperX.setMaxSpeed(KeyFrameDauer_0_2);

        calcSpeed(KeyFrameDauer_0_2, StandartAcc, abs(KeyFramePosition_0_2 - stepperX.currentPosition()), abs(KeyFramePan_0_2 - stepperY.currentPosition()), zeitdesMoves(KeyFrameDauer_0_2, StandartAcc, abs(KeyFramePosition_0_2 - stepperX.currentPosition())));

        stepperY.setAcceleration(accGlobal);
        stepperY.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        calcSpeed(KeyFrameDauer_0_2, StandartAcc, abs(KeyFramePosition_0_2 - stepperX.currentPosition()), abs(KeyFrameTilt_0_2 - stepperZ.currentPosition()), zeitdesMoves(KeyFrameDauer_0_2, StandartAcc, abs(KeyFramePosition_0_2 - stepperX.currentPosition())));
        stepperZ.setAcceleration(accGlobal);
        stepperZ.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        stepperX.moveTo(KeyFramePosition_0_2);
        stepperY.moveTo(KeyFramePan_0_2);
        stepperZ.moveTo(KeyFrameTilt_0_2);

        pause_timeKeyframe = 1000 * KeyFramePause_0_2;
      }
      else
      { //wenn kein KeyFrame gesetzt ist.
        go = 0;
        SendOSCMessage("/start_label/color", "orange");
        SendOSCMessage("/start_label", "No Keyframe");
        SendOSCMessage("/start/color", "green");
        SendOSCMessage("/start", 0);
      }
    }
  }
  else if (goModus == 1)
  {
    if (Keyframe == 0)
    {
      if (KeyFrame_1_1 == 1)
      {
        //langsamsts Movement berechnen
        stepperX.setAcceleration(KeyFrameBeschleunigung_1_1);
        stepperX.setMaxSpeed(KeyFrameDauer_1_1);

        calcSpeed(KeyFrameDauer_1_1, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_1 - stepperX.currentPosition()), abs(KeyFramePan_1_1 - stepperY.currentPosition()), zeitdesMoves(KeyFrameDauer_1_1, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_1 - stepperX.currentPosition())));
        stepperY.setAcceleration(accGlobal);
        stepperY.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        calcSpeed(KeyFrameDauer_1_1, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_1 - stepperX.currentPosition()), abs(KeyFrameTilt_1_1 - stepperZ.currentPosition()), zeitdesMoves(KeyFrameDauer_1_1, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_1 - stepperX.currentPosition())));
        stepperZ.setAcceleration(accGlobal);
        stepperZ.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        stepperX.moveTo(KeyFramePosition_1_1);
        stepperY.moveTo(KeyFramePan_1_1);
        stepperZ.moveTo(KeyFrameTilt_1_1);

        pause_timeKeyframe = 1000 * KeyFramePause_1_1;
      }
      else
      {
        gotoKeyframe(Keyframe + 1);
      }
    }
    if (Keyframe == 1)
    {
      if (KeyFrame_1_2 == 1)
      {
        //langsamsts Movement berechnen
        stepperX.setAcceleration(KeyFrameBeschleunigung_1_2);
        stepperX.setMaxSpeed(KeyFrameDauer_1_2);

        calcSpeed(KeyFrameDauer_1_2, KeyFrameBeschleunigung_1_2, abs(KeyFramePosition_1_2 - stepperX.currentPosition()), abs(KeyFramePan_1_2 - stepperY.currentPosition()), zeitdesMoves(KeyFrameDauer_1_2, KeyFrameBeschleunigung_1_2, abs(KeyFramePosition_1_2 - stepperX.currentPosition())));
        stepperY.setAcceleration(accGlobal);
        stepperY.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        calcSpeed(KeyFrameDauer_1_2, KeyFrameBeschleunigung_1_2, abs(KeyFramePosition_1_2 - stepperX.currentPosition()), abs(KeyFrameTilt_1_2 - stepperZ.currentPosition()), zeitdesMoves(KeyFrameDauer_1_2, KeyFrameBeschleunigung_1_2, abs(KeyFramePosition_1_2 - stepperX.currentPosition())));
        stepperZ.setAcceleration(accGlobal);
        stepperZ.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        stepperX.moveTo(KeyFramePosition_1_2);
        stepperY.moveTo(KeyFramePan_1_2);
        stepperZ.moveTo(KeyFrameTilt_1_2);

        pause_timeKeyframe = 1000 * KeyFramePause_1_2;
      }
      else
      {
        gotoKeyframe(Keyframe + 1);
      }
    }
    if (Keyframe == 2)
    {
      if (KeyFrame_1_3 == 1)
      {
        //langsamsts Movement berechnen
        stepperX.setAcceleration(KeyFrameBeschleunigung_1_3);
        stepperX.setMaxSpeed(KeyFrameDauer_1_3);

        calcSpeed(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_3 - stepperX.currentPosition()), abs(KeyFramePan_1_3 - stepperY.currentPosition()), zeitdesMoves(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_3 - stepperX.currentPosition())));
        stepperY.setAcceleration(accGlobal);
        stepperY.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        calcSpeed(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_3 - stepperX.currentPosition()), abs(KeyFrameTilt_1_3 - stepperZ.currentPosition()), zeitdesMoves(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_3 - stepperX.currentPosition())));
        stepperZ.setAcceleration(accGlobal);
        stepperZ.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        stepperX.moveTo(KeyFramePosition_1_3);
        stepperY.moveTo(KeyFramePan_1_3);
        stepperZ.moveTo(KeyFrameTilt_1_3);

        pause_timeKeyframe = 1000 * KeyFramePause_1_3;
      }
      else
      {
        gotoKeyframe(Keyframe + 1);
      }
    }
    if (Keyframe == 3)
    {
      if (KeyFrame_1_4 == 1)
      {
        //langsamsts Movement berechnen
        stepperX.setAcceleration(KeyFrameBeschleunigung_1_3);
        stepperX.setMaxSpeed(KeyFrameDauer_1_3);

        calcSpeed(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_4 - stepperX.currentPosition()), abs(KeyFramePan_1_4 - stepperY.currentPosition()), zeitdesMoves(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_4 - stepperX.currentPosition())));
        stepperY.setAcceleration(accGlobal);
        stepperY.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        calcSpeed(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_4 - stepperX.currentPosition()), abs(KeyFrameTilt_1_4 - stepperZ.currentPosition()), zeitdesMoves(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_4 - stepperX.currentPosition())));
        stepperZ.setAcceleration(accGlobal);
        stepperZ.setMaxSpeed(speedGlobal);

        accGlobal = 2000000000;
        speedGlobal = 2000000000;

        stepperX.moveTo(KeyFramePosition_1_4);
        stepperY.moveTo(KeyFramePan_1_4);
        stepperZ.moveTo(KeyFrameTilt_1_4);

        pause_timeKeyframe = 1000 * KeyFramePause_1_3;
      }
      else
      {
        gotoKeyframe(0); // loop zurück
      }
    }
  }
  else if (goModus == 2) //alles anders hier LOL
  {
    if (Keyframe == 0)
    {
      if (KeyFrame_2_1 == 1)
      {

        reversemsg = 0;
        stepperX.setAcceleration(10 * StandartAcc);
        stepperX.setMaxSpeed(StandartSpeed);
        stepperY.setAcceleration(10 * StandartAcc);
        stepperY.setMaxSpeed(StandartSpeed);
        stepperZ.setAcceleration(10 * StandartAcc);
        stepperZ.setMaxSpeed(StandartSpeed);

        stepsbetweenPicturesX = (KeyFramePosition_2_1 - stepperX.currentPosition()) / bilder1;
        stepsbetweenPicturesY = (KeyFramePan_2_1 - stepperY.currentPosition()) / bilder1;
        stepsbetweenPicturesZ = (KeyFrameTilt_2_1 - stepperZ.currentPosition()) / bilder1;

        if (abs(KeyFramePosition_2_1 - stepperX.currentPosition()) < 3000)
        { //bugfix=??
          stepsbetweenPicturesX = 1000;
        }

        modus2target = KeyFramePosition_2_1;
        modus2Interval = 1000 * intervall1;
        modus2Waiting = millis() + 1000 * intervall1;

        pause_timeKeyframe = 0;
      }
      else
      {
        gotoKeyframe(Keyframe + 1);
      }
    }
    if (Keyframe == 1)
    {
      if (KeyFrame_2_2 == 1)
      {

        reversemsg = 0;
        stepperX.setAcceleration(10 * StandartAcc);
        stepperX.setMaxSpeed(StandartSpeed);
        stepperY.setAcceleration(10 * StandartAcc);
        stepperY.setMaxSpeed(StandartSpeed);
        stepperZ.setAcceleration(10 * StandartAcc);
        stepperZ.setMaxSpeed(StandartSpeed);

        stepsbetweenPicturesX = (KeyFramePosition_2_2 - stepperX.currentPosition()) / bilder1;
        stepsbetweenPicturesY = (KeyFramePan_2_2 - stepperY.currentPosition()) / bilder1;
        stepsbetweenPicturesZ = (KeyFrameTilt_2_2 - stepperZ.currentPosition()) / bilder1;

        if (abs(KeyFramePosition_2_2 - stepperX.currentPosition()) < 3000)
        { //bugfix=??
          stepsbetweenPicturesX = 1000;
        }

        modus2target = KeyFramePosition_2_2;
        modus2Interval = 1000 * intervall1;
        modus2Waiting = millis() + 1000 * intervall1;

        pause_timeKeyframe = 0;
      }
      else
      {
        gotoKeyframe(Keyframe + 1);
      }
    }
    if (Keyframe == 2)
    {
      if (KeyFrame_2_3 == 1)
      {

        reversemsg = 0;
        stepperX.setAcceleration(10 * StandartAcc);
        stepperX.setMaxSpeed(StandartSpeed);
        stepperY.setAcceleration(10 * StandartAcc);
        stepperY.setMaxSpeed(StandartSpeed);
        stepperZ.setAcceleration(10 * StandartAcc);
        stepperZ.setMaxSpeed(StandartSpeed);

        stepsbetweenPicturesX = (KeyFramePosition_2_3 - stepperX.currentPosition()) / bilder2;
        stepsbetweenPicturesY = (KeyFramePan_2_3 - stepperY.currentPosition()) / bilder2;
        stepsbetweenPicturesZ = (KeyFrameTilt_2_3 - stepperZ.currentPosition()) / bilder2;

        if (abs(KeyFramePosition_2_3 - stepperX.currentPosition()) < 3000)
        { //bugfix=??
          stepsbetweenPicturesX = 1000;
        }

        modus2target = KeyFramePosition_2_3;
        modus2Interval = 1000 * intervall2;
        modus2Waiting = millis() + 1000 * intervall2;

        pause_timeKeyframe = 0;
      }
      else
      {
        gotoKeyframe(0);
      }
    }
  }
}

void sliderAusmessen()
{
  goModus = -3; // auf Nullposition fahren
  go = 1;
  driverX.en_pwm_mode(false); // disable stealthcop für stallguard
  driverX.reset();
  stepperX.setAcceleration(5000);
  stepperX.setMaxSpeed(50000);
  stepperX.move(-100000000);
}

void stallDetect(uint32_t ms)
{

  static uint32_t last_time = 0;

  if ((ms - last_time) > 50)
  { //run every 0.05s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driverX.DRV_STATUS(); //status des Treibers wird ausgelesen

    //Messung

    if (drv_status.sg_result == 0 && goModus == -3) //wenn der Stepper irgendwo gegenfährt (sg.result== 0) wobei sg = stallguard
    {                                               //zurückfahen (modus -3)

      stepperX.setCurrentPosition(-400); //einige Steps von 0 weg um anstoßen zu verhindern   NULLposition

      goModus = -2;
      driverX.reset();
      Serial.println(" F ");
      drv_status.sg_result = 1;
      stepperX.moveTo(100000);
    }
    else if (drv_status.sg_result == 0 && goModus == -2) //wenn der Stepper irgendwo gegenfährt (sg.result== 0) wobei sg = stallguard
    {                                                    // an maximum fahren  (modus -2)

      stepperX.stop();
      MaxSliderPosition = stepperX.currentPosition() - 400; // maximalposition, wieder ein stück weg um anstoßen zu verhindern
      driverX.en_pwm_mode(true);                            // Toggle stealthChop on TMC2130/2160/5130/5160
      driverX.reset();
      Serial.println(MaxSliderPosition);
    }
  }
}

void randomMovement()
{

  if (stepperX.distanceToGo() == 0)
  {
    stepperX.moveTo(random(500, MaxSliderPosition));
    stepperX.setMaxSpeed(50000);
    stepperX.setAcceleration(2500);
    Serial.println("Random");
  }
  if (stepperY.distanceToGo() == 0)
  {
    stepperY.moveTo(random(500, MaxPankopf));
    stepperY.setMaxSpeed(50000);
    stepperY.setAcceleration(2500);
    Serial.println("Random");
  }
  if (stepperZ.distanceToGo() == 0)
  {
    stepperZ.moveTo(random(500, MaxTiltkopf));
    stepperZ.setMaxSpeed(50000);
    stepperZ.setAcceleration(2500);
    Serial.println("Random");
  }
}

void debugOutput() //einfache funktion die verschiedene Diagnosewerte des Tribers ausgibt
{

  static uint32_t last_time = 0;

  if ((millis() - last_time) > 100)
  { //run every 0.1s
    last_time = millis();

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driverX.DRV_STATUS();

    if (true)
    {
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
#pragma region OSCWert - Sende - Funktionen

/////////////////
//Send OSCWert Funktionen

void SendOSCMessage(OSCMessage msgOUT, float wert)
{
  msgOUT.add(wert);
  Udp.beginPacket(Udp.remoteIP(), destPort); //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp);                          // send the bytes
  Udp.endPacket();                           // mark the end of the OSC Packet
  msgOUT.empty();                            // free space occupied by message
}
void SendOSCMessage(OSCMessage msgOUT, char *wert)
{
  msgOUT.add(wert);
  Udp.beginPacket(Udp.remoteIP(), destPort); //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp);                          // send the bytes
  Udp.endPacket();                           // mark the end of the OSC Packet
  msgOUT.empty();                            // free space occupied by message
}
void SendOSCMessage(OSCMessage msgOUT, int wert)
{
  msgOUT.add(wert);
  Udp.beginPacket(Udp.remoteIP(), destPort); //Sendet den Wert der am Encoder eingestellt wurde an den Anzeiger für die Position
  msgOUT.send(Udp);                          // send the bytes
  Udp.endPacket();                           // mark the end of the OSC Packet
  msgOUT.empty();                            // free space occupied by message
}

#pragma endregion
#pragma region ModusauswahlundStart

void changeModus(int modus)
{

  if (go == 1 && goModus != modus)
  { // Slider läuft aber nicht im ausgewählten Modus
    SendOSCMessage("/start_label/color", "green");
    SendOSCMessage("/start_label", "Start");
    SendOSCMessage("/start/color", "orange");
    SendOSCMessage("/start", 0);
  }
  else if (go == 1 && goModus == modus)
  { //Slider läuft gerad im ausgewwählten Modus
    SendOSCMessage("/start_label/color", "red");
    SendOSCMessage("/start_label", "Stop");
    SendOSCMessage("/start/color", "red");
    SendOSCMessage("/start", 1);
  }
  else if (go == 0)
  { //Slider läuft gerad im ausgewwählten Modus
    SendOSCMessage("/start_label/color", "green");
    SendOSCMessage("/start_label", "Start");
    SendOSCMessage("/start/color", "green");
    SendOSCMessage("/start", 0);
  }
}

void Modus0(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 0;
  changeModus(0);

  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}
void Modus1(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 1;
  changeModus(1);
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}
void Modus2(OSCMessage &msg, int addrOffset)
{ // In welchem Modus sind wir
  modus = 2;
  changeModus(2);
  Serial.print("Modus:"); //Hier einfügen, was gemacht werden soll
  Serial.println(modus);
}
void Modus3(OSCMessage &msg, int addrOffset)
{
  modus = 3;
  stepperX.setMaxSpeed(50000); //setting spped and acceleration for mode 3
  stepperX.setAcceleration(2500);
  stepperY.setMaxSpeed(50000);
  stepperY.setAcceleration(2500);
  stepperZ.setMaxSpeed(50000);
  stepperZ.setAcceleration(2500);
  changeModus(3);
  Serial.print("Modus:");
  Serial.println(modus);
}

void start(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  bool go_ = msg.getFloat(0);

  if (go_ == 1 && go == 0)
  {
    goModus = modus;
    go = 1;
    SendOSCMessage("/start_label/color", "red");
    SendOSCMessage("/start_label", "Stop");
    SendOSCMessage("/start/color", "red");
    SendOSCMessage("/start", 1);
    gotoKeyframe(NextKeyFrame[goModus]);
  }
  else if (go_ == 1 && go == 1)
  {
    goModus = modus;
    SendOSCMessage("/start_label/color", "red");
    SendOSCMessage("/start_label", "Stop");
    SendOSCMessage("/start/color", "red");
    SendOSCMessage("/start", 1);
    gotoKeyframe(NextKeyFrame[goModus]);
  }
  else
  {
    go = 0;
    SendOSCMessage("/start_label/color", "green");
    SendOSCMessage("/start_label", "Start");
    SendOSCMessage("/start/color", "green");
    SendOSCMessage("/start", 0);

    if (goModus == 3 || goModus == -1)
    {
      stepperX.stop();
      stepperY.stop();
      stepperZ.stop();
    }
  }

  Serial.print("Start? ");
  Serial.println(go);
}
#pragma endregion
#pragma region Manuelle Bewegung

void SliderBewegung(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  if (go == 1)
  {
    stepperX.moveTo(MaxSliderPosition * msg.getFloat(0));
  }

  Serial.print("Sliderposition = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(stepperX.distanceToGo());
}

void Pankopf_Bewegung(OSCMessage &msg, int addrOffset)
{ //Pankopf1
  if (go == 1)
  {
    if (msg.getFloat(0) == 1)
    {
      stepperY.move(200);
    }
    else
    {
      stepperY.move(-200);
    }
  }

  Serial.print("Pankopf = : ");
  Serial.println(Pankopf);
}

void Tiltkopf_Bewegung(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  if (go == 1)
  {
    stepperZ.moveTo(MaxTiltkopf * msg.getFloat(0));
  }

  Serial.print("Tiltkopf = : "); //Hier einfügen, was gemacht werden soll
  Serial.println(stepperZ.distanceToGo());
}

//Modus 3----------------------------------------------------------------------------------------------------------------------------------------------------
void Rotation(OSCMessage &msg, int addrOffset)
{
  float rotationY = msg.getFloat(1);
  float rotationZ = msg.getFloat(0);

  if (rotationY > 0.45f && rotationY < 0.55f && goModus == 3)
  {
    stepperY.stop();
  }
  else if (rotationY > 0.55f && goModus == 3)
  {
    rotationY = rotationY - 0.5f;

    stepperY.move(-800 * rotationY);
  }
  else if (rotationY < 0.45f && goModus == 3)
  {
    rotationY = 0.5f - rotationY;
    stepperY.move(800 * rotationY);
  }

  if (rotationZ > 0.45f && rotationZ < 0.55f && goModus == 3)
  {
    stepperY.stop();
  }
  else if (rotationZ > 0.55f && goModus == 3)
  {
    rotationZ = rotationZ - 0.5f;

    stepperZ.move(800 * rotationZ);
  }
  else if (rotationZ < 0.45f && goModus == 3)
  {
    rotationZ = 0.5f - rotationZ;
    stepperZ.move(-800 * rotationZ);
  }

  Serial.print("RotationPAN= : ");
  Serial.print(rotationY);
  Serial.print(" RotationTILT= : ");
  Serial.println(rotationZ);
}

void Bewegung(OSCMessage &msg, int addrOffset)
{
  float sliderpos = msg.getFloat(0);
  if (sliderpos > 0.4f && sliderpos < 0.6f && goModus == 3)
  {
    stepperX.stop();
  }
  else if (sliderpos > 0.6f && goModus == 3)
  {
    sliderpos = sliderpos - 0.5f;

    stepperX.move(1300 * sliderpos);
  }
  else if (sliderpos < 0.4f && goModus == 3)
  {
    sliderpos = 0.5f - sliderpos;
    stepperX.move(-1300 * sliderpos);
  }

  Serial.print("Moving to ?= : ");
  Serial.println(stepperX.distanceToGo());
}

#pragma endregion
#pragma region Keyframes
void Reverse(OSCMessage &msg, int addrOffset)
{ // Slider Position Modus 1
  reversemsg = msg.getFloat(0);

  Serial.print("reverse? ");
  Serial.println(reversemsg);
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
  stepperX.setMaxSpeed(30000);
  stepperX.setAcceleration(2500);
  stepperY.setMaxSpeed(30000);
  stepperY.setAcceleration(2500);
  stepperZ.setMaxSpeed(30000);
  stepperZ.setAcceleration(2500);

  if (modus == 0)
  {
    Serial.print("View Keyframe 0 1?= : ");
    Serial.println(view);
    if (view == 1)
    {

      stepperX.moveTo(KeyFramePosition_0_1);
      stepperY.moveTo(KeyFramePan_0_1);
      stepperZ.moveTo(KeyFrameTilt_0_1);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 1?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_1_1);
      stepperY.moveTo(KeyFramePan_1_1);
      stepperZ.moveTo(KeyFrameTilt_1_1);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }

  if (modus == 2)
  {
    Serial.print("View Keyframe 2 1?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_2_1);
      stepperY.moveTo(KeyFramePan_2_1);
      stepperZ.moveTo(KeyFrameTilt_2_1);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }
}

void BeschleunigungKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 1. KeyFrames

  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_1 = StandartAcc * (1.05 - msg.getFloat(0));
    Serial.print("KeyFrameBeschleunigung_1_1?= : ");
    Serial.println(KeyFrameBeschleunigung_1_1);
  }
}

void DauerKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 1. KeyFrames
  if (modus == 0)
  {
    KeyFrameDauer_0_1 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 0 1?= : ");
    Serial.println(KeyFrameDauer_0_1);
    SendOSCMessage("/modus_0/dauer_anzeige_k1", zeitdesMoves(KeyFrameDauer_0_1, StandartAcc * 0.5, abs(KeyFramePosition_0_1 - KeyFramePosition_0_2)));
  }
  if (modus == 1)
  {
    KeyFrameDauer_1_1 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 1 1?= : ");
    Serial.println(KeyFrameDauer_1_1);
    SendOSCMessage("/modus_1/dauer_anzeige_1", zeitdesMoves(KeyFrameDauer_1_1, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_1 - KeyFramePosition_1_2)));
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_1 = 200 * (0.005 + msg.getFloat(0));
    Serial.print("KeyFrameDauer 2 1?= : ");
    Serial.println(KeyFrameDauer_2_1);
    SendOSCMessage("/modus_2/dauer_anzeige_1", KeyFrameDauer_2_1);
    updateSequenzDauer();
  }
}

void PauseKeyFrame1(OSCMessage &msg, int addrOffset)
{ //Pause des 1. KeyFrames
  if (modus == 0)
  {
    KeyFramePause_0_1 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 0 1?= : ");
    Serial.println(KeyFramePause_0_1);
    SendOSCMessage("/modus_0/pause_anzeige_k1", KeyFramePause_0_1);
  }
  if (modus == 1)
  {
    KeyFramePause_1_1 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 1 1?= : ");
    Serial.println(KeyFramePause_1_1);
    SendOSCMessage("/modus_1/pause_anzeige_1", KeyFramePause_1_1);
  }
  if (modus == 2)
  { //gibts nicht
    KeyFramePause_2_1 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 2 1?= : ");
    Serial.println(KeyFramePause_2_1);
    SendOSCMessage("/modus_2/pause_anzeige_1", KeyFramePause_2_1);
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
  stepperX.setMaxSpeed(30000);
  stepperX.setAcceleration(2500);
  stepperY.setMaxSpeed(30000);
  stepperY.setAcceleration(2500);
  stepperZ.setMaxSpeed(30000);
  stepperZ.setAcceleration(2500);

  if (modus == 0)
  {
    Serial.print("View Keyframe 0 2?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_0_2);
      stepperY.moveTo(KeyFramePan_0_2);
      stepperZ.moveTo(KeyFrameTilt_0_2);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 2?= : ");
    Serial.println(view);
    {
      if (view == 1)
      {
        stepperX.moveTo(KeyFramePosition_1_2);
        stepperY.moveTo(KeyFramePan_1_2);
        stepperZ.moveTo(KeyFrameTilt_1_2);
        goModus = -1;
        go = 1;
        SendOSCMessage("/start_label/color", "red");
        SendOSCMessage("/start_label", "Stop");
        SendOSCMessage("/start/color", "red");
        SendOSCMessage("/start", 1);
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
        stepperX.moveTo(KeyFramePosition_2_2);
        stepperY.moveTo(KeyFramePan_2_2);
        stepperZ.moveTo(KeyFrameTilt_2_2);
        goModus = -1;
        go = 1;
        SendOSCMessage("/start_label/color", "red");
        SendOSCMessage("/start_label", "Stop");
        SendOSCMessage("/start/color", "red");
        SendOSCMessage("/start", 1);
      }
    }
  }
}

void BeschleunigungKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 2. KeyFrames
  if (modus == 0)
  {
    KeyFrameBeschleunigung_0_2 = StandartAcc * (1.05 - msg.getFloat(0));
    Serial.print("KeyFrameBeschleunigung_0_2?= : ");
    Serial.println(KeyFrameBeschleunigung_0_2);
  }

  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_2 = StandartAcc * (1.05 - msg.getFloat(0));
    Serial.print("KeyFrameBeschleunigung_1_2?= : ");
    Serial.println(KeyFrameBeschleunigung_1_2);
  }
}

void DauerKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 2. KeyFrames
  if (modus == 0)
  {
    KeyFrameDauer_0_2 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 0 2?= : ");
    Serial.println(KeyFrameDauer_0_2);
    SendOSCMessage("/modus_0/dauer_anzeige_k2", zeitdesMoves(KeyFrameDauer_0_2, StandartAcc * 0.5, abs(KeyFramePosition_0_2 - KeyFramePosition_0_1)));
  }
  if (modus == 1)
  {
    KeyFrameDauer_1_2 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 1 2?= : ");
    Serial.println(KeyFrameDauer_1_2);
    SendOSCMessage("/modus_1/dauer_anzeige_2", zeitdesMoves(KeyFrameDauer_1_2, KeyFrameBeschleunigung_1_2, abs(KeyFramePosition_1_2 - KeyFramePosition_1_3)));
  }
  if (modus == 2)
  {
    KeyFrameDauer_2_2 = 200 * (0.005 + msg.getFloat(0));
    Serial.print("KeyFrameDauer 2 2?= : ");
    Serial.println(KeyFrameDauer_2_2);
    SendOSCMessage("/modus_2/dauer_anzeige_2", KeyFrameDauer_2_2);
    updateSequenzDauer();
  }
}

void PauseKeyFrame2(OSCMessage &msg, int addrOffset)
{ //Pause des 2. KeyFrames
  if (modus == 0)
  {
    KeyFramePause_0_2 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 0 2?= : ");
    Serial.println(KeyFramePause_0_2);
    SendOSCMessage("/modus_0/pause_anzeige_k2", KeyFramePause_0_2);
  }
  if (modus == 1)
  {
    KeyFramePause_1_2 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 1 2?= : ");
    Serial.println(KeyFramePause_1_2);
    SendOSCMessage("/modus_1/pause_anzeige_2", KeyFramePause_1_2);
  }
  if (modus == 2)
  {
    KeyFramePause_2_2 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 2 2?= : ");
    Serial.println(KeyFramePause_2_2);
    SendOSCMessage("/modus_2/pause_anzeige_2", KeyFramePause_2_2);
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
  stepperX.setMaxSpeed(30000);
  stepperX.setAcceleration(2500);
  stepperY.setMaxSpeed(30000);
  stepperY.setAcceleration(2500);
  stepperZ.setMaxSpeed(30000);
  stepperZ.setAcceleration(2500);

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 3?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_1_3);
      stepperY.moveTo(KeyFramePan_1_3);
      stepperZ.moveTo(KeyFrameTilt_1_3);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }

  if (modus == 2)
  {
    Serial.print("View Keyframe 2 3?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_2_3);
      stepperY.moveTo(KeyFramePan_2_3);
      stepperZ.moveTo(KeyFrameTilt_2_3);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }
}

void BeschleunigungKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 3. KeyFrames

  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_3 = StandartAcc * (1.05 - msg.getFloat(0));
    Serial.print("KeyFrameBeschleunigung_1_3?= : ");
    Serial.println(KeyFrameBeschleunigung_1_3);
  }
}

void DauerKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 3. KeyFrames
  if (modus == 1)
  {
    KeyFrameDauer_1_3 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 1 3?= : ");
    Serial.println(KeyFrameDauer_1_3);
    SendOSCMessage("/modus_1/dauer_anzeige_3", zeitdesMoves(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_3 - KeyFramePosition_1_4)));
  }
  if (modus == 2)
  { //gibts nicht oder ????
    KeyFrameDauer_2_3 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 2 3?= : ");
    Serial.println(KeyFrameDauer_2_3);
    SendOSCMessage("/modus_2/dauer_anzeige_3", zeitdesMoves(KeyFrameDauer_2_3, KeyFrameBeschleunigung_2_3, abs(KeyFramePosition_2_3 - KeyFramePosition_2_1)));
  }
}

void PauseKeyFrame3(OSCMessage &msg, int addrOffset)
{ //Pause des 3. KeyFrames
  if (modus == 1)
  {
    KeyFramePause_1_3 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 1 3?= : ");
    Serial.println(KeyFramePause_1_3);
    SendOSCMessage("/modus_1/pause_anzeige_3", KeyFramePause_1_3);
  }

  if (modus == 2)
  {
    KeyFramePause_2_3 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 2 3?= : ");
    Serial.println(KeyFramePause_2_3);
    SendOSCMessage("/modus_2/pause_anzeige_3", KeyFramePause_2_3);
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
  stepperX.setMaxSpeed(30000);
  stepperX.setAcceleration(2500);
  stepperY.setMaxSpeed(30000);
  stepperY.setAcceleration(2500);
  stepperZ.setMaxSpeed(30000);
  stepperZ.setAcceleration(2500);

  if (modus == 1)
  {
    Serial.print("View Keyframe 1 4?= : ");
    Serial.println(view);
    if (view == 1)
    {
      stepperX.moveTo(KeyFramePosition_1_4);
      stepperY.moveTo(KeyFramePan_1_4);
      stepperZ.moveTo(KeyFrameTilt_1_4);
      goModus = -1;
      go = 1;
      SendOSCMessage("/start_label/color", "red");
      SendOSCMessage("/start_label", "Stop");
      SendOSCMessage("/start/color", "red");
      SendOSCMessage("/start", 1);
    }
  }
}

void BeschleunigungKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Beschleunigung des 4. KeyFrames
  if (modus == 1)
  {
    KeyFrameBeschleunigung_1_4 = StandartAcc * (1.05 - msg.getFloat(0));
    Serial.print("KeyFrameBeschleunigung_1_4?= : ");
    Serial.println(KeyFrameBeschleunigung_1_4);
  }
}

void DauerKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Geschwindigkeit des 4. KeyFrames
  if (modus == 1)
  {
    KeyFrameDauer_1_4 = StandartSpeed * (1.02 - msg.getFloat(0));
    Serial.print("KeyFrameDauer 1 4?= : ");
    Serial.println(KeyFrameDauer_1_4);
    SendOSCMessage("/modus_1/dauer_anzeige_4", zeitdesMoves(KeyFrameDauer_1_4, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_4 - KeyFramePosition_1_1)));
  }
}

void PauseKeyFrame4(OSCMessage &msg, int addrOffset)
{ //Pause des 4. KeyFrames
  if (modus == 1)
  {
    KeyFramePause_1_4 = 30 * msg.getFloat(0);
    Serial.print("KeyFramePause 1 4?= : ");
    Serial.println(KeyFramePause_1_4);
    SendOSCMessage("/modus_1/pause_anzeige_4", KeyFramePause_1_4);
  }
}

#pragma endregion
#pragma region etc
//Modus_2---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void FPS(OSCMessage &msg, int addrOffset)
{
  fps = 45 * ((1.0f / 3.0f) + msg.getFloat(0));
  fps = int(fps);

  Serial.print("fps: ");
  Serial.println(fps);
  SendOSCMessage("/modus_2/fps_anzeige", fps);
  updateSequenzDauer();
}

void Intervall1(OSCMessage &msg, int addrOffset)
{
  intervall1 = 100 * (0.01 + msg.getFloat(0));
  Serial.print("Intervall1 1?= : ");
  Serial.println(intervall1);
  SendOSCMessage("/modus_2/intervall_anzeige_1", intervall1);
  updateSequenzDauer();
}

void Intervall2(OSCMessage &msg, int addrOffset)
{
  intervall2 = 100 * (0.01 + msg.getFloat(0));
  Serial.print("Intervall2 1?= : ");
  Serial.println(intervall2);
  SendOSCMessage("/modus_2/intervall_anzeige_2", intervall2);
  updateSequenzDauer();
}

// UI Updates --------------------------------------------------------

void updateSequenzDauer()
{
  float sD1;
  float sD2;
  bilder1 = (KeyFrameDauer_2_1 * 60) / intervall1;
  bilder2 = (KeyFrameDauer_2_2 * 60) / intervall2;
  sD1 = bilder1 / fps; //Dauer in Minuten
  sD2 = bilder2 / fps;

  SendOSCMessage("/modus_2/sequenzdauer_anzeige_1", sD1);
  SendOSCMessage("/modus_2/sequenzdauer_anzeige_2", sD2);
}

void updateUI() //updated die UI mit aktueller Poition der einzelnen Achsen.
{
  SendOSCMessage("/slider_fader", float(float(SliderPosition) / float(MaxSliderPosition)));
  SendOSCMessage("/pan_position", float(Pankopf) / float(MaxPankopf));
  SendOSCMessage("/tilt_position", float(Tiltkopf) / float(MaxTiltkopf));
  SendOSCMessage("/modus_1/reverse_toggle", reversemsg);
}

void updateUIBIG()
{ //updated alle Elemente der UI, seltener, um Nachrichtenflut zu vermeiden. ((vllt mit OSC Bundle Lösen ??))

  SliderPosition = stepperX.currentPosition();
  Tiltkopf = stepperZ.currentPosition();
  Pankopf = stepperY.currentPosition();

  changeModus(modus); //updated den Start knopf

  //Modus 0
  SendOSCMessage("/modus_0/keyframe_1", KeyFrame_0_1);
  SendOSCMessage("/modus_0/keyframe_2", KeyFrame_0_2);

  SendOSCMessage("/modus_0/dauer_anzeige_k1", zeitdesMoves(KeyFrameDauer_0_1, StandartAcc * 0.5, abs(KeyFramePosition_0_1 - KeyFramePosition_0_2)));
  SendOSCMessage("/modus_0/pause_anzeige_k1", KeyFramePause_0_1);
  SendOSCMessage("/modus_0/dauer_anzeige_k2", zeitdesMoves(KeyFrameDauer_0_2, StandartAcc * 0.5, abs(KeyFramePosition_0_2 - KeyFramePosition_0_1)));
  SendOSCMessage("/modus_0/pause_anzeige_k2", KeyFramePause_0_2);

  //Modus 1
  SendOSCMessage("/modus_1/keyframe_1", KeyFrame_1_1);
  SendOSCMessage("/modus_1/keyframe_2", KeyFrame_1_2);
  SendOSCMessage("/modus_1/keyframe_3", KeyFrame_1_3);
  SendOSCMessage("/modus_1/keyframe_4", KeyFrame_1_4);

  SendOSCMessage("/modus_1/dauer_anzeige_1", zeitdesMoves(KeyFrameDauer_1_1, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_1 - KeyFramePosition_1_2)));
  SendOSCMessage("/modus_1/pause_anzeige_1", KeyFramePause_1_1);
  SendOSCMessage("/modus_1/dauer_anzeige_2", zeitdesMoves(KeyFrameDauer_1_2, KeyFrameBeschleunigung_1_2, abs(KeyFramePosition_1_2 - KeyFramePosition_1_3)));
  SendOSCMessage("/modus_1/pause_anzeige_2", KeyFramePause_1_2);
  SendOSCMessage("/modus_1/dauer_anzeige_3", zeitdesMoves(KeyFrameDauer_1_3, KeyFrameBeschleunigung_1_3, abs(KeyFramePosition_1_3 - KeyFramePosition_1_4)));
  SendOSCMessage("/modus_1/pause_anzeige_3", KeyFramePause_1_3);
  SendOSCMessage("/modus_1/dauer_anzeige_4", zeitdesMoves(KeyFrameDauer_1_4, KeyFrameBeschleunigung_1_1, abs(KeyFramePosition_1_4 - KeyFramePosition_1_1)));
  SendOSCMessage("/modus_1/pause_anzeige_4", KeyFramePause_1_4);

  //Modus 2
  SendOSCMessage("/modus_2/keyframe_1", KeyFrame_2_1);
  SendOSCMessage("/modus_2/keyframe_2", KeyFrame_2_2);
  SendOSCMessage("/modus_2/keyframe_3", KeyFrame_2_3);
  SendOSCMessage("/modus_2/fps_anzeige", fps);
  updateSequenzDauer();
}

#pragma endregion
#pragma region settings
// Settings--------------------------------------------------------------------------------------------------------------------------------------------

void SliderEnable(OSCMessage &msg, int addrOffset) //simpler Knopf zum deaktiviern von AchseX
{
  slider_enable = msg.getFloat(0);
  digitalWrite(EN_PINX, slider_enable);

  Serial.print("Slider Enable?= : ");
  Serial.println(slider_enable);
}

void PanEnable(OSCMessage &msg, int addrOffset) //simpler Knopf zum deaktiviern von AchseY
{
  pan_enable = msg.getFloat(0);
  digitalWrite(EN_PINY, pan_enable);

  Serial.print("Pan Enable?= : ");
  Serial.println(pan_enable);
}

void TiltEnable(OSCMessage &msg, int addrOffset) //simpler Knopf zum deaktiviern von AchseZ
{
  tilt_enable = msg.getFloat(0);
  digitalWrite(EN_PINZ, tilt_enable);

  Serial.print("Tilt Enable?= : ");
  Serial.println(tilt_enable);
}

void Kalibrieren(OSCMessage &msg, int addrOffset)
{
  kalibrieren = msg.getFloat(0);

  sliderAusmessen(); //Startet die Ausmeessen funktion

  Serial.print("Kalibrieren?= : ");
  Serial.println(kalibrieren);
}

void Reset(OSCMessage &msg, int addrOffset)
{
  reset = msg.getFloat(0);

  driverX.reset(); //reset den treiber per spi
  driverY.reset();
  driverZ.reset();

  stepperX.setCurrentPosition(stepperX.currentPosition()); //stoppt den stepper auf seiner aktuellen position
  stepperY.setCurrentPosition(stepperY.currentPosition());
  stepperZ.setCurrentPosition(stepperZ.currentPosition());
  go = 0;

  NextKeyFrame[0] = 0;
  NextKeyFrame[1] = 0;
  NextKeyFrame[2] = 0;

  Serial.print("Reset= : ");
  Serial.println(reset);
}

#pragma endregion
#pragma region OSCMessageRecive
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
      //ModusAuswahl
      msgIN.route("/modus_0/s", Modus0);
      msgIN.route("/modus_1/s", Modus1);
      msgIN.route("/modus_2/s", Modus2);
      msgIN.route("/modus_3/s", Modus3);

      msgIN.route("/start", start); //Startknöpfe

      //in jedem Modus die Fader/encoder zur direkten Steuerung
      msgIN.route("/slider_fader", SliderBewegung);
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
      msgIN.route("/modus_2/view_1", ViewKeyFrame1);
      msgIN.route("/modus_2/dauer_fader_1", DauerKeyFrame1);
      msgIN.route("/modus_2/intervall_fader_1", Intervall1);

      msgIN.route("/modus_2/keyframe_2", Keyframe2); //KeyFrame 2, Modus 2
      msgIN.route("/modus_2/view_2", ViewKeyFrame2);
      msgIN.route("/modus_2/dauer_fader_2", DauerKeyFrame2);
      msgIN.route("/modus_2/intervall_fader_2", Intervall2);

      msgIN.route("/modus_2/keyframe_3", Keyframe3); //KeyFrame 3, Modus 2
      msgIN.route("/modus_2/view_3", ViewKeyFrame3);

      //Modus_3
      msgIN.route("/modus_3/pan_tilt", Rotation);
      msgIN.route("/modus_3/slider_fader", Bewegung);

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

///////////////////////////////////////////////////////////////////////////////////////7
//
//    LOOP
//
//
void loop()
{

  OSCMsgReceive(); //& empfangen von OSC Messages

  //UI UPDATES
  static uint32_t last_time;
  static uint32_t last_time_long;
  if (millis() - last_time > 200)
  { //nur alle ... millisekunden ausführen, vermeidet unnötige NachrichtenFlut

    updateUI();
    //debugOutput();

    last_time = millis();
  }
  if (millis() - last_time_long > 10000)
  { //nur alle 10s ausführen, falls das Ipad disconnected ist, werden alle Buttons und Werte geupdated

    updateUIBIG();
    last_time_long = millis();
  }

  //randomMovement(); // Debug funktion
  // Stepper .run() functions
  if (go == 1)
  {

    // stepper runnt nur wenn er imm erlaubten bereich ist.
    /*if ((stepperX.currentPosition() == 0 && stepperX.distanceToGo() < 0)||(stepperX.currentPosition() == MaxSliderPosition && stepperX.distanceToGo() > 0)) {
      Serial.println("Endstop hit");
    }else {
      stepperX.run();
    }

    if ((stepperY.currentPosition() == 0 && stepperY.distanceToGo() < 0)||(stepperY.currentPosition() == MaxPankopf && stepperY.distanceToGo() > 0)) {
      Serial.println("Endstop hit");
    }else {
      stepperY.run(); 
    }

    if ((stepperZ.currentPosition() == 0 && stepperZ.distanceToGo() < 0)||(stepperZ.currentPosition() == MaxTiltkopf && stepperZ.distanceToGo() > 0)) {
      Serial.println("Endstop hit");
    }else {
      stepperZ.run();
    }*/

    stepperX.run();
    stepperY.run();
    stepperZ.run();

    SliderPosition = stepperX.currentPosition();
    Tiltkopf = stepperZ.currentPosition();
    Pankopf = stepperY.currentPosition();
  }

  // check, ob das Ziel erreicht wurde.
  if (go == 1)
  {
    //VIEW MODUS
    if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0 && stepperZ.distanceToGo() == 0 && goModus == -1)
    { // goModus -1 ist, wenn view an einem Keyframe gedrückt wurde.

      Serial.println("view Done"); //
      go = 0;
      SendOSCMessage("/start_label/color", "green");
      SendOSCMessage("/start_label", "Start");
      SendOSCMessage("/start/color", "green");
      SendOSCMessage("/start", 0);
    }

    //STALLGUARD MODUS
    else if (goModus == -3 || goModus == -2)
    { // run Stalldetect wenn slider ausgemessen wird.

      stallDetect(millis());
    }

    //EIGENTLICHER CHECK ALLER ANDEREN MODI
    else if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0 && stepperZ.distanceToGo() == 0 && goModus != 3)
    {
      if (goModus == 2)
      {
        if (triggeredPicture == false)
        {
          //trigger Picture
          //takePicture();
          triggeredPicture = true;

          Serial.println("CLICK");
        }
        if (millis() > modus2Waiting)
        { //wenn lang genug gewartet wurde weiterbewegen
          Serial.print("MovinG: ");
          Serial.println(stepsbetweenPicturesX);
          stepperX.move(stepsbetweenPicturesX);
          stepperY.move(stepsbetweenPicturesY);
          stepperZ.move(stepsbetweenPicturesZ);
          triggeredPicture = false; // bereit ein weiteres Bild zu machen
          modus2Waiting = millis() + modus2Interval;
        }
        if (abs(stepperX.currentPosition() - modus2target) < stepsbetweenPicturesX) //wenn das keyframe beim nächsten schritt erreciht werden würde.
        {

          Serial.print("next Keyframe: ");
          Serial.println(NextKeyFrame[goModus]);
          if (NextKeyFrame[goModus] >= 3)
          {
            NextKeyFrame[goModus] = 0;
            go = 0; //Slider anhalten am Ende

            SendOSCMessage("/start_label/color", "green");
            SendOSCMessage("/start_label", "Start");
            SendOSCMessage("/start/color", "green");
            SendOSCMessage("/start", 0);
          }
          else
          {
            gotoKeyframe(NextKeyFrame[goModus]);
            NextKeyFrame[goModus] = NextKeyFrame[goModus] + 1; //nächsten Frame
          }
        }
      }
      else
      {
        if (isPause == true)
        {

          if (millis() > pause_time)
          {
            Serial.print("next Keyframe: ");
            Serial.println(NextKeyFrame[goModus]);

            gotoKeyframe(NextKeyFrame[goModus]);
            isPause = false;

            if (reversemsg == 0)
            {
              NextKeyFrame[goModus] = NextKeyFrame[goModus] + 1; //nächsten Frame
            }
            else
            {
              NextKeyFrame[goModus] = NextKeyFrame[goModus] - 1; //Frame vorher
            }

            //check , ob nächster Frame wieder zurück muss;
            if (reversemsg == 1 && NextKeyFrame[goModus] <= 0)
            {
              reversemsg = 0;
              SendOSCMessage("/modus_1/revese_toggle", false);
            }
            else
            {
              if (goModus == 0)
              {
                if (NextKeyFrame[goModus] >= 1)
                {
                  reversemsg = 1;
                  SendOSCMessage("/modus_1/revese_toggle", true);
                }
              }
              else if (goModus == 1)
              {
                if (NextKeyFrame[goModus] >= 3)
                {
                  reversemsg = 1;
                  SendOSCMessage("/modus_1/revese_toggle", true);
                }
              }
            }
          }
        }
        else
        {
          isPause = true;
          pause_time = millis() + pause_timeKeyframe;
          Serial.println("Pause Start");
        }
      }
    }
  }
}
