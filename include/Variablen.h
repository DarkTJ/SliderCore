//Variablen:


int Geschwindigkeit; //Geschwindikeit des Sliders

float Tiltkopf;
float Pankopf;
float SliderPosition;

//Variablen für KeyFrame Toggle
bool KeyFrame_0_1; //Keyframes 1
bool KeyFrame_1_1;
bool KeyFrame_2_1;

bool KeyFrame_0_2; //Keyframes 2
bool KeyFrame_1_2;
bool KeyFrame_2_2;

bool KeyFrame_1_3; //Keyframes 3
bool KeyFrame_2_3;

bool KeyFrame_1_4; //Keyframes 4

//Variablen für KeyFrame Position
float KeyFramePosition_0_1;
float KeyFramePosition_1_1;
float KeyFramePosition_2_1;

float KeyFramePosition_0_2;
float KeyFramePosition_1_2;
float KeyFramePosition_2_2;

float KeyFramePosition_1_3;
float KeyFramePosition_2_3;

float KeyFramePosition_1_4;

//Variablen für KeyFrame Pan
float KeyFramePan_0_1;
float KeyFramePan_1_1;
float KeyFramePan_2_1;

float KeyFramePan_0_2;
float KeyFramePan_1_2;
float KeyFramePan_2_2;

float KeyFramePan_1_3;
float KeyFramePan_2_3;

float KeyFramePan_1_4;

//Variablen für KeyFrame Tilt
float KeyFrameTilt_0_1;
float KeyFrameTilt_1_1;
float KeyFrameTilt_2_1;

float KeyFrameTilt_0_2;
float KeyFrameTilt_1_2;
float KeyFrameTilt_2_2;

float KeyFrameTilt_1_3;
float KeyFrameTilt_2_3;

float KeyFrameTilt_1_4;

//Variablen für KeyFrame Beschleunigung
float KeyFrameBeschleunigung_0_1;
float KeyFrameBeschleunigung_1_1;
float KeyFrameBeschleunigung_2_1;

float KeyFrameBeschleunigung_0_2;
float KeyFrameBeschleunigung_1_2;
float KeyFrameBeschleunigung_2_2;

float KeyFrameBeschleunigung_1_3;
float KeyFrameBeschleunigung_2_3;

float KeyFrameBeschleunigung_1_4;

//Variablen für KeyFrame Beschleunigung
float KeyFrameDauer_0_1;
float KeyFrameDauer_1_1;
float KeyFrameDauer_2_1;

float KeyFrameDauer_0_2;
float KeyFrameDauer_1_2;
float KeyFrameDauer_2_2;

float KeyFrameDauer_1_3;
float KeyFrameDauer_2_3;

float KeyFrameDauer_1_4;

//Variablen für KeyFrame Pausendauer
float KeyFramePause_0_1;
float KeyFramePause_1_1;
float KeyFramePause_2_1;

float KeyFramePause_0_2;
float KeyFramePause_1_2;
float KeyFramePause_2_2;

float KeyFramePause_1_3;
float KeyFramePause_2_3;

float KeyFramePause_1_4;

// Modus 2 
float fps;
float sequenzdauer1;
float sequenzdauer2;
float intervall1;
float intervall2;

//Modus 3 Einlesen
float rotation;

bool go; //ist der Slider an oder ist der Slider aus?
bool reversemsg; // Modus 1 reverse an oder aus 

float Beschleunigung; //Beschleunigungsart zwischen linear und smooth

int Bildzahl;   //Anzahl der Bilder zwischen zwei Keyframes im Modus2
int Wartedauer; //Zeit zwischen den Bildern

int modus; //Nummer des aktuellen Modoi

int bilder; //Anzahl der Bilder

//Slider
float SliderPositon;
float SliderPan;
float SliderTilt;

//Settings
bool pan_enable;
bool slider_enable;
bool tilt_enable;
bool kalibrieren;
bool reset;
