 
//Variablen:


int Geschwindigkeit; //Geschwindikeit des Sliders

long Tiltkopf;
long MaxTiltkopf;
long Pankopf;
long MaxPankopf;
long SliderPosition;
long MaxSliderPosition;

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

// Aktuelle Status jedes Modi
int NextKeyFrame[3]; // für jeden Modi eine Position in der Liste


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
float KeyFrameBeschleunigung_2_1 = 10000;

float KeyFrameBeschleunigung_0_2;
float KeyFrameBeschleunigung_1_2;
float KeyFrameBeschleunigung_2_2 = 10000;

float KeyFrameBeschleunigung_1_3;
float KeyFrameBeschleunigung_2_3 = 10000;

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
float fps = 30;
float sequenzdauer1;
float sequenzdauer2;
float intervall1;
float intervall2;
float stepsbetweenPicturesX;
float stepsbetweenPicturesY;
float stepsbetweenPicturesZ;
int bilder1; //Anzahl der Bilder
int bilder2; //Anzahl der Bilder
uint32_t modus2Waiting;
float modus2Interval;
bool triggeredPicture;
float modus2target;


int modus; //Nummer des aktuellen Modi in UI   
bool go; //ist der Slider an oder ist der Slider aus?
int goModus;    // In welchem Modus läuft der Slider gerade;
bool isPause;
unsigned long pause_time;
unsigned long pause_timeKeyframe;

bool reversemsg; // reverse an oder aus, kann in Modus 1 manuel gändert werden.
bool accToHigh;
float speedGlobal;
float accGlobal;





//Settings
bool pan_enable;
bool slider_enable;
bool tilt_enable;
bool kalibrieren;
bool reset;
