/*
** DasBoot Controller
**
*/

// COMPILE SWITCHES
#define WAIT_FOR_SERIAL false
#define ECHO_NMEA false

typedef enum  { BOOT, BUGGY } VehicleType;
#define N_VEHICLES 2
const VehicleType vehicle = BOOT;

#include <Arduino.h>
// Libraries for timer processes
#include <Zero_Timer_Wrap.h>
#include <FireTimer.h>

// Libraries for display
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ezButton.h>
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

// Libraries for encoder
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#define SHORT_PRESS_TIME 0.05
#define LONG_PRESS_TIME 0.5

// FEATHER PINS
#define MOTOR_ENABLE_PIN 12
#define MOTOR_DRIVE1 11
#define MOTOR_DRIVE2 10

#define RUDDER_PIN A1

// Libraries for Control
#include <QuickPID.h>

// Libraries for measurements
#include <angle_support.h>


#define OTTO_LEG_W 75
#define OTTO_LEG_H 75
const float ottoOffsets[22][2] = {
   { 1 * OTTO_LEG_H,    13 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,    10 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,    10 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,    12 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,    12 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     8 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     8 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     9 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     7 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     8 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     8 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     5 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     5 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     6 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     4 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     5 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     5 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     1 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     1 * OTTO_LEG_W },
   { 3 * OTTO_LEG_H,     3 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     3 * OTTO_LEG_W },
   { 1 * OTTO_LEG_H,     0 * OTTO_LEG_W }
};

int nOttoWpts = 22;

const float boxOffsets[5][2] = {
   { 0.0,     0.0 },
   { 0.0,    100.0 },
   { 100.0,  100.0 },
   { 100.0,    0.0 },
   { 0.0,     0.0 },
};
int nBoxWpts = 5;

#define NUM_ROUTES 3

////////////////////////////////////////////////////////////////////////
// TIMER PROCESSES INIT
////////////////////////////////////////////////////////////////////////
float fastProcessFreq = 1000.0;
float slowProcessFreq = 10.0;
FireTimer sendSerial;
#define sendSerialPeriod 100
FireTimer navRxTimeout;
#define navRxTimeoutPeriod 500
FireTimer homeLatch;
#define homeLatchPeriod 250

////////////////////////////////////////////////////////////////////////
// DISPLAY INIT
////////////////////////////////////////////////////////////////////////
#define NLINES 8
#define LINE_SIZE 24
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
char display_text[NLINES][LINE_SIZE];
ezButton buttonA(BUTTON_A);  
ezButton buttonB(BUTTON_B);  
ezButton buttonC(BUTTON_C);

char test_text[NLINES][LINE_SIZE];

////////////////////////////////////////////////////////////////////////
// ENCODER INIT
////////////////////////////////////////////////////////////////////////
#define SS_SWITCH        24
#define SS_NEOPIX        6

#define PORT_ENC_ADDR   0x36

Adafruit_seesaw PortEncoder;
seesaw_NeoPixel PortPixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t portEncoderPosition, prevEncoderPosition=0;


////////////////////////////////////////////////////////////////////////
// MODE INIT
////////////////////////////////////////////////////////////////////////

// MAIN
enum ModeState { MODE_SEL, STANDBY, AUTO, MANUAL, COMM, CONTROL };
ModeState mode = STANDBY;
ModeState targetMode = STANDBY;

const char* modes[] = {"MODE_SEL", "STANDBY", "AUTO", "MANUAL", "COMM", "CONTROL"};
int nModes = 6;

// CONTROL MAIN
enum ControlModeState { MENU, SET_HOME, TYPE, GAINS, ROUTE };
ControlModeState controlMode = MENU;
ControlModeState controlTargetMode = SET_HOME;

const char* controlModes[] = {"MENU","SET_HOME", "TYPE", "GAINS", "ROUTE"};
int nControlModes = 5;

// CONTROL TYPE
enum ControlTypeState { SELECT_TYPE, HEADING, BEARING, XTE };
ControlTypeState controlTypeMode = HEADING;
ControlTypeState controlTypeTargetMode = HEADING;
ControlTypeState controlType = HEADING;

const char* controlTypeModes[] = {"SELECT_TYPE", "HEADING", "BEARING", "XTE"};
int nControlTypeModes = 4;

// CONTROL GAINS
enum ControlGainState { SELECT_GAIN, SELECT_KP, SELECT_KI, SELECT_KD };
ControlGainState controlGainMode = SELECT_GAIN;
ControlGainState controlGainTargetMode = SELECT_KP;

const char* controlGainModes[] = {"SELECT_GAIN", "SELECT_KP", "SELECT_KI", "SELECT_KD"};
int nControlGainModes = 4;

// CONTROL ROUTE
enum ControlRouteState { SELECT_ROUTE, FREE, BOX, OTTO };
ControlRouteState controlRouteMode = SELECT_ROUTE;
ControlRouteState controlRouteTargetMode = FREE;

const char* controlRouteModes[] = {"SELECT_ROUTE", "ADD WPT", "BOX", "OTTO"};
int nControlRouteModes = 4;

////////////////////////////////////////////////////////////////////////
// ACTUATOR INIT
////////////////////////////////////////////////////////////////////////
float motorTimeManual = 1.0;
float motorDriveTime = 0;
bool motorOn = false;
bool motorFwd = true;


////////////////////////////////////////////////////////////////////////
// SERIAL COMM INIT
////////////////////////////////////////////////////////////////////////
float hdmTime=0;

////////////////////////////////////////////////////////////////////////
// MEASUREMENT INIT
////////////////////////////////////////////////////////////////////////
float elapsedTime = 0;
float prevElapsedTime = 0,dt=0;
float pitch, roll, heading, track, turnRate, speed;
int trackValidity;
int rudderRaw = 0;
float rudderAngle = 0;
float rudderCenter = 2080.0;
float rudderScale = 180.0/2750.0;
float rudderScaleStbd = -45.0/(1520-rudderCenter);
float rudderScalePort = 45.0/(2690-rudderCenter);


////////////////////////////////////////////////////////////////////////
// CONTROL INIT
////////////////////////////////////////////////////////////////////////
float hdgTarget = 0;
float turnRateTarget = 0;
float hdgTarget360, hdg360;

float hdgError,trError;
float trMax = 5.0; // deg/sec
float trHdgMax = 45.0;
float trDeadZone = 0.25;
float hdgDeadZone = 1.0;
float trHdgCoef = -1.0*trMax/trHdgMax;

float hdgKp = 1.0;
float hdgKi = 0.1;
float hdgKd = 0.00;

float ctrlOutput = 0.0;
float turnCtrlOutput = 0.0;

float turnKp = 0.22;
float turnKi = 0.0;
float turnKd = 0.0;

float zeroDiff = 0;

//QuickPID headingPID(&hdgError, &ctrlOutput, &zeroDiff);
QuickPID turnRatePID(&trError, &turnCtrlOutput, &zeroDiff);

float rudderTarget = 0;
float rudderCtrlBand = 2;

int tmpInt;

////////////////////////////////////////////////////////////////////////
// Updates from brain box 
////////////////////////////////////////////////////////////////////////
//int mode = 0; // Standby
float setHeading = 0;
float KpBase = 1.4;
float KiBase = KpBase/20.0/1.0;
float Kp[N_VEHICLES] = {KpBase, 0.88 };
float Ki[N_VEHICLES] = {KiBase, KiBase};
float Kd[N_VEHICLES] = { 0, 0 };
float xteKp[N_VEHICLES] = { 4.0, 4.0 };
float xteKi[N_VEHICLES] = {0.5, 0.5};
float xteKd[N_VEHICLES] = { 0, 0 };
float currentMeas = 0;
char txSendBuffer[64];
float ctrlGain=1.0;
float headingDiff = 0;
float steeringCmd = 0;
float steerAdj;
int navSource = 0;
const float xteInterceptAngle = 30.0;
const float xteControlThreshold = 10.0;
float xteSteeringCmd = 0;
float xte=0,atd=0,rtd=0;
float interceptBearing;
float xteOffset;

float steeringCenterPoint[N_VEHICLES] = { 90.0, 96.0 };

struct Waypoint {
  float lat;
  float lon;
  float alt;
};

Waypoint here, home;
Waypoint waypointDB[25];

struct Route {
  char name[20];
  int nWaypoints;
  bool isRelative;
  Waypoint wpts[25];
};

Route routeFree, routeBoxRel, routeOtto;
int routeSel = 0;   // routeFree=0, routeBoxRel=1, routeHONABS=2, routeOtto=3;
int nRoutes = 3;

Route *RouteDB[5];

struct GuidanceVector {
  float bearing;
  float distance;
};

GuidanceVector vectorTo, vectorFrom, vectorWP, vectorXTE;


QuickPID headingPID(&headingDiff, &steeringCmd, &zeroDiff);
QuickPID xtePID(&xte,&xteSteeringCmd,&zeroDiff);

////////////////////////////////////////////////////////////////////////
// HIGH RATE PROCESS
////////////////////////////////////////////////////////////////////////

void fastProcess(void) {

}

////////////////////////////////////////////////////////////////////////
// SLOW RATE PROCESS
////////////////////////////////////////////////////////////////////////

void slowProcess(void) {

}

////////////////////////////////////////////////////////////////////////
// vector_to
// calculates the vector from one Waypoint to another
////////////////////////////////////////////////////////////////////////
GuidanceVector vector_to(Waypoint from, Waypoint to){
  GuidanceVector vectorResult;
  const float R = 6371e3;

  float lat1Rad = radians(from.lat);
  float lon1Rad = radians(from.lon);
  float lat2Rad = radians(to.lat);
  float lon2Rad = radians(to.lon);

  float dLon = lon2Rad - lon1Rad;
  float dLat = lat2Rad - lat1Rad;
  float y = sin(dLon)*cos(lat1Rad);
  float x = cos(lat1Rad)*sin(lat2Rad) - sin(lat1Rad)*cos(lat2Rad)*cos(dLon);
  float bearingRad = atan2(y,x);
  float bearingDeg = degrees(bearingRad);
  //float distance = sqrt(x^2 + y^2)*rad2deg*1852;

  float cLat = cos(lat1Rad);
  float distance = R * sqrt(dLat*dLat + cLat*cLat*dLon*dLon);

  if(bearingDeg < 0) {
    bearingDeg += 360;
  }


  vectorResult.bearing = bearingDeg;
  vectorResult.distance = distance;

  return(vectorResult);

}

////////////////////////////////////////////////////////////////////////
// MOTOR PROCESSES
////////////////////////////////////////////////////////////////////////

// CW Rotation
void motor_forward(void) {
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  digitalWrite(MOTOR_DRIVE1, LOW);
  digitalWrite(MOTOR_DRIVE2, HIGH);
}

// CCW Rotation
void motor_reverse(void) {
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  digitalWrite(MOTOR_DRIVE1, HIGH);
  digitalWrite(MOTOR_DRIVE2, LOW);
}

// Motor Stop
void motor_stop(void) {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(MOTOR_DRIVE1, LOW);
  digitalWrite(MOTOR_DRIVE2, LOW);
}

////////////////////////////////////////////////////////////////////////
// VALIDATE NMEA CHECKSUM PROCESS
////////////////////////////////////////////////////////////////////////

char nmea_checksum(const char *nmea_sentence) {
  char cs = 0;
  int maxLen = 100;
  int i=1;

  while(nmea_sentence[i] != '*' && nmea_sentence[i] != 0 && i <= maxLen ) {
    cs ^= nmea_sentence[i];
    i++;
  }

  if(i==maxLen) {
    cs = 0;
  }

  return(cs);

}

// Validates a full NMEA sentence (Returns true if valid, false if failed)
bool validate_NMEA_Checksum(const char* sentence) {
  // Sentences must start with '$' and contain '*'
  if (sentence[0] != '$') return false;
  
  byte calculatedXOR = 0;
  int i = 1; // Start right after '$'
  
  // Step through string until '*' is hit or string ends
  while (sentence[i] != '*' && sentence[i] != '\0') {
    calculatedXOR ^= sentence[i];
    i++;
  }
  
  // If we reached the end without finding '*', it's malformed
  if (sentence[i] != '*') return false;
  
  // Grab the 2 hex characters following '*'
  char hexHigh = sentence[i + 1];
  char hexLow = sentence[i + 2];
  
  // Convert those 2 characters back into a numeric byte value
  char hexArray[3] = { hexHigh, hexLow, '\0' };
  byte parsedChecksum = strtol(hexArray, NULL, 16);
  
  // Compare calculated vs parsed
  return (calculatedXOR == parsedChecksum);
}

// Not used here with String class message builder
bool add_NMEA_checksum(char *nmea_sentence){
  char cs;
  char endChars[10];
  cs = nmea_checksum(nmea_sentence);

  sprintf(endChars,"*%2X\n",cs);
  strcat(nmea_sentence,endChars);

  return true;
}

////////////////////////////////////////////////////////////////////////
// MENU PROCESS
////////////////////////////////////////////////////////////////////////
void display_menu(int nMenuItems, int selectedItem,const char **itemLabels) {


      // Format Display
      //Serial.print(modes[targetMode]);
      for(int m=0;m<nMenuItems;m++) {
        if(selectedItem == m) {
          sprintf(display_text[m+2],"==> %s",itemLabels[m]);
          //sprintf(test_text[m+1],"==> %s",itemLabels[m]);
          //Serial.println(test_text[m+1]);
        }
        else {
          sprintf(display_text[m+2],"    %s",itemLabels[m]);
          //sprintf(test_text[m+1],"    %s",itemLabels[m]);
          //Serial.println(test_text[m+1]);

        }        
      }

}

/////////////////////////////////////////////////////////////
//  menu
/////////////////////////////////////////////////////////////
int do_menu(int nMenuItems, int selectedItem,const char **itemLabels) {
          int selectResult = selectedItem;

          if(portEncoderPosition < prevEncoderPosition){
            selectResult += 1;
            if(selectResult >= nMenuItems) { selectResult = 1; }
          }
          if(portEncoderPosition > prevEncoderPosition){
            selectResult -= 1;
            // Note decrimenting enums creates and overflow from 0 to 255;
            if(selectResult <= 0) { selectResult = nMenuItems-1; }
          }
          //Serial.println(controlTargetMode);

          // Format Display
          display_menu(nMenuItems-1, selectedItem-1, &(itemLabels[1]) );

          return selectResult;
          
}


////////////////////////////////////////////////////////////////////////
// SETUP PROCESS
////////////////////////////////////////////////////////////////////////
void setup() {
  // 
  //
  
  // Comm Setup

  Serial.begin(115200);
  while (WAIT_FOR_SERIAL && !Serial) {
        // Wait for serial port to connect. Needed for native USB port only
        delay(10);
      }
  Serial.println("Serial connected!");

  // Pull up serial lines to reduce transcients due to noise 
  pinMode(0, INPUT_PULLUP);
  Serial1.begin(57600);



  // Setup time based processes
  setup_timer(1, true, fastProcessFreq, fastProcess) ;
  setup_timer(2, true, slowProcessFreq, slowProcess) ;
 
  // Start Serial Timer 
  sendSerial.begin(sendSerialPeriod);  


  // HW Setup
  analogReadResolution(12);
  

  //
  // Display Setup
  //
  display.begin(0x3C, true); // Address 0x3C default
  delay(100);
  
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);
  
  //pinMode(BUTTON_A, INPUT_PULLUP);
  //pinMode(BUTTON_B, INPUT_PULLUP);
  //pinMode(BUTTON_C, INPUT_PULLUP);
    // Setup Display Buttons
  buttonA.setDebounceTime(50); // set debounce time to 50 milliseconds
  buttonB.setDebounceTime(50); // set debounce time to 50 milliseconds
  buttonC.setDebounceTime(50); // set debounce time to 50 milliseconds

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);


  strcpy(display_text[0],"_DasBoot Controller__");  // 21 Char width max
  strcpy(display_text[1],"Line 1");
  strcpy(display_text[2],"Line 2");
  strcpy(display_text[3],"Line 3");
  strcpy(display_text[4],"Line 4");
  strcpy(display_text[5],"Line 5");
  strcpy(display_text[6],"Line 6");
  strcpy(display_text[7],"Line 7");

  for(int line=0;line<NLINES;line++) {
      display.println(display_text[line]);
  }
  display.display();

  //////// Encoder
  Serial.println("Looking for Port seesaw!");
  if (! PortEncoder.begin(PORT_ENC_ADDR) || ! PortPixel.begin(PORT_ENC_ADDR)) {
    Serial.println("Couldn't find seesaw on PORT_ENC_ADDR address");
    while(1) delay(10);
  }
  Serial.println("Port seesaw started");

  uint32_t version = ((PortEncoder.getVersion() >> 16) & 0xFFFF);
  if (version  != 4991){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(10);
  }
  Serial.println("Found Product 4991 - Port");

  // set not so bright!
  PortPixel.setBrightness(255);
  PortPixel.show();
  
  // use a pin for the built in encoder switch
  PortEncoder.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  Serial.println("ENCODER ..");
  portEncoderPosition = PortEncoder.getEncoderPosition();
  Serial.println(portEncoderPosition);

  Serial.println("Turning on interrupts");
  delay(10);
  PortEncoder.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  PortEncoder.enableEncoderInterrupt();

  // MOTOR Setup
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DRIVE1, OUTPUT);
  pinMode(MOTOR_DRIVE2, OUTPUT);

  Serial.println("Motor CW");
  motor_forward();
  delay(1000);

  Serial.println("Motor CCW");
  motor_reverse();
  delay(1000);

  Serial.println("Motor OFF");
  motor_stop();
  delay(100);

  // // Initialize Controllers
  // headingPID.SetTunings(hdgKp, hdgKi, hdgKd);
  // headingPID.SetMode(QuickPID::Control::timer);
  // headingPID.SetSampleTimeUs(50*1000);
  // headingPID.SetOutputLimits(-25.0, 25.0);
  // headingPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  // Serial.println(headingPID.GetMode());

  turnRatePID.SetTunings(turnKp, turnKi, turnKd);
  turnRatePID.SetMode(QuickPID::Control::timer);
  turnRatePID.SetSampleTimeUs(50*1000);
  turnRatePID.SetOutputLimits(-25.0, 25.0);
  turnRatePID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  Serial.println(turnRatePID.GetMode());

  // Initialize Heading Controller
  headingPID.SetTunings(Kp[vehicle], Ki[vehicle], Kd[vehicle]);
  headingPID.SetMode(QuickPID::Control::timer);
  headingPID.SetSampleTimeUs(50*1000);
  headingPID.SetOutputLimits(-25.0, 25.0);
  headingPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  if(vehicle==BOOT) {
    headingPID.SetOutputLimits(-40.0, 40.0);
  }
  else
  {
    headingPID.SetOutputLimits(-45.0, 45.0);
  }

  // Initialize XTE Controller
  xtePID.SetTunings(xteKp[vehicle], xteKi[vehicle], xteKd[vehicle]);
  xtePID.SetMode(QuickPID::Control::timer);
  xtePID.SetSampleTimeUs(50*1000);
  xtePID.SetOutputLimits(-25.0, 25.0);
  xtePID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  if(vehicle==BOOT) {
    xtePID.SetOutputLimits(-40.0, 40.0);
  }
  else
  {
    xtePID.SetOutputLimits(-45.0, 45.0);
  }

  // Initialize Guidance
  here.lat = 0;
  here.lon = 0;
  here.alt = 0;
  home.lat = 0;
  home.lon = 0;
  home.alt = 0;
  vectorTo.bearing = 0;
  vectorTo.distance = 0;
  vectorFrom.bearing = 0;
  vectorFrom.distance = 0;
  vectorWP.bearing = 0;
  vectorWP.distance = 0;
  vectorXTE.bearing = 0;
  vectorXTE.distance = 0;

  // Initialize Routes
  strcpy(routeFree.name,"FREE\0");
  routeFree.nWaypoints = 0;
  routeFree.isRelative = true;
  routeFree.wpts[0].lat = 0;
  routeFree.wpts[0].lon = 0;
  routeFree.wpts[0].alt = 0;
  routeFree.wpts[1].lat = 1;
  routeFree.wpts[1].lon = 1;
  routeFree.wpts[1].alt = 1;
  routeFree.wpts[2].lat = 2;
  routeFree.wpts[2].lon = 2;
  routeFree.wpts[2].alt = 2;

  Waypoint *idxWpt;

  strcpy(routeOtto.name,"OTTO\0");
  routeOtto.nWaypoints = nOttoWpts;
  routeOtto.isRelative = true;
  idxWpt = &routeOtto.wpts[0];
  for(int i=0; i<nOttoWpts; i++) {
    (idxWpt+i)->lat = ottoOffsets[i][0];
    (idxWpt+i)->lon = ottoOffsets[i][1];
    (idxWpt+i)->alt = 0;
  }

  strcpy(routeBoxRel.name,"BOX25\0");
  routeBoxRel.nWaypoints = nBoxWpts;
  routeBoxRel.isRelative = true;
  idxWpt = &routeBoxRel.wpts[0];
  for(int i=0; i<nOttoWpts; i++) {
    (idxWpt+i)->lat = boxOffsets[i][0];
    (idxWpt+i)->lon = boxOffsets[i][1];
    (idxWpt+i)->alt = 0;
  }

  RouteDB[0] = &routeFree;
  RouteDB[1] = &routeBoxRel;
  RouteDB[2] = &routeOtto;


  // Enable Timer Handlers
  Serial.println("Starting Timers");
  // Enable time based processes
  enable_timer(1,true);
  enable_timer(2,true);
  elapsedTime = (float)millis()/1000.0;
  dt = elapsedTime - prevElapsedTime;

}

////////////////////////////////////////////////////////////////////////
// LOOP PROCESS
////////////////////////////////////////////////////////////////////////

bool portButton;
bool portShortPress,portLongPress;
float portPressTime;
int32_t port_new_position;
String tempStr,nmeaStr = String(""),evtMessage = String(""), hdmMessage = String("");
float sendTime = 0;

char nmeaBuffer[256];
int nmeaBufferIndex = 0;
float ch_width[4];
float prevChWidth[4];
float prevSelectWidth;
bool navRecieved = false;
bool gpsRecieved = false;
float gpsLat = 0.0;
float gpsLon = 0.0;
float gpsAlt = 0.0;
int gpsNSats=0;
float msgElapsedTime;

bool guidanceInit = false;
bool guidanceUpdate = false;
bool firstHomePulse = true;
bool sendEvtMessage = false;

bool buttonShortPress = false;
bool buttonLongPress = false;
bool buttonRelease = false;
bool buttonHeld = false;
int buttonInputState = 0;
float buttonTime;


int wayPointNum = 0;
int nextWaypointNum = 0;
bool nextWaypointReady = false;
float nextWayPointDist = 6.0;

int guidanceMode = 0;
float arrivalAngle = 0;
float xteAngle = 0;
float xteBearing = 0;
float xteHeadingAdjust = 0;
float newSetHeading  = 0;

unsigned char rxBuffer[64];
int nChars=0;

float rudderError=0;

void loop() {
  
  char nmeaSentence[40];
  bool nmeaReady,nmeaStart; 
  char tempChars[40];

  bool waypointAdded = false;
  char sentCS;
  

  // Time Update
    // Collect Measurements
  elapsedTime = (float)millis()/1000.0;
  dt = elapsedTime - prevElapsedTime;
  //Serial.println(elapsedTime);
  //delay(100);

  rudderRaw = analogRead(RUDDER_PIN);
  int rudderOffsetRaw = rudderRaw - rudderCenter;
  if(rudderOffsetRaw <= 0) {  //STBD
    rudderAngle = rudderOffsetRaw * rudderScaleStbd;

  }
  else{
    rudderAngle = rudderOffsetRaw * rudderScalePort;
  }
  //rudderAngle = ((int)rudderRaw - rudderCenter) * rudderScale;
  //Serial.print(rudderRaw); Serial.print(" "); 
  //Serial.print(rudderOffsetRaw); Serial.print(" "); 
  //Serial.print(rudderAngle); Serial.print(" ");
  //Serial.print(rudderTarget);
  //Serial.println("");


  // if(elapsedTime >= sendTime) {
  //   Serial1.println("Message");
  //   sendTime = elapsedTime + 0.1;
  // }
  // Query UI Controls
  buttonA.loop(); 
  buttonB.loop(); 
  buttonC.loop(); 
  if (buttonA.isPressed()) { 
    Serial.println("--- BTN A ---");
  }
  if (buttonB.isPressed()) { Serial.println("--- BTN B ---"); }
  if (buttonC.isPressed()) { 
    Serial.println("--- BTN C ---");
  }

  // Handle Encoders
  buttonShortPress = false;
  buttonLongPress = false;
  buttonRelease = false;

  // Button Pressed and not held
  if (!PortEncoder.digitalRead(SS_SWITCH) & !buttonHeld) {
    buttonHeld = true;
    buttonTime = elapsedTime;
    //Serial.println("Button Pressed");
  }

  // Button up but held (released)
  if( PortEncoder.digitalRead(SS_SWITCH) & buttonHeld) {
    buttonHeld = false;
    buttonRelease = true;
    //Serial.println("Button Released");
  }


  if ( buttonInputState == 0) {
    if(buttonHeld && elapsedTime - buttonTime > LONG_PRESS_TIME) {
      buttonInputState = 1;
      buttonLongPress = true;
      //Serial.println("LONG PRESS");
    }
    else if (buttonRelease && elapsedTime - buttonTime > SHORT_PRESS_TIME) {
      buttonShortPress = true;
      //Serial.println("SHORT PRESS");
    }
  }
  else if (buttonInputState == 1) {
    if(buttonRelease) {
      buttonInputState = 0;
    }
  }

  //port_new_position = PortEncoder.getEncoderPosition(); 
  portEncoderPosition = PortEncoder.getEncoderPosition(); 
  //int32_t x = PortEncoder.getEncoderPosition();
  //Serial.println(port_new_position);
  // did we move arounde?
  // if (portEncoderPosition != port_new_position) {
  //   //Serial.print("Port Enc: ");
  //   Serial.print(port_new_position);         // display new position
  //   Serial.println(" ");
  //   if(port_new_position > portEncoderPosition) {
  //     motorDriveTime = elapsedTime + motorTimeManual;
  //     motorFwd = true;
  //     motorOn = true;
  //     //Serial1.println("FWD");
  //   }
  //   if(port_new_position < portEncoderPosition) {
  //     motorDriveTime = elapsedTime + motorTimeManual;
  //     motorFwd = false;
  //     motorOn = true;
  //     //Serial1.println("REV");
  //   }
  //   portEncoderPosition = port_new_position;      // and save for next round
  // }



  // Process NMEA Input
  nmeaReady = false;
  gpsRecieved = false;
  int nBytesRx = 0;


  while (Serial1.available() > 0 && nBytesRx < 250) {
    //Serial.write(Serial1.read());  // read it and send it out Serial (USB)
    nBytesRx++;
  //   //Serial.print("+");
  //   if((nBytesRx % 100) == 0) { Serial.print("+"); }
    int recievedByte = Serial1.read();
    char incomingByte = (char)recievedByte;
  //   //Serial.print("Received on Serial1: ");
  //   //Serial.print(incomingByte,HEX);
    //Serial.print(incomingByte);
    if(incomingByte == '$') {
      //Serial.print("-start-");
      nmeaStart = true;
      nmeaStr = String("");
      nmeaBufferIndex = 0;     
    }
    if(incomingByte == 10) 
    {
       //Serial.print("-END-");
       if(validate_NMEA_Checksum(nmeaStr.c_str())){
         nmeaReady = true;
       }
       else
       {
         Serial.println("##### NMEA CHECKSUM FAIL #####");
       }
       nmeaStart = false;
       //nmeaBuffer[nmeaBufferIndex] = 0;
       break;
    }
    if(nmeaStart == true) {
  //     nmeaBuffer[nmeaBufferIndex] = incomingByte;
       nmeaBufferIndex++;
       nmeaStr += incomingByte;
  //     //Serial.print("+");
  //     //Serial.print(incomingByte);
  //     //Serial.println("]");
  //     //for(int i = 0; i<10000000; i ++) {}
    }
    if(nmeaBufferIndex > 100) {
       nmeaStart = false;
       nmeaBufferIndex=0;
       break;
    }    
  }

  //Process NMEA Messages
  if(nmeaReady)
  {
    //Serial.print("Resending:");
    if ( false && ECHO_NMEA ) { Serial.println(nmeaStr); };
    Serial1.println(nmeaStr);
    //Serial.print("--->Parsing:");
    //Serial.println(nmeaStr);
    //Serial.println(nmeaStr.indexOf('*'));
    //    Serial.print("--->Buffer:");
    //Serial.println(nmeaBuffer);
    //Serial.println(nmeaStr.substring(1,6));

    if(nmeaStr.substring(1,6) == String("INNAV")) {
      //Serial.println("NAV MESSAGE");
      int firstComma = nmeaStr.indexOf(',');
      int secondComma = nmeaStr.indexOf(',',firstComma+1);
      //Serial.print(firstComma); Serial.print(" ");
      //Serial.print(secondComma); Serial.print(" ");
      String pString = nmeaStr.substring(firstComma+1,secondComma);
      //Serial.print(pString);Serial.print(" ");
      msgElapsedTime = pString.toFloat();
      //Serial.print(msgElapsedTime,3);Serial.print(" ");
      

      int thirdComma = nmeaStr.indexOf(',',secondComma+1);
      pString = nmeaStr.substring(secondComma+1,thirdComma);
      //Serial.print(pString);Serial.print(" ");
      pitch = pString.toFloat();
      //Serial.print(pitch);Serial.print(" ");
  

      int thisComma = thirdComma;
      int nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      roll = pString.toFloat();
      //Serial.print(roll);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      track = pString.toFloat();
      //Serial.print(track);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      trackValidity = pString.toInt();
      //Serial.print(trackValidity);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      turnRate = pString.toFloat();
      //Serial.print(turnRate);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      gpsLat = pString.toFloat();
      //Serial.print(gpsLat);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      gpsLon = pString.toFloat();
      //Serial.print(gpsLon);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      gpsAlt = pString.toFloat();
      //Serial.print(gpsAlt);Serial.print(" ");

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      speed = pString.toFloat();
      //Serial.print(speed);Serial.print(" ");

      int star = nmeaStr.indexOf('*');
      pString = nmeaStr.substring(nextComma+1,star);
      //Serial.print(pString);Serial.print(" ");
      gpsNSats = pString.toInt();
      //Serial.print(gpsNSats);Serial.print(" ");
      
      gpsRecieved = true;
      hdmTime = elapsedTime;
      //Serial.println("");
      //Serial.println("");

    }

    else if(nmeaStr.substring(1,6) == String("INATT")) {
      //Serial.println("ATT MESSAGE");
      int firstComma = nmeaStr.indexOf(',');
      int secondComma = nmeaStr.indexOf(',',firstComma+1);
      //Serial.print(firstComma); Serial.print(" ");
      //Serial.print(secondComma); Serial.print(" ");
      String pString = nmeaStr.substring(firstComma+1,secondComma);
      //Serial.print(pString);Serial.print(" ");
      pitch = pString.toFloat();
      //Serial.print(pitch);Serial.print(" ");
      //Serial.println("");

      int thirdComma = nmeaStr.indexOf(',',secondComma+1);
      pString = nmeaStr.substring(secondComma+1,thirdComma);
      //Serial.print(pString);Serial.print(" ");
      roll = pString.toFloat();
      //Serial.print(roll);Serial.print(" ");
      //Serial.println("");

      int star = nmeaStr.indexOf('*');
      pString = nmeaStr.substring(thirdComma+1,star);
      //Serial.print(pString);Serial.print(" ");
      heading = pString.toFloat();
      //Serial.print(heading);Serial.print(" ");
      
      //Serial.println("");

    }
    else if (nmeaStr.substring(1,6) == String("INHDM")) {
      //Serial.print("HDM MESSAGE ");
      hdmMessage = nmeaStr;
      hdmTime = elapsedTime;
      int firstComma = nmeaStr.indexOf(',');
      int secondComma = nmeaStr.indexOf(',',firstComma+1);
      String pString = nmeaStr.substring(firstComma+1,secondComma);
      float msgElapsedTime = pString.toFloat();
      //Serial.print(msgElapsedTime);Serial.print(" ");


      firstComma = secondComma;
      secondComma = nmeaStr.indexOf(',',firstComma+1);
      pString = nmeaStr.substring(firstComma+1,secondComma);
      //Serial.print(pString);
      track = pString.toFloat();

      int thisComma = secondComma;
      int nextComma = nmeaStr.indexOf(',',thisComma+1); // Dump the T
      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      turnRate = pString.toFloat();

      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1); // Dump the A
      thisComma = nextComma;
      nextComma = nmeaStr.indexOf(',',thisComma+1);      
      pString = nmeaStr.substring(thisComma+1,nextComma);
      speed = pString.toFloat();
      //Serial.print(track);
      //Serial.print(" ");
      //Serial.print(turnRate);

      //Serial.println("");
    }
    else if (nmeaStr.substring(1,6) == String("INPOS")) {
      //Serial.println("POS MESSAGE:");
      int firstComma = nmeaStr.indexOf(',');
      int secondComma = nmeaStr.indexOf(',',firstComma+1);
      String pString = nmeaStr.substring(firstComma+1,secondComma);
      float gpsTime = pString.toFloat();
      //Serial.print(gpsTime);Serial.print(" ");


      firstComma = secondComma;
      secondComma = nmeaStr.indexOf(',',firstComma+1);
      pString = nmeaStr.substring(firstComma+1,secondComma);
      //Serial.print(pString);
      gpsLat = pString.toFloat();
      //Serial.print(gpsLat);Serial.print(" ");

      int thisComma = secondComma;
      int nextComma = nmeaStr.indexOf(',',thisComma+1); // Dump the T     
      pString = nmeaStr.substring(thisComma+1,nextComma);
      gpsLon = pString.toFloat();
      //Serial.print(gpsLon);Serial.print(" ");
      //Serial.println(" ");

      gpsRecieved = true;

    }

  } // END NMEA READY



  // XTE RTD Calculations
  guidanceUpdate = false;
  //Serial.println(guidanceInit);
  if((gpsRecieved && guidanceInit) ){
    here.lat = gpsLat;
    here.lon = gpsLon;
    vectorTo = vector_to(here,waypointDB[wayPointNum]);
    vectorWP = vector_to(waypointDB[wayPointNum+1],waypointDB[wayPointNum]);
    vectorFrom = vector_to(waypointDB[wayPointNum+1],here);

    // Calculate Cross Track Error
    // Point One - From Waypoint
    // Point two - To Waypoint
    // Point three - current position

    float baseLat = waypointDB[wayPointNum].lat;
    float baseLon = waypointDB[wayPointNum].lon;
    float metersPerDeg = 60.0*1852.0;
    float lonAdj = cos(radians(baseLat));

    float x2=0;
    float y2=0;
    float x1 = (waypointDB[wayPointNum+1].lon - baseLon) * metersPerDeg * lonAdj ;
    float y1 = (waypointDB[wayPointNum+1].lat - baseLat) * metersPerDeg;
    float x0 = (here.lon - baseLon) * metersPerDeg * lonAdj;
    float y0 = (here.lat - baseLat) * metersPerDeg;

    float denom = sqrt( (y2-y1)*(y2-y1) + (x2-x1)*(x2-x1) );

    if (denom > 0 ) {
      xte = ( (y2-y1)*x0 - (x2-x1)*y0 - x1*y2) / denom;
      atd = ((x0-x1)*(x2-x1) + (y0-y1)*(y2-y1)) / denom;
      rtd = ((x2-x0)*(x2-x1) + (y2-y0)*(y2-y1)) / denom;
    }
    else
    {
      xte = 0;
      atd = 0;
      rtd = 0;
    }

    vectorXTE.distance = abs(xte);
    if(xte >= 0) {
      vectorXTE.bearing = normalize_angle360(vectorWP.bearing - 90.0);
      interceptBearing = normalize_angle360(vectorXTE.bearing + xteInterceptAngle);
    }
    else
    {
      vectorXTE.bearing = normalize_angle360(vectorWP.bearing + 90.0);
      interceptBearing = normalize_angle360(vectorXTE.bearing - xteInterceptAngle);
    }

    if (false  && wayPointNum >= 0) {
      Serial.print("XTE LEGS ");
      Serial.print(wayPointNum);
      Serial.println(" ");
      Serial.print(" LEG FROM->TO: ");
      Serial.print(vectorWP.bearing); Serial.print(" ");
      Serial.print(vectorWP.distance); Serial.print(" ");
      Serial.println(" ");
      Serial.print(" LEG FROM->HERE: ");
      Serial.print(vectorFrom.bearing); Serial.print(" ");
      Serial.print(vectorFrom.distance); Serial.print(" "); 
      Serial.println(" ");
      Serial.print(" LEG HERE->TO: ");
      Serial.print(vectorTo.bearing); Serial.print(" ");
      Serial.print(vectorTo.distance); 
      Serial.println(" "); 
      Serial.print("XTE Calc [");
      Serial.print(wayPointNum); Serial.print("] ");
      Serial.print(xte); Serial.print(" ");
      Serial.print(atd); Serial.print(" ");
      Serial.print(rtd); Serial.print(" ");
      Serial.println(" ");
    }
    
     
    guidanceUpdate = true;
  } 



  ////////////////////////
  // MODE STATE MACHINE
  if(buttonLongPress && mode != MODE_SEL) {
    targetMode = mode;
    mode=MODE_SEL;
  }

  //Serial.print(modes[mode]);
  //Serial.print(":");
  for(int m=2;m<8;m++) {
        sprintf(display_text[m],"    ");
  }
  switch(mode) {

    case MODE_SEL:
      motorOn = false;
      motor_stop();
      if(portEncoderPosition < prevEncoderPosition){
        targetMode = (ModeState)( (int)targetMode+1 );
        if(targetMode >= nModes) { targetMode = STANDBY; }
      }
      if(portEncoderPosition > prevEncoderPosition){
        targetMode = (ModeState)( (int)targetMode-1 );
        if(targetMode < STANDBY) { targetMode = CONTROL; }
      }
      // Format Display
      display_menu(nModes-1, targetMode-1, &(modes[1]));
      //Serial.print(modes[targetMode]);
      // for(int m=1;m<nModes;m++) {
      //   if(targetMode == m) {
      //     sprintf(display_text[m+1],"==> %s",modes[m]);
      //   }
      //   else {
      //     sprintf(display_text[m+1],"    %s",modes[m]);
      //   }        
      // }

      if(buttonShortPress) {
        mode=targetMode;
        if (mode == CONTROL) { controlTypeMode = SELECT_TYPE; }
      } 
    break;

    case STANDBY:
      // CCW +1
      motorOn = false;
      motor_stop();

      if(portEncoderPosition > prevEncoderPosition) {
        //Serial.print(portEncoderPosition); Serial.print(" "); Serial.print(prevEncoderPosition); Serial.println(" ");
        hdgTarget -= 10*(portEncoderPosition - prevEncoderPosition);
        if(hdgTarget > 180){
          hdgTarget -= 360;
        }
      }
      // CW -1
      if(portEncoderPosition < prevEncoderPosition) {
        //Serial.print(portEncoderPosition); Serial.print(" "); Serial.print(prevEncoderPosition); Serial.println(" ");
        hdgTarget -= 10*(portEncoderPosition - prevEncoderPosition);
        if(hdgTarget <= -180) {
          hdgTarget += 360;
        }
      }
      hdgTarget360 = hdgTarget;
      if(hdgTarget360 <0) { 
        hdgTarget360 += 360;
      }

      hdg360 = track;
      if(hdg360 <0) { 
        hdg360 += 360;
      }

      setHeading = hdgTarget360;
      headingDiff = 0;
      steerAdj = 0;
      headingPID.Reset();
      xtePID.Reset();
      xtePID.SetOutputSum(0.0);

      //controlTypeModes
      sprintf(display_text[2]," STANDBY - %s",controlTypeModes[controlTypeMode]);
      sprintf(display_text[3],"    %03d      %+03d  ",(int)hdgTarget360,(int)turnRateTarget);
      sprintf(display_text[4],"  - %03d -  - %+03d -",(int)hdg360,(int)turnRate);

      if(buttonShortPress) {
        mode=AUTO;
        waypointDB[wayPointNum+1].lat = gpsLat;
        waypointDB[wayPointNum+1].lon = gpsLon;
        evtMessage += "WPTADD,";
        evtMessage += String(wayPointNum+1);
        evtMessage += ",";
        evtMessage += String(waypointDB[wayPointNum+1].lat,8);
        evtMessage += ",";
        evtMessage += String(waypointDB[wayPointNum+1].lon,8);
        evtMessage += ":";
        sendEvtMessage = true;
      }
      // Set next waypoint to here for xte calculations
      waypointDB[wayPointNum+1].lat = gpsLat;
      waypointDB[wayPointNum+1].lon = gpsLon;
      
      sprintf(display_text[6],"W:%d B:%03d D:%d X:%d",wayPointNum,int(vectorTo.bearing),int(vectorTo.distance),int(xte));
      
    break;

    case AUTO:
      

      switch(controlTypeMode) {

        ////////////// HEADING CONTROL /////////////////
        case HEADING:
          // Update Target 
          if(portEncoderPosition > prevEncoderPosition) {
            //Serial.print(portEncoderPosition); Serial.print(" "); Serial.print(prevEncoderPosition); Serial.println(" ");
            hdgTarget -= 10*(portEncoderPosition - prevEncoderPosition);
            if(hdgTarget > 180){
              hdgTarget -= 360;
            }
          }
          // CW -1
          if(portEncoderPosition < prevEncoderPosition) {
            //Serial.print(portEncoderPosition); Serial.print(" "); Serial.print(prevEncoderPosition); Serial.println(" ");
            hdgTarget -= 10*(portEncoderPosition - prevEncoderPosition);
            if(hdgTarget <= -180) {
              hdgTarget += 360;
            }
          }

          hdgTarget360 = hdgTarget;
          if(hdgTarget360 <0) { 
            hdgTarget360 += 360;
          }

          hdg360 = track;
          if(hdg360 <0) { 
            hdg360 += 360;
          }

          setHeading = hdgTarget360;

        break;

        ////////////// BEARING CONTROL /////////////////
        case BEARING:
          // REMOVE WAYPOINT IF CLOSE ENOUGH
          if(vectorTo.distance < nextWayPointDist  ) {
            if(wayPointNum >0 ) {
              // Remove WPT from List
              evtMessage += "WPTREM,";
              evtMessage += String(wayPointNum);
              evtMessage += ":";
              wayPointNum--;
              // Recalc Bearing and Distance to new Waypoint
              here.lat = gpsLat;
              here.lon = gpsLon;
              vectorTo = vector_to(here,waypointDB[wayPointNum]);

              sendEvtMessage = true;

            }
          }

          setHeading = vectorTo.bearing;
          hdgTarget = setHeading;
          sprintf(tempChars,"Bearing Here -> WPT %d: %f, %f",wayPointNum,vectorTo.bearing,vectorTo.distance);
          Serial.println(tempChars);

        break;

        ////////////// BEARING CONTROL /////////////////
        case XTE:

          // REMOVE WAYPOINT IF CLOSE ENOUGH
          if(vectorTo.distance < nextWayPointDist  ) {
            if(wayPointNum >0 ) {
              // Remove WPT from List
              evtMessage += "WPTREM,";
              evtMessage += String(wayPointNum);
              evtMessage += ":";
              wayPointNum--;
              // Recalc Bearing and Distance to new Waypoint
              here.lat = gpsLat;
              here.lon = gpsLon;
              vectorTo = vector_to(here,waypointDB[wayPointNum]);

              sendEvtMessage = true;

            }
          }

          setHeading = vectorTo.bearing;
            if (rtd > 0) {  // before WP
              
              if (abs(xte) > xteControlThreshold) {  // outside control band
                setHeading = interceptBearing;
                xtePID.Reset();
              }
              else { // in contrl band
                xteOffset = 1.0 * xteSteeringCmd;
                setHeading = normalize_angle360(vectorTo.bearing + xteOffset);
              }
            }
            else {
              xtePID.Reset();
            }
            // if( abs(arrivalAngle) < 45.0 ) {
            //   xteHeadingAdjust = 5.0 * xte;
            //   if( abs(xteHeadingAdjust) < abs(xteAngle) ) {
            //     newSetHeading = vectorTo.bearing - xteHeadingAdjust;            
            //   }
            //   else
            //   {
            //     newSetHeading = xteBearing;
            //   }
            //   setHeading = normalize_angle360(newSetHeading);

            // }

            hdgTarget = setHeading;
            
            //Serial.println(" ");
            Serial.print("XTE CONTROL XTE:");
            Serial.print(xte); Serial.print(" SH:");
            Serial.print(setHeading); Serial.print(" C:");
            Serial.print(xteSteeringCmd); Serial.print(" ");
            Serial.println("");



        break;

        default:
          Serial.println("INVALID CONTROL TYPE");
      }

    headingDiff = normalize_angle(setHeading - track);
    if(abs(headingDiff) > 15.0) {
      headingPID.SetOutputSum(0.0);
    }
    headingPID.Compute();

    if(abs(xte) > xteControlThreshold) {
      xtePID.Reset();
      xtePID.SetOutputSum(0.0);
    }
    xtePID.Compute();

    rudderTarget = steeringCmd;

    //  if(portEncoderPosition > prevEncoderPosition) {
    //     //Serial.print(portEncoderPosition); Serial.print(" "); Serial.print(prevEncoderPosition); Serial.println(" ");
    //     hdgTarget -= 10*(portEncoderPosition - prevEncoderPosition);
    //     if(hdgTarget > 180){
    //       hdgTarget -= 360;
    //     }
    //   }
    //   // CW -1
    //   if(portEncoderPosition < prevEncoderPosition) {
    //     //Serial.print(portEncoderPosition); Serial.print(" "); Serial.print(prevEncoderPosition); Serial.println(" ");
    //     hdgTarget -= 10*(portEncoderPosition - prevEncoderPosition);
    //     if(hdgTarget <= -180) {
    //       hdgTarget += 360;
    //     }
    //   }

    //   hdgError = track - hdgTarget;
    //   if (hdgError > 180 ) {
    //     hdgError -= 360;
    //   }
    //   else if (hdgError <= -180) {
    //     hdgError += 360;
    //   }

    //   turnRateTarget = hdgError*trHdgCoef;
    //   if (turnRateTarget > trMax) {
    //     turnRateTarget = trMax;
    //   }
    //   if (turnRateTarget < -1*trMax) {
    //     turnRateTarget = -1.0*trMax;
    //   }

    //   trError = turnRate - turnRateTarget;

    //   //if(abs(hdgError) > 10.0) {
    //   //  turnRatePID.SetOutputSum(0.0);
    //   //}
    //   turnRatePID.Compute();
    //   //Serial.print(hdgError); Serial.print(" "); Serial.print(trError); Serial.print(" "); Serial.println(turnCtrlOutput);
    //   //turnCtrlOutput = turnPID.generate(trErr)  /** need to add library and check **/

    //   if ( abs(trError) > trDeadZone && abs(hdgError) > hdgDeadZone ) {
    //     motorDriveTime = elapsedTime + abs(turnCtrlOutput);
    //     if(turnCtrlOutput > 0) {
    //       motorFwd = true;
    //     }
    //     else
    //     {
    //       motorFwd = false;
    //     }
    //   }

    //   if (motorDriveTime > elapsedTime) {
    //     if (motorFwd) {
    //       motor_forward();
    //     }
    //     else
    //     {
    //       motor_reverse();
    //     }
    //     motorOn = true;
    //   }
    //   else
    //   {
    //     motor_stop();
    //     motorOn = false;
    //   }


      hdgTarget360 = hdgTarget;
      if(hdgTarget360 <0) { 
        hdgTarget360 += 360;
      }

      hdg360 = track;
      if(hdg360 <0) { 
        hdg360 += 360;
      }

      // Motor Handler
      rudderError = rudderTarget - rudderAngle;
      if( abs(rudderError) > rudderCtrlBand && abs(rudderAngle) < 50 )
      {
        //Serial.print(elapsedTime);
        //Serial.print(" ");
        //Serial.println(motorDriveTime);
        if(rudderError <= 0) { 
          motor_forward();
          //sprintf(display_text[2],"--->Forward");
          motorOn = true;
          motorFwd = true;
        }
        else { 
          motor_reverse();
          motorOn = true;
          motorFwd = false;
          //sprintf(display_text[2],"    Reverse<---");
        }
      }
      else
      {
        motorOn = false;
        motor_stop();
        sprintf(display_text[2]," ");
      }

      //sprintf(display_text[2],"        AUTO");
      sprintf(display_text[2],"   AUTO - %s",controlTypeModes[controlTypeMode]);
      sprintf(display_text[3],"    %03d      %+03d  ",(int)hdgTarget360,(int)turnRateTarget);
      if(motorOn && motorFwd) {
        sprintf(display_text[4]," => %03d      %+03d  ",(int)hdg360,(int)turnRate);
      }
      else if (motorOn) {
        sprintf(display_text[4],"    %03d <=   %+03d  ",(int)hdg360,(int)turnRate);
      }
      else
      {
        sprintf(display_text[4],"    %03d      %+03d  ",(int)hdg360,(int)turnRate);
      }

      sprintf(display_text[6],"W:%d B:%03d D:%d X:%d",wayPointNum,int(vectorTo.bearing),int(vectorTo.distance),int(xte));
      

      if(buttonShortPress) {
        mode=STANDBY;
      }


    break;

    case MANUAL:
      if(portEncoderPosition > prevEncoderPosition) {
        rudderTarget += 1;
      }
      if(portEncoderPosition < prevEncoderPosition) {
        rudderTarget -= 1;
      }
      if(buttonShortPress) {
        motorDriveTime = elapsedTime;
        motorOn = false;
        rudderTarget = 0;
      }

      // Motor Handler

      rudderError = rudderTarget - rudderAngle;
      if( abs(rudderError) > rudderCtrlBand && abs(rudderAngle) < 45 )
      {
        //Serial.print(elapsedTime);
        //Serial.print(" ");
        //Serial.println(motorDriveTime);
        if(rudderError <= 0) { 
          motor_forward();
          sprintf(display_text[2],"--->Forward");
        }
        else { 
          motor_reverse();
          sprintf(display_text[2],"    Reverse<---");
        }
      }
      else
      {
        motorOn = false;
        motor_stop();
        sprintf(display_text[2]," ");
      }
      

      // if(portEncoderPosition > prevEncoderPosition) {
      //   motorDriveTime = elapsedTime + motorTimeManual;
      //   motorFwd = true;
      //   motorOn = true;
      //   //Serial1.println("FWD");
      // }
      // if(portEncoderPosition < prevEncoderPosition) {
      //   motorDriveTime = elapsedTime + motorTimeManual;
      //   motorFwd = false;
      //   motorOn = true;
      //   //Serial1.println("REV");
      // }
      // if(buttonShortPress) {
      //   motorDriveTime = elapsedTime;
      //   motorOn = false;
      // }

      // // Motor Handler
      // if(motorOn && elapsedTime < motorDriveTime)
      // {
      //   //Serial.print(elapsedTime);
      //   //Serial.print(" ");
      //   //Serial.println(motorDriveTime);
      //   if(motorFwd) { 
      //     motor_forward();
      //     sprintf(display_text[2],"--->Forward");
      //   }
      //   else { 
      //     motor_reverse();
      //     sprintf(display_text[2],"    Reverse<---");
      //   }
      // }
      // else
      // {
      //   motorOn = false;
      //   motor_stop();
      //   sprintf(display_text[2]," ");
      // }
      


    break;

    case COMM:
      motorOn = false;
      motor_stop();
      //hdmMessage.toCharArray(display_text[2],20);
      sprintf(display_text[2],"Track %05.1f",track);
      sprintf(display_text[3],"Turn  %5.1f",turnRate);
      sprintf(display_text[4],"Speed %5.1f",speed);
      sprintf(display_text[5],"Age   %05d",(int)((elapsedTime - hdmTime)*1000));

    break;

    case CONTROL:
      motorOn = false;
      motor_stop();

      switch(controlMode) {

        case MENU:
          // if(portEncoderPosition < prevEncoderPosition){
          //   controlTargetMode = (ControlModeState)( (int)controlTargetMode+1 );
          //   if(controlTargetMode >= nControlModes) { controlTargetMode = SET_HOME; }
          // }
          // if(portEncoderPosition > prevEncoderPosition){
          //   controlTargetMode = (ControlModeState)( (int)controlTargetMode-1 );
          //   // Note decrimenting enums creates and overflow from 0 to 255;
          //   if(controlTargetMode <= MENU) { controlTargetMode = ROUTE; }
          // }
          // //Serial.println(controlTargetMode);

          // // Format Display
          // display_menu(nControlModes-1, controlTargetMode-1, &(controlModes[1]));
          //int tmpMode = 0;
          tmpInt = do_menu( (int)nControlModes, (int)controlTargetMode, controlModes );
          controlTargetMode = (ControlModeState)tmpInt;
          
          if(buttonShortPress) {        
            controlMode=controlTargetMode;
            Serial.print("CTRL MODE -> ");
            Serial.println(controlMode);
            //controlMode=MENU;
          }
        break; 

        case SET_HOME:
          //Serial.print("Setting Home");
          sprintf(display_text[2],"Home Set");
          sprintf(display_text[3],"Lat:  %08.8f",gpsLat);
          sprintf(display_text[4],"Lon: %08.8f",gpsLon);

          if(buttonShortPress) {
            wayPointNum=0;
            Serial.print("LATCH WPT ");
            Serial.println(wayPointNum);
            waypointDB[wayPointNum].lat = gpsLat;
            waypointDB[wayPointNum].lon = gpsLon;
            evtMessage += "WPTADD,";
            evtMessage += String(wayPointNum);
            evtMessage += ",";
            evtMessage += String(waypointDB[wayPointNum].lat,8);
            evtMessage += ",";
            evtMessage += String(waypointDB[wayPointNum].lon,8);
            evtMessage += ":";
            sendEvtMessage = true;
            waypointAdded = true;
            //Serial.print("Setting Home: ");
            //Serial.print(display_text[3]); Serial.print(" ");
            //Serial.print(display_text[4]); Serial.print(" ");
            //Serial.println("");
            Serial.println(evtMessage);
            controlMode=MENU;
            guidanceInit = true;
          }
        break;

        case TYPE:
          tmpInt = do_menu( (int)nControlTypeModes, (int)controlTypeTargetMode, controlTypeModes );
          controlTypeTargetMode = (ControlTypeState)tmpInt;
          
          if(buttonShortPress) {        
            controlTypeMode=controlTypeTargetMode;
            //controlType=
            Serial.print("CTRL Type -> ");
            Serial.println(controlTypeMode);
            controlMode=MENU;
          }
        break;


        case ROUTE:
          tmpInt = do_menu( (int)nControlRouteModes, (int)controlRouteTargetMode, controlRouteModes );
          controlRouteTargetMode = (ControlRouteState)tmpInt;
          
          if(buttonShortPress) {        
            controlRouteMode=controlRouteTargetMode;
            //controlType=
            Serial.print("ROUTE Type -> ");
            Serial.println(controlRouteMode);

            switch(controlRouteMode) {
              case(FREE):
                Serial.println("ROUTE = FREE");
                wayPointNum++;
                Serial.print("LATCH WPT ");
                Serial.println(wayPointNum);
                waypointDB[wayPointNum].lat = gpsLat+1.0/60.0;
                waypointDB[wayPointNum].lon = gpsLon;
                waypointDB[wayPointNum+1].lat = gpsLat;
                waypointDB[wayPointNum+1].lon = gpsLon;
                evtMessage += "WPTADD,";
                evtMessage += String(wayPointNum);
                evtMessage += ",";
                evtMessage += String(waypointDB[wayPointNum].lat,8);
                evtMessage += ",";
                evtMessage += String(waypointDB[wayPointNum].lon,8);
                evtMessage += ":";
                sendEvtMessage = true;
                waypointAdded = true;
              break;

              case(BOX):
                Serial.println("ADDING BOX WAYPOINTS");
                routeSel = 1;
                for (int p = 1; p < RouteDB[routeSel]->nWaypoints+1; p++ ) {
                  wayPointNum++;
                  float metersPerDeg = 60.0*1852.0;
                  float lonAdj = cos(radians(gpsLat));
                  waypointDB[p].lat = gpsLat + RouteDB[routeSel]->wpts[p-1].lat/metersPerDeg;
                  waypointDB[p].lon = gpsLon + RouteDB[routeSel]->wpts[p-1].lon/(metersPerDeg * lonAdj);
                }
                waypointDB[wayPointNum+1].lat = gpsLat;
                waypointDB[wayPointNum+1].lon = gpsLon;
                waypointAdded = true;
              break;

              case(OTTO):
                Serial.println("ADDING OTTO WAYPOINTS");
                routeSel = 2;
                for (int p = 1; p < RouteDB[routeSel]->nWaypoints+1; p++ ) {
                  wayPointNum++;
                  float metersPerDeg = 60.0*1852.0;
                  float lonAdj = cos(radians(gpsLat));
                  waypointDB[p].lat = gpsLat + RouteDB[routeSel]->wpts[p-1].lat/metersPerDeg;
                  waypointDB[p].lon = gpsLon + RouteDB[routeSel]->wpts[p-1].lon/(metersPerDeg * lonAdj);
                }
                waypointDB[wayPointNum+1].lat = gpsLat;
                waypointDB[wayPointNum+1].lon = gpsLon;
                waypointAdded = true;
              break;

              default:
                controlRouteMode = FREE;

            }
            controlMode=MENU;
          }
        break;


        default:
          Serial.print("ERRONEOUS CONTROL MODE ");
          Serial.print(controlMode);
          controlMode=MENU;

      }
    break;

    default:
      Serial.print("ERRONEOUS MODE ");
      Serial.print(mode);
      delay(5000);
  }
  //Serial.println("");


  if(waypointAdded) {
    Serial.println("WPT DB");
    for(int w=0; w<= wayPointNum+1; w++) {

      Serial.print(w);
      Serial.print(" ");
      Serial.print(waypointDB[w].lat,8);
      Serial.print(" ");
      Serial.print(waypointDB[w].lon,8);
      if(w > 0){
        GuidanceVector t=vector_to(waypointDB[w-1],waypointDB[w]);
        Serial.print(" ");
        Serial.print(t.bearing,3);
        Serial.print(" ");
        Serial.print(t.distance,3);
      }         
      Serial.println("");
    }
  }


  // Paint Display
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  // Update Contents
  // Line 2
  tempStr = "t= ";
  tempStr += String(elapsedTime,1);
  tempStr += " dt= ";
  tempStr += String(int(dt*1000));
  tempStr.toCharArray(display_text[1],LINE_SIZE);

  
  tempStr = "    ";
  tempStr += String(int(rudderAngle));
  tempStr += " tgt= ";
  tempStr += String(int(rudderTarget));
  tempStr.toCharArray(display_text[7],LINE_SIZE);
  // // Line 3 Mode
  // tempStr = "--Line 3:";
  // tempStr.toCharArray(display_text[2],LINE_SIZE);

  // // Line 4 RC Inputs
  // tempStr = "---Line 4:";
  // tempStr.toCharArray(display_text[3],LINE_SIZE);
  
  // // Line 5 Guidance
  // sprintf(display_text[4],"--G W:%d B:%03d D:%d X:%d",1,2,3,4);

  // // Line 6 Control
  // sprintf(display_text[5],"S:%03d H:%03d D:%+03d",5,6,7);

  // // Line 7 Actuators
  // sprintf(display_text[6],"A:%03d %03d %03d %03d",8,9,10,11);

  // // Line 8 Status
  // tempStr = "Line 8:";
  // tempStr.toCharArray(display_text[7],LINE_SIZE);

  // Write out lines and display
  for(int line=0;line<NLINES;line++) {
      display.println(display_text[line]);
  }
  // send the display
  display.display();

  
  ////////////////// SERIAL OUTPUT /////////////////////////
  if(sendSerial.fire()) {
    

    // CRACT Message
    tempStr = "$CRACT,";
    tempStr += String(elapsedTime,2);
    tempStr += ",";
    tempStr += String(rudderTarget);
    tempStr += ",";
    tempStr += String(rudderAngle);
    tempStr += ",";
    tempStr += String(0.0);
    tempStr += ",";
    tempStr += String(0.0);
    // ** Do this later
    //nBytes = tempStr.length()+1;
    //tempStr.toCharArray(txSendBuffer,nBytes);
    //txSendBuffer[nBytes] = 0;
    //GPS.addChecksum(txSendBuffer);
    //tempStr = txSendBuffer;
    // ** in place of this
    //tempStr += "*FF";
    //tempStr += "\n";
    sentCS = nmea_checksum(tempStr.c_str());
    sprintf(tempChars,"%02X",(int)sentCS);
    tempStr += "*";
    tempStr += tempChars;
    tempStr += "\n";
    //Serial.print("       TX:");
    if (ECHO_NMEA) { Serial.print(tempStr); }
    Serial1.print(tempStr);

    int autoMode=0;
    if (mode == 2) { autoMode = 1; }
    // CRCTL Message
    tempStr = "$CRCTL,";
    tempStr += String(elapsedTime,2);
    tempStr += ",";
    tempStr += String(autoMode);
    tempStr += ",";
    tempStr += String(track);
    tempStr += ",";
    tempStr += String(setHeading);
    tempStr += ",";
    tempStr += String(headingDiff);
    tempStr += ",";
    tempStr += String(steeringCmd);
    tempStr += ",";
    if(controlTypeMode != XTE ) {
      tempStr += String(headingPID.GetKp());
      tempStr += ",";
      tempStr += String(headingPID.GetKi());
    }
    else
    {
      tempStr += String(xtePID.GetKp());
      tempStr += ",";
      tempStr += String(xtePID.GetKi());
    }

    // ** Do this later
    //nBytes = tempStr.length()+1;
    //tempStr.toCharArray(txSendBuffer,nBytes);
    //txSendBuffer[nBytes] = 0;
    //GPS.addChecksum(txSendBuffer);
    //tempStr = txSendBuffer;
    // ** in place of this
    sentCS = nmea_checksum(tempStr.c_str());
    sprintf(tempChars,"%02X",(int)sentCS);
    tempStr += "*";
    tempStr += tempChars;
    tempStr += "\n";
    //Serial.print("       TX:");
    if (ECHO_NMEA) { Serial.print(tempStr); }
    Serial1.print(tempStr);

    // CRGUI Message
    if(true) {
      int msgMode;
      msgMode = int(controlTypeMode)-1;
      if(msgMode <0) { msgMode = 0; }
      // if (navSource == 1 && guidanceMode == 0) {
      //   msgMode = 1;
      // }
      // else if (navSource == 1 && guidanceMode == 1) {
      //   msgMode = 2;
      // }
      tempStr = "$CRGUI,";
      tempStr += String(elapsedTime,2);
      tempStr += ",";
      tempStr += String(msgMode);
      tempStr += ",";
      tempStr += String(wayPointNum);
      tempStr += ",";
      if(wayPointNum < 0) {
        tempStr += "0.0";
        tempStr += ",";
        tempStr += "0.0";
        tempStr += ",";
        tempStr += "0.0";
        tempStr += ",";
        tempStr += "0.0";
        tempStr += ",";
        tempStr += "0.0";
        tempStr += ",";
        tempStr += "0.0";

      }
      else
      {
        tempStr += String(waypointDB[wayPointNum].lat,8);
        tempStr += ",";
        tempStr += String(waypointDB[wayPointNum].lon,8);
        tempStr += ",";
        tempStr += String(vectorTo.bearing);
        tempStr += ",";
        tempStr += String(vectorTo.distance);
        tempStr += ",";
        tempStr += String(xte);
        tempStr += ",";
        tempStr += String(rtd);
      }

      // ** Do this later
      //nBytes = tempStr.length()+1;
      //tempStr.toCharArray(txSendBuffer,nBytes);
      //txSendBuffer[nBytes] = 0;
      //GPS.addChecksum(txSendBuffer);
      //tempStr = txSendBuffer;
      // ** in place of this
      sentCS = nmea_checksum(tempStr.c_str());
      sprintf(tempChars,"%02X",(int)sentCS);
      tempStr += "*";
      tempStr += tempChars;
      tempStr += "\n";
      //Serial.print("       TX:");
      if (false || ECHO_NMEA) { Serial.print(tempStr); }
      Serial1.print(tempStr);
    }

    if(sendEvtMessage && true) {
      tempStr = "$CREVT,";
      tempStr += String(elapsedTime,2);
      tempStr += ",";
      tempStr += evtMessage;
      sentCS = nmea_checksum(tempStr.c_str());
      sprintf(tempChars,"%02X",(int)sentCS);
      tempStr += "*";
      tempStr += tempChars;
      tempStr += "\n";
      if (ECHO_NMEA || true) { Serial.print(tempStr); }
      Serial1.print(tempStr);
      sendEvtMessage = false;
      evtMessage = "";
    }

  }




  // Save previous states
  prevElapsedTime = elapsedTime;
  prevEncoderPosition = portEncoderPosition;

  //delay(100);
}
