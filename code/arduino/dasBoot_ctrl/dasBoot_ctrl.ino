/*
** DasBoot Controller
**
*/

// COMPILE SWITCHES
#define WAIT_FOR_SERIAL false

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


////////////////////////////////////////////////////////////////////////
// TIMER PROCESSES INIT
////////////////////////////////////////////////////////////////////////
float fastProcessFreq = 1000.0;
float slowProcessFreq = 10.0;
FireTimer sendSerial;
#define sendSerialPeriod 200
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
enum ModeState { MODE_SEL, STANDBY, AUTO, MANUAL, COMM, CONTROL };
ModeState mode = STANDBY;
ModeState targetMode = STANDBY;

const char* modes[] = {"MODE_SEL", "STANDBY", "AUTO", "MANUAL", "COMM", "CONTROL"};
int nModes = 6;



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
int rudderRaw = 0;
float rudderAngle = 0;
float rudderCenter = 2048.0;
float rudderScale = 180.0/2750.0;


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
float hdgKi = 0.0;
float hdgKd = 0.00;

float ctrlOutput = 0.0;
float turnCtrlOutput = 0.0;

float turnKp = 0.22;
float turnKi = 0.0;
float turnKd = 0.0;

float zeroDiff = 0;

QuickPID headingPID(&hdgError, &ctrlOutput, &zeroDiff);
QuickPID turnRatePID(&trError, &turnCtrlOutput, &zeroDiff);




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

  Serial1.begin(57600);



  // Setup time based processes
  setup_timer(1, true, fastProcessFreq, fastProcess) ;
  setup_timer(2, true, slowProcessFreq, slowProcess) ;


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

  // Initialize Controllers
  headingPID.SetTunings(hdgKp, hdgKi, hdgKd);
  headingPID.SetMode(QuickPID::Control::timer);
  headingPID.SetSampleTimeUs(50*1000);
  headingPID.SetOutputLimits(-25.0, 25.0);
  headingPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  Serial.println(headingPID.GetMode());

  turnRatePID.SetTunings(turnKp, turnKi, turnKd);
  turnRatePID.SetMode(QuickPID::Control::timer);
  turnRatePID.SetSampleTimeUs(50*1000);
  turnRatePID.SetOutputLimits(-25.0, 25.0);
  turnRatePID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  Serial.println(turnRatePID.GetMode());


  //Serial.println(modes[mode]);

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

char nmeaBuffer[64];
int nmeaBufferIndex = 0;
float ch_width[4];
float prevChWidth[4];
float prevSelectWidth;
bool navRecieved = false;
bool gpsRecieved = false;
float gpsLat = 0.0;
float gpsLon = 0.0;
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


void loop() {
  
  char nmeaSentence[40];
  bool nmeaReady,nmeaStart; 
  char tempChars[40];
  

  // Time Update
    // Collect Measurements
  elapsedTime = (float)millis()/1000.0;
  dt = elapsedTime - prevElapsedTime;
  //Serial.println(elapsedTime);
  //delay(100);

  rudderRaw = analogRead(RUDDER_PIN);
  rudderAngle = ((int)rudderRaw - rudderCenter) * rudderScale;
  Serial.print(rudderRaw); Serial.print(" "); Serial.println(rudderAngle); 


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
  while (int nChar = Serial1.available() > 0) {
    int recievedByte = Serial1.read();
    char incomingByte = (char)recievedByte;
    //Serial.print("Received on Serial1: ");
    //Serial.print("Char[");
    //Serial.print(incomingByte);
    //Serial.println("]");

    if(incomingByte == '$') {
      //Serial.print("-start-");
      nmeaStart = true;
      nmeaStr = String("");
      nmeaBufferIndex = 0;     
    }
    if(incomingByte == 10) 
    {
      //Serial.print("-END-");
      nmeaReady = true;
      nmeaStart = false;
      nmeaBuffer[nmeaBufferIndex] = 0;
      break;
    }
    if(nmeaStart == true) {
      nmeaBuffer[nmeaBufferIndex] = incomingByte;
      nmeaBufferIndex++;
      nmeaStr += incomingByte;
      //Serial.print("+");
      //Serial.print(incomingByte);
      //Serial.println("]");
      for(int i = 0; i<10000000; i ++) {}
    }    
  }

  //Process NMEA Messages
  if(nmeaReady)
  {
    //Serial.print("Resending:");
    //Serial.println(nmeaStr);
    Serial1.println(nmeaStr);
    //Serial.print("--->Parsing:");
    //Serial.println(nmeaStr);
    //Serial.println(nmeaStr.indexOf('*'));
    //    Serial.print("--->Buffer:");
    //Serial.println(nmeaBuffer);
    //Serial.println(nmeaStr.substring(1,6));
    if(nmeaStr.substring(1,6) == String("INATT")) {
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
      if(portEncoderPosition < prevEncoderPosition){
        targetMode = (ModeState)( (int)targetMode+1 );
        if(targetMode >= nModes) { targetMode = STANDBY; }
      }
      if(portEncoderPosition > prevEncoderPosition){
        targetMode = (ModeState)( (int)targetMode-1 );
        if(targetMode < STANDBY) { targetMode = CONTROL; }
      }
      // Format Display
      //Serial.print(modes[targetMode]);
      for(int m=1;m<nModes;m++) {
        if(targetMode == m) {
          sprintf(display_text[m+1],"==> %s",modes[m]);
        }
        else {
          sprintf(display_text[m+1],"    %s",modes[m]);
        }        
      }

      if(buttonShortPress) {
        mode=targetMode;
      } 
    break;

    case STANDBY:
      // CCW +1
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
      sprintf(display_text[2],"        STANDBY");
      sprintf(display_text[3],"    %03d      %+03d  ",(int)hdgTarget360,(int)turnRateTarget);
      sprintf(display_text[4],"  - %03d -  - %+03d -",(int)hdg360,(int)turnRate);

      if(buttonShortPress) {
        mode=AUTO;
      }
    break;

    case AUTO:
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

      hdgError = track - hdgTarget;
      if (hdgError > 180 ) {
        hdgError -= 360;
      }
      else if (hdgError <= -180) {
        hdgError += 360;
      }

      turnRateTarget = hdgError*trHdgCoef;
      if (turnRateTarget > trMax) {
        turnRateTarget = trMax;
      }
      if (turnRateTarget < -1*trMax) {
        turnRateTarget = -1.0*trMax;
      }

      trError = turnRate - turnRateTarget;

      //if(abs(hdgError) > 10.0) {
      //  turnRatePID.SetOutputSum(0.0);
      //}
      turnRatePID.Compute();
      //Serial.print(hdgError); Serial.print(" "); Serial.print(trError); Serial.print(" "); Serial.println(turnCtrlOutput);
      //turnCtrlOutput = turnPID.generate(trErr)  /** need to add library and check **/

      if ( abs(trError) > trDeadZone && abs(hdgError) > hdgDeadZone ) {
        motorDriveTime = elapsedTime + abs(turnCtrlOutput);
        if(turnCtrlOutput > 0) {
          motorFwd = true;
        }
        else
        {
          motorFwd = false;
        }
      }

      if (motorDriveTime > elapsedTime) {
        if (motorFwd) {
          motor_forward();
        }
        else
        {
          motor_reverse();
        }
        motorOn = true;
      }
      else
      {
        motor_stop();
        motorOn = false;
      }


      hdgTarget360 = hdgTarget;
      if(hdgTarget360 <0) { 
        hdgTarget360 += 360;
      }

      hdg360 = track;
      if(hdg360 <0) { 
        hdg360 += 360;
      }


      sprintf(display_text[2],"        AUTO");
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
      

      if(buttonShortPress) {
        mode=STANDBY;
      }


    break;

    case MANUAL:
      if(portEncoderPosition > prevEncoderPosition) {
        motorDriveTime = elapsedTime + motorTimeManual;
        motorFwd = true;
        motorOn = true;
        //Serial1.println("FWD");
      }
      if(portEncoderPosition < prevEncoderPosition) {
        motorDriveTime = elapsedTime + motorTimeManual;
        motorFwd = false;
        motorOn = true;
        //Serial1.println("REV");
      }
      if(buttonShortPress) {
        motorDriveTime = elapsedTime;
        motorOn = false;
      }

      // Motor Handler
      if(motorOn && elapsedTime < motorDriveTime)
      {
        //Serial.print(elapsedTime);
        //Serial.print(" ");
        //Serial.println(motorDriveTime);
        if(motorFwd) { 
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


    break;

    case COMM:
      //hdmMessage.toCharArray(display_text[2],20);
      sprintf(display_text[2],"Track %05.1f",track);
      sprintf(display_text[3],"Turn  %5.1f",turnRate);
      sprintf(display_text[4],"Speed %5.1f",speed);
      sprintf(display_text[5],"Age   %05d",(int)((elapsedTime - hdmTime)*1000));

    break;

    case CONTROL:
    break;

    default:
      Serial.print("ERRONEOUS MODE ");
      Serial.print(mode);
      delay(5000);
  }
  //Serial.println("");





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

  


  // Save previous states
  prevElapsedTime = elapsedTime;
  prevEncoderPosition = portEncoderPosition;

  //delay(100);
}
