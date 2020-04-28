// Config variables if you do not have hardware connected.
//#define servo
#define LCD
#define STEPPER
//#define LEDs
#define POTS
#define PRINT_LOST_TICKS
#define debugging

// include libraries
#ifdef servo
#include <Servo.h>
#endif

#ifdef LCD
#include <MCUFRIEND_kbv.h> //MCUfriend TFT Library 
MCUFRIEND_kbv tft;      
#include <TouchScreen.h> //Adafruit TFT library 
#endif

#ifdef STEPPER
#include <AccelStepper.h> //Accel Stepper Library
#endif 

#include "TimerOne.h"           // Timer component
                                //  By Jesse Tane, Jérôme Despatis, Michael Polli, Dan Clemens, Paul Stroffregen
                                //  https://playground.arduino.cc/Code/Timer1/

#include <MsTimer2.h>

// stepper variables
#define STEP 53
#define DIR 51

//Create stepper object
#ifdef STEPPER
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
#endif

//Touch screeen variables
const int XP = 6, XM = A2, YP = A1, YM = 7; //ID=0x9341
const int TS_LEFT = 178, TS_RT = 907, TS_TOP = 174, TS_BOT = 902;
uint16_t ID;

#ifdef LCD
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;
#endif

uint8_t Orientation = 0;    //PORTRAIT

#define BLACK   0x0000
#define WHITE   0xFFFF
#define MINPRESSURE 200
#define MAXPRESSURE 1000


// constants
const int red = 6;   // breathing in LED
const int green = 7; // breathing out LED

const int watchdog_pin = A11;
const int enabled_pin = 45;

#ifdef POTS
const int IE_pin = A13;  // Analog input pin that the potentiometer is attached to #BLUE
const int BPM_pin = A12; // #GREEN
const int volume_pin = A14; // #YELLOW
const int set_point_pin = A8;  // #WHITE
#endif

//Calibration variables
const uint16_t set_point_bottom = 850;
const uint16_t set_point_top = 600;
const uint16_t volume_top = 600;
const uint16_t steps_to_top = 300*8;

// ventilator parameter limits
const float BPM_MIN = 10.0f;
const float BPM_MAX = 25.0f; //Any higher and always below TI_MIN
const float TH_MIN = 0.0f;
const float TH_MAX = 2.0f;
const float IE_MIN = 1.0f;
const float IE_MAX = 2.5f;
const float VOL_MIN = 200.0f;
const float VOL_MAX = 600.0f;
const float TI_MIN = 1.0f;

//Ventilator state
const uint8_t IN = 0;
const uint8_t HOLD = 1;
const uint8_t OUT = 2;
const uint8_t DISABLED = 3;

// ventilator parameters
uint8_t bpm = 20;             // breaths per minute

float t_i;                // inspiration time in units of seconds
float t_h = 0.5;          // hold time in units of seconds
float t_e;                // expiration time in units of seconds
float t_b = 60.0 / bpm;   // seconds per breath
float IE = 1.0;           // inspiration/expiration ratio 1 to 5

float desired_IE = 1.0;   // desired inspiration/expiration ratio (IE*10) (read from POT)
uint8_t desired_bpm = 20; // desired breaths per minute (read from POT)
uint16_t volume = 30;        // tidal volume, amount of air per breath

// local variables
volatile uint16_t T;        // time in units of dT within cycle
uint16_t T_b;      // breath cycle time in ms
uint16_t T_i;      // inspiration time in ms
uint16_t T_h;      // hold time in ms
int          vel_i;    // inspiration motor velocity
int          vel_e;    // expiration motor velocity
uint16_t     accel_i;  //inspriation and expiration acceleration
uint16_t     accel_e;  
int          vel;      // current servo velocity
uint16_t num_steps;  //number of steps to move to achieve volume
bool stopMove = false;
bool e_inspiration = false; // inspiration error flag (inspiration too short)

volatile uint8_t state = 255;
uint8_t previous_state = 255;

volatile uint16_t lost_ticks = 0;

// watchdog variables
uint8_t watchdog_state = LOW;
uint8_t watchdog_ticks = 0;


char breath_summary[64];

char command[16];   // serial commands stashed here
char *bp = command; // pointer into command buffer

#ifdef servo
Servo myservo;
#endif      // servo object

/*
   Serial command prototol:
   PFFFF\n

   where P denotes the parameter to change:
     b breaths per minute
     r IE ratio
     v volume

   and FFF is a valid floating point number with the new value for the parameter.
*/

/**********************************************************************************
   Periodic loop for the ventilator task
 **********************************************************************************/

void update_breathing_parameters()
{
  // convert breath parameters into inspiration and expiration times
  t_b = 60.0 / desired_bpm;
  t_i = t_b / (1.0f + desired_IE) - t_h;  // inspiration time

  if (t_i < TI_MIN) {
    // check for inspiration time too short, happens for large IE and hold time
    e_inspiration = true;
    t_i = TI_MIN;
    Serial.print("trying to set inspiration time too small\n");
    return;  // don't update any control parameters TODO is this best way to handle this?
  }

  e_inspiration = false;

  bpm = desired_bpm;
  IE = desired_IE;
  
  t_e = IE * (t_i + t_h);         // expiration time

  // convert breath parameters into times expressed in units of ms millis();
  T_b = t_b * 1000.0;   // total breath cycle time
  T_h = t_h * 1000.0;   // hold time
  T_i = t_i * 1000.0;   // inspiration time

  //this does not take into account the acceleration profile of the stepper
  vel_i = volume/volume_top*steps_to_top/ t_i*1.2;  // velocity required to expel volume during inspiration
  vel_e = volume/volume_top*steps_to_top/ t_e*1.2; // velocity required to remove volume during expiration

  num_steps = volume/volume_top*steps_to_top;

  #ifdef debugging
  // create the string that is displayed to console on every breath
  char s1[16], s2[16], s3[16], s4[16];
  
  dtostrf(bpm, 6, 2, s1);
  dtostrf(IE, 6, 2, s2);
  dtostrf(t_h, 6, 2, s3);
  dtostrf(volume, 6, 2, s4);
  sprintf(breath_summary, "breath: bpm=%s, IE=%s, hold=%s, vol=%s\n", s1, s2, s3, s4);
  #endif
}

void check_serial(void){
    if (Serial.available()) {
    *bp = Serial.read(); // get next char from the serial port into buffer
    if (*bp == '\n') {
      float x;

      // process a complete message
      *bp = '\0';  // null terminate the string
      switch (command[0]) {       // parse the message
        case 'b':  // new BPM
          x = atof(command + 1);
          if (x >= BPM_MIN && x <= BPM_MAX)
            desired_bpm = x;
          break;
        case 'r':  // new IE ratio
          x = atof(command + 1);
          if (x >= IE_MIN && x <= IE_MAX)
            desired_IE = x;
          break;
        case 'v':  // new volume
          x = atof(command + 1);
          if (x >= VOL_MIN && x <= VOL_MAX)
            volume = x;
          break;
        case 'h':  // hold time
          x = atof(command + 1);
          if (x >= TH_MIN && x <= TH_MAX)
            t_h = x;
          break;
      }
      bp = command; // reset the buffer pointer
      update_breathing_parameters(); // convert to units used in control loop

      // convert the float parameters into units of dT

    } else
      bp++; // update pointer into command buffer
    //WARNING Potential buffer overflow here!
  }
}

#ifdef POTS
void check_inputs(void)
{
  int sensorValue = analogRead(IE_pin);
  desired_IE = (IE_MIN + (IE_MAX-IE_MIN) * sensorValue / 1023);
  sensorValue = analogRead(BPM_pin);
  desired_bpm = BPM_MIN + (BPM_MAX-BPM_MIN) * sensorValue / 1023;
  sensorValue = analogRead(volume_pin);
  volume = VOL_MIN+ (VOL_MAX-VOL_MIN) * sensorValue / 1023;
}

uint16_t read_set_point(){
  return analogRead(set_point_pin);
}
#endif

#ifdef LCD
void screen_update(void){
  char s1[16], s2[16];
  
  dtostrf(desired_IE, 3, 1, s1);
  dtostrf(IE, 3, 1, s2);
  
  tft.setCursor(96, 24);
  tft.print(desired_bpm);
  tft.setCursor(96, 48); 
  tft.print(s1);

  tft.setCursor(0, 72); 
  
  if (e_inspiration) {
    tft.print("INVALID VALUE");
  } else {
    tft.print("             ");
  }

  tft.setCursor(96, 124);
  tft.print(bpm);
  tft.setCursor(96, 148);
  tft.print(s2);
  tft.setCursor(96, 172);
  tft.print(volume);
    
}

void screen_in(void){
  tft.setCursor(0, 244);
  tft.print("IN      ");
}

void screen_out(void){
  tft.setCursor(0, 244);
  tft.print("OUT     ");
}

void screen_hold(void){
  tft.setCursor(0, 244);
  tft.print("HOLD    ");
}

void screen_disabled(void){
  tft.setCursor(0, 244);
  tft.print("DISABLED");
}

void screen_lost_ticks(void) {
  tft.setCursor(160, 244);
  tft.print('E');
  tft.setCursor(180, 244);
  tft.print(lost_ticks);
}

// Function to Init the LCD 
void init_lcd(void){
    //Setup LCD screen for debugging
    tft.reset();
    ID = tft.readID();
    tft.begin(ID);
    tft.setRotation(Orientation);

    //Write basic start message to screen
    tft.fillScreen(WHITE);
    tft.setTextSize(3);
    tft.setTextColor(BLACK, WHITE);

    //Write initial screen values
    char s1[16], s2[16];
      
    dtostrf(desired_IE, 3, 1, s1);
    dtostrf(IE, 3, 1, s2);
    
    tft.setCursor(0, 0);
    tft.print("-- Desired --");
    tft.setCursor(0, 24); 
    tft.print("BPM: ");
    tft.setCursor(96, 24);
    tft.print(desired_bpm);
    tft.setCursor(0, 48); 
    tft.print("IE:  ");
    tft.setCursor(96, 48);
    tft.print(s1);

    tft.setCursor(0, 72); 
    
    if (e_inspiration) {
      tft.print("INVALID VALUES");
    } else {
      tft.print("              ");
    }

    tft.setCursor(0, 100);
    tft.print("-- Actual --");
    tft.setCursor(0, 124);
    tft.print("BPM: ");
    tft.setCursor(96, 124);
    tft.print(bpm);
    tft.setCursor(0, 148);
    tft.print("IE:  ");
    tft.setCursor(96, 148);
    tft.print(s2);
    tft.setCursor(0, 172);
    tft.print("Vol: ");
    tft.setCursor(96, 172);
    tft.print(volume);

    screen_lost_ticks();
}
#endif


void servo_to_start(){
  #ifdef STEPPER
  uint16_t error = set_point_bottom-read_set_point();
  while (abs(error) > 10) {
     stepper.runToNewPosition(error > 10 ? -10 : 10);
     error = set_point_bottom-read_set_point();
     stepper.setCurrentPosition(0);
  }
  #endif
}

void loop() {
  // process messages from the serial port
    tft.fillScreen(WHITE);
    tft.setTextSize(3);
    tft.setTextColor(BLACK, WHITE);

  while(1){
    tft.setCursor(0, 0);
    tft.print((float)read_set_point());
  }

}

/**********************************************************************************
   Initializer the Arduino for ventilator task
 **********************************************************************************/
void setup() {

  // connect i/o devices
  #ifdef servo
  myservo.attach(9);  // attach servo to pin 9 PWM
  #endif

  #ifdef LEDs
  pinMode(red, OUTPUT);    // breathing in
  digitalWrite(red, LOW);

  pinMode(green, OUTPUT);  // breathing out
  digitalWrite(green, LOW);
  #endif

  pinMode(watchdog_pin, OUTPUT);
  digitalWrite(watchdog_pin, LOW);

  pinMode(enabled_pin, INPUT_PULLUP); 

  //Setup LCD screen for debugging
  #ifdef LCD
  init_lcd();
  #endif

  //Setup Stepper
  #ifdef STEPPER
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(48000);
  //stepper.moveTo(steps_to_top); //TODO test to just move it up
  #endif

  // connect serial port for debug and command
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial to connect
  }

  Serial.print("QUT Ventilator control - startup\n");
  
  #ifdef POTS
  check_inputs();
  #endif
  
  Timer1.initialize(200);  
  Timer1.attachInterrupt(Timer);

  MsTimer2::set(1, update);
  MsTimer2::start();

  Serial.print("QUT Ventilator control - ready\n");
}

void update() {
  uint8_t disabled = digitalRead(enabled_pin);


  // Send watchdog signal at 20hz
  if (++watchdog_ticks >= 50) {
    digitalWrite(watchdog_pin, watchdog_state);
    watchdog_state = HIGH - watchdog_state;
    watchdog_ticks = 0;
  }
}

void Timer() {
  #ifdef STEPPER
  if (!stopMove) stepper.run();
  #endif
}
