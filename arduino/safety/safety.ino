// Ventilator safety monitor system
//
//  2 inputs::
//     analog voltage from pot (pot_pin)
//     heartbeat input (hb_input_pin)
//
//  2 outouts::
//     loss of heartbeat transitions (hb_alarm_pin)
//     loss of analog voltage variation (pot_alarm_pin)
//
#include <MsTimer2.h>

//#define POT_ALARM_ENABLED
#define HB_ALARM_ENABLED
#define POT_ALARM_ENABLED

// i/o constants
const int hb_input_pin = 8;   // hearbeat input pin
const int pot_pin = 0;  // Analog input pin that the potentiometer is attached to

const int hb_alarm_pin = 6;   // HB alarm (red)
const int pot_alarm_pin = 7;  // pot alarm (green)


// timing parameters
unsigned int dT = 10;

// alarm criteria
const int  pot_excursion = 100;         // amount of pot ADC variation over the interval (units of 0.1%)
const unsigned int T_pot_read = 100;    // interval to check pot variation (milliseconds)
const unsigned int T_pot_check = 2000;  // interval to read the pot (milliseconds)

// 10Hz square wave heartbeat signal, 20 transitions per second

const int  hb_min_count = 15;           // minimum number of heartbeats in interval T_hb_check
const int  hb_max_count = 25;           // maximum number of heartbeats in interval T_hb_check
const unsigned int T_hb_check = 1000;   // interval to count hearbeat transitions (milliseconds)

// local variables
int n_hb_check = T_hb_check;     // heartbeat check downcounter
int n_pot_read = T_pot_read;     // pot read downcounter
int n_pot_check = T_pot_check;   // pot read downcounter

unsigned char hb_previous = 0;   // heartbeat previous value
unsigned int hb_transitions = 0; // number of heartbeat transitions within interval
unsigned int pot_min = 0xffff;   // minimum pot value within interval
unsigned int pot_max = 0;        // maximum pot value within interval

volatile bool hb_alarm = false;     // whether the heartbeat alarm should ring
volatile bool pot_alarm = false;    // whether the pot alarm should ring
uint8_t hb_alarm_state = LOW;       // heart beat alarm pin state (toggle for tone)
uint8_t pot_alarm_state = LOW;      // pot alarm pin state (toggle for tone)

void loop() {
  
  //+++++++++++++++++++++++++++ HEARTBEAT CHECK ++++++++++++++++++++++++++++++++
  #ifdef HB_ALARM_ENABLED
  // count transitions on the heartbeat
  unsigned char hb_current = digitalRead(hb_input_pin);
  if (hb_previous ^ hb_current) {
    hb_transitions++; // update the transition counter
    Serial.print("hb transition\n");
  }
  hb_previous = hb_current;

  // periodically check if there are sufficient transitions
  n_hb_check -= dT;
  if (n_hb_check < 0) {
    // check the heartbeat
    if (hb_transitions < hb_min_count || hb_transitions > hb_max_count) {
      // ** INCORRECT NUMBER OF HEARTBEAT PULSES IN THE INTERVAL
      hb_alarm = true;
    } else {
      hb_alarm = false;
      // debatable if we do anything if alarm condition disappears
    }

    hb_transitions = 0;  // reset the transition counter
    n_hb_check = T_hb_check; // reset the counter
    //Serial.print("hb check\n");
  }
  #endif
  //+++++++++++++++++++++++++++ POTENTIOMETER CHECK ++++++++++++++++++++++++++++++++
  #ifdef POT_ALARM_ENABLED
  // periodically read the pot
  n_pot_read -= dT;
  if (n_pot_read < 0) {
    // time to read the pot
    unsigned int val = analogRead(pot_pin); // read value
    if (val < pot_min)
      pot_min = val;
    else if (val > pot_max)
      pot_max = val;
    n_pot_read = T_pot_read;
  }

  // periodically check if there is sufficient variation
  n_pot_check -= dT;
  if (n_pot_check < 0) {

    // check if the span is reasonable
    if ((pot_max - pot_min) < pot_excursion) {
      // ** POTENTIOMETER INPUT IS NOT VARYING **
      pot_alarm = true;
    } else {
      // debatable if we do anything if alarm condition disappears
      pot_alarm = false
    }
    pot_min = 0xffff;
    pot_max = 0;

    n_pot_check = T_pot_check; // reset the counter
    //Serial.print("pot check\n");
  }
  #endif
  delay(dT);
}

void setup() {

  // configure the i/o pins
  pinMode(hb_alarm_pin, OUTPUT);              // heartbeat alarm (red)
  digitalWrite(hb_alarm_pin, LOW);

  pinMode(pot_alarm_pin, OUTPUT);              // pot variation alarm (green)
  digitalWrite(pot_alarm_pin, LOW);
  
  pinMode(hb_input_pin, INPUT_PULLUP); // heartbeat input

  // connect serial port for debug and command
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Serial.print("QUT Ventilator safety monitor - starting up...\n");
  delay(5000);  // wait for the machine to start up and some motion
  Serial.print("QUT Ventilator safety monitor - checking\n");

  MsTimer2::set(1, update);
  MsTimer2::start();
}

void update() {
  if (hb_alarm) {
    digitalWrite(hb_alarm_pin, hb_alarm_state);
    hb_alarm_state = HIGH - hb_alarm_state;
  } else {
    digitalWrite(hb_alarm_pin, LOW);
  }

  if (pot_alarm) {
    digitalWrite(pot_alarm_pin, pot_alarm_state);
    pot_alarm_state = HIGH - pot_alarm_state;
  } else {
    digitalWrite(pot_alarm_pin, LOW);
  }
}
