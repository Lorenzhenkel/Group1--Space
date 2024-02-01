#include <Arduino.h>
#include <SPI.h>
#include <pid_ctrl.h>
#include <util.h>
#include <iostream>
#include <PID_v1.h>
#define ntc_pin_inside A0         // Pin,to which the voltage divider inside is connected
#define ntc_pin_outside A1         // Pin,to which the voltage divider outside is connected
#define test_pin A3 // Test
#define vd_power_pin 2        // 5V for the voltage divider
#define nominal_resistance 10000       //Nominal resistance at 25⁰C
#define nominal_temeprature 25   // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5    // Number of samples
#define beta 3950  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000   //Value of  resistor used for the voltage divider

/* GLOBAL VARIABLES */
pid_params pid;
PIDControl controller; 

// volatile float ctrl;
int samples = 0;   //array to store the samples
float temp_inside = 0; // temperature of the inside thermostat
float temp_outside = 0; // temperature of the outside thermostat
const int SS_PIN_inside = 9;  // Chip Select of the inside display
const int SS_PIN_outside = 10;  // Chip Select of the outside display


float readTemperature(int ntc_pin); // Function to read the temperature, both inside and outside (ntc_pin is the inside or outside pin)
float get_target_temp(float temp_outside);
void sendDigit(uint8_t digit, int SS_PIN); 

void setup(void) {

  pinMode(vd_power_pin, OUTPUT);

  Serial.begin(9600);   //initialize serial communication at a baud rate of 9600   

  controller.setup_pid(A3, 10, 250, 10.0f);
    // following for display 
  pinMode(SS_PIN_inside, OUTPUT);
  pinMode(SS_PIN_outside, OUTPUT);
  SPI.begin(); 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // Adjust SPI settings as needed

}

void loop(void) {


  
    temp_inside = readTemperature(ntc_pin_inside); // get temperature of the inside thermostat
  
    temp_outside = readTemperature(ntc_pin_outside); // get temperature of the outside thermostat 
  

    int itemp_inside = 0;
    int itemp_outside = 0;

    /*temp_inside += 0.5;
    temp_outside += 0.5;
    itemp_inside = temp_inside;
    itemp_outside = temp_outside;*/

    itemp_inside = static_cast<int>(round(temp_outside));
    itemp_outside = static_cast<int>(round(temp_inside));

  // Use arrays to store binary values for unit and dec
  const uint8_t Binaries[] = {126, 10, 182, 158, 202, 220, 252, 14, 254, 222};

  int i[2]; // inside temperature converted to int
  int o[2]; // outside temperature converted to int



float setpoint = get_target_temp(temp_outside);
float error = setpoint - temp_inside;
float dt = 100;
pid.k_p = 0.5f;
pid.k_i = 0.4f;
pid.k_d = 0.3f;
controller.pid_ctrl(&pid, error, dt);
Serial.print("Temperature Outside: ");
Serial.println(temp_outside);

Serial.print("Setpoint: ");
Serial.println(setpoint);

Serial.print("Error: ");
Serial.println(error);

// Add a delay or wait for the next iteration
delay(1000); // 1 second delay



o[1] = itemp_outside %10;
itemp_outside/=10;
o[0] = itemp_outside;

i[1] = itemp_inside %10;
itemp_inside/=10;
i[0] = itemp_inside;


    sendDigit(Binaries[o[1]], SS_PIN_outside); // send right digit of outside temperature
    sendDigit(Binaries[o[0]], SS_PIN_outside); // send left digit of outside temperature
    sendDigit(Binaries[i[1]], SS_PIN_inside); // send right digit of inside temperature
    sendDigit(Binaries[i[0]], SS_PIN_inside); // send left digit of inside temperature
    delay(2000);

}




float readTemperature(int ntc_pin) {

  uint8_t i;

  float average;

  samples = 0;

  // take voltage readings from the voltage divider

  digitalWrite(vd_power_pin, HIGH);

  for (i = 0; i < samplingrate; i++) {

    samples += analogRead(ntc_pin);

    delay(10);

  }

  digitalWrite(vd_power_pin, LOW);

  average = 0;

  average = samples / samplingrate;

  // Serial.println("\n \n");

 // Serial.print("ADC readings ");

  // Serial.println(average);

  // Calculate NTC resistance

  average = 1023 / average - 1;

  average = Rref / average;

  // Serial.print("Thermistor resistance ");

 // Serial.println(average);

  float temperature;

  temperature = average / nominal_resistance;     // (R/Ro)

  temperature = log(temperature);                  // ln(R/Ro)

  temperature /= beta;                   // 1/B * ln(R/Ro)

  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)

  temperature = 1.0 / temperature;                 // Invert

  temperature -= 273.15;                         // convert absolute temp to C

  // Serial.print("Temperature ");

  // Serial.print(temperature);

  // Serial.println(" *C");

  return temperature;
}



void sendDigit(uint8_t digit, int SS_PIN) {
  digitalWrite(SS_PIN, LOW);  // Enable the slave device
  SPI.transfer(digit);         // Send the digit data
  digitalWrite(SS_PIN, HIGH); // Disable the slave device
}


/*
PIDControl::PIDControl(){}

PIDControl::~PIDControl(){
  pinMode(_pin, INPUT); // Have to select here
}

void PIDControl::reset(){
    _error[0] = _error[1] = 0;
    _prev_ctrl = 0;
}

void PIDControl::setup_pid(uint8_t pin, float min, float max, float f){
    _pin = pin;
    _min = min;
    _max = max;
    _error[0] = 0;
    _error[1] = 0;
    _prev_ctrl = 0;
    pinMode(pin, OUTPUT);  //  set pin as output
    analogWriteFrequency(pin, ABS(f));  // set PWM frequency
}

void PIDControl::pid_ctrl(const pid_params *pid, float error, float dt){
  //PID Logic um PWM einzustellen (wenn ctrl so und so dann set PWM Wert) -------------------------------------------------------
 float ctrl = _get_pid_ctrl(pid, &error, &dt);

 float var1;
 float var2;
 float var3;
 float pinName1;
 float pinName2;
 if (ctrl < var1) {
  digitalWrite(pinName1, HIGH);
  digitalWrite(pinName2, LOW);
 }
 else if(ctrl > var1) {
  digitalWrite(pinName1, LOW);
  digitalWrite(pinName2, HIGH);
 }
 else { 
  digitalWrite(pinName1, LOW);
  digitalWrite(pinName2, LOW);
  }

}

// DO NOT ALTER THIS METHOD
float PIDControl::_get_pid_ctrl(const pid_params *params,
                               const float *error,
                               const float *dt){
  //  Discrete PID Control derived from Z-Transform
  float kd_div_dt = params->k_d / (*dt);
  float ki_div_2_dt = params->k_i / 2.0f * (*dt);
  float kp = params->k_p;
  float ctrl = _prev_ctrl;

  ctrl += (kp + ki_div_2_dt + kd_div_dt) * (*error);
  ctrl -= (kp - ki_div_2_dt + 2.0f * kd_div_dt) * _error[1];
  ctrl += kd_div_dt * _error[0];
  _prev_ctrl = ctrl;

  _error[0] = _error[1];
  _error[1] = *error;

  return ctrl;
}
*/