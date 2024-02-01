#include <Arduino.h>
#include <SPI.h>
#include <PID_v1.h>

#define ntc_pin_inside A0         // Pin,to which the voltage divider inside is connected
#define ntc_pin_outside A1         // Pin,to which the voltage divider outside is connected
#define vd_power_pin 2        // 5V for the voltage divider
#define nominal_resistance 10000       //Nominal resistance at 25⁰C
#define nominal_temeprature 25   // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5    // Number of samples
#define beta 3950  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000   //Value of  resistor used for the voltage divider




int samples = 0;   //array to store the samples
float temp_inside = 0; // temperature of the inside thermostat
float temp_outside = 0; // temperature of the outside thermostat
const int SS_PIN_inside = 9;  // Chip Select of the inside display
const int SS_PIN_outside = 10;  // Chip Select of the outside display
//Define Variables we'll be connecting to

double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=0.5, Kd=0.25;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);




float readTemperature(int ntc_pin); // Function to read the temperature, both inside and outside (ntc_pin is the inside or outside pin)
float get_target_temp(float temp_outside);

void sendDigit(uint8_t digit, int SS_PIN); // Function to write digit on display

// function to control heating and cooling blanket, changes nominal inside temperature according to outside temmperature


void setup(void) {

  pinMode(vd_power_pin, OUTPUT);

  Serial.begin(9600);   //initialize serial communication at a baud rate of 9600

    // following for display 
  pinMode(SS_PIN_inside, OUTPUT);
  pinMode(SS_PIN_outside, OUTPUT);
  SPI.begin(); 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // Adjust SPI settings as needed

 Serial.begin(9600);

 Setpoint = get_target_temp(temp_outside);
  //initialize the variables we're linked to
  Input = temp_inside;



  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

void loop(void) {
  temp_inside = readTemperature(ntc_pin_inside); // get temperature of the inside thermostat
   Serial.print("temp_inside ");
  Serial.println(temp_inside);
    temp_outside = readTemperature(ntc_pin_outside); // get temperature of the outside thermostat 
     Serial.print("temp_outside ");
  Serial.println(temp_outside);
  // Input = temp_inside;
  myPID.Compute();
  analogWrite(ntc_pin_outside, Output);
  Serial.print("PID ");
  Serial.println(Output);
  delay(1000);

  

    int itemp_inside;
    int itemp_outside;

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



o[1] = itemp_outside %10;
itemp_outside/=20;
o[0] = itemp_outside;

i[1] = itemp_inside %10;
itemp_inside/=20;
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

//  Serial.println("\n \n");

//  Serial.print("ADC readings ");

//  Serial.println(average);

  // Calculate NTC resistance

  average = 1023 / average - 1;

  average = Rref / average;

//  Serial.print("Thermistor resistance ");

//  Serial.println(average);

  float temperature;

  temperature = average / nominal_resistance;     // (R/Ro)

  temperature = log(temperature);                  // ln(R/Ro)

  temperature /= beta;                   // 1/B * ln(R/Ro)

  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)

  temperature = 1.0 / temperature;                 // Invert

  temperature -= 273.15;                         // convert absolute temp to C

  Serial.print("Temperature ");

  Serial.print(temperature);

  Serial.println(" *C");

  return temperature;
}

// Temperature function
float tempFunction(float temp_outside) {
    return -0.6 * temp_outside + 21;
}   
float get_target_temp(float temp_outside){
  float Setpoint;
  if (temp_outside > 3.33f)
  {
    Setpoint = 19;
  }
  else
  {
    Setpoint = tempFunction(temp_outside);
  }
  return Setpoint;
}


void sendDigit(uint8_t digit, int SS_PIN) {
  digitalWrite(SS_PIN, LOW);  // Enable the slave device
  SPI.transfer(digit);         // Send the digit data
  digitalWrite(SS_PIN, HIGH); // Disable the slave device
}

