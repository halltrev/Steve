#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 2
SoftwareSerial LCDSerial(3,2); // pin 2 = TX, pin 3 = RX (unused)
char salinitystring[20]; // create string array
int salinity_power_pin = 6; // Digital I/O pin, Global variable
//int readings[] = {-7,0,22,145,569,570,646,690,1023};
int solDI_pin = 9;
int solSalt_pin = 10;
int salinity_input_pin = A1; // Analog input pin
float salinityReading;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  LCDSerial.begin(9600);
  delay(500);
  clearScreen();
  LCDSerial.write(254);
  LCDSerial.write(128);
  LCDSerial.write("Salinity:            ");
  pinMode(solDI_pin, OUTPUT);
  pinMode(solSalt_pin, OUTPUT);
  pinMode (salinity_power_pin, OUTPUT);
  //for(int i = 0; i < 9; i++) {
    //Serial.println(salinity_value(readings[i]),9);  
  //}
}

void loop() {
salinityReading = salinity_value(sensor_reading( salinity_power_pin, salinity_input_pin ));
Serial.println (salinityReading,9);
dtostrf(salinityReading,11,9,salinitystring);
//sprintf(salinitystring,"%4d",salinityReading); // create strings from the numbers
LCDSerial.write(254); // cursor to 7th position on first line
LCDSerial.write(137);
LCDSerial.write(salinitystring);

}

int sensor_reading( int power_pin, int input_pin ) {
  int reading;
  digitalWrite (power_pin, HIGH ); // Turn on the sensor
  delay(100); // Wait to settle
  reading = analogRead (input_pin ); // Read voltage
  Serial.print(reading); Serial.print(" ");
  digitalWrite (power_pin, LOW ); // Turn off the sensor
  delay(100); // Wait to settle
  return reading;
}

float salinity_value(int reading) {
 int rbreak = 570; // separation between linear segments
 int rlimit = 1023; // upper limit of acceptable readings
 float salinity; // Compute salinity from calibration equations
 if ( reading<0 ) {
 // print error message, reading can’t be negative
 } else if ( reading<rbreak ) {
 salinity = .0000822999*reading;
 } else if ( reading<=rlimit ) {
 salinity = (.00000405447*pow(reading,2))-(.00426449*reading)+1.16329;
 } else {
 // do something to be safe
 }
 return(salinity);
}

 void clearScreen()
{
  //clears the screen, you will use this a lot!
  LCDSerial.write(0xFE);
  LCDSerial.write(0x01); 
}
