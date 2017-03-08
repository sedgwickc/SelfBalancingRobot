/* Self Balancing Robot
 * Charles Sedgwick
 * 
 * This sketch uses a PID controller to allow a two wheeled robot to balance upright.
 * The PID library used can be found at the author's website: brettbeauregard.com
 * 
 * The IMU used is the Adafruit 10DOF sensor. 
 * 
 * This sketch can be used according to the terms of the GPL v3.0 licence.
 */

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <PID_v1.h>

#define ABACKWARD 3
#define AFORWARD 5
#define BFORWARD 6
#define BBACKWARD 11

// PID controller values
double output_motor_speed;
double setpoint_pitch;
double input_pitch;
/* The Kp, Ki, and Kd values must be tuned for each robot */
double Kp=24.0, Ki=0, Kd=0.0;

PID myPID(&input_pitch, &output_motor_speed, &setpoint_pitch, Kp, Ki, Kd, DIRECT);


/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;


void forwardWSpeed(int speed){
  analogWrite(AFORWARD, speed);  
  digitalWrite(ABACKWARD, LOW);
  analogWrite(BFORWARD, speed); 
  digitalWrite(BBACKWARD, LOW);   
}

void backwardWSpeed(int speed){  
  digitalWrite(AFORWARD, LOW);   
  analogWrite(ABACKWARD, speed);
  digitalWrite(BFORWARD, LOW);  
  analogWrite(BBACKWARD, speed);   
}

void i2cScan(void){
    byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    Serial.print("Testing address: ");
    Serial.print(address, HEX);
    Serial.println("  !");
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.begin(address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    Serial.print("ret value: ");
    Serial.print(error, HEX);
    Serial.println("  !");
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
  else
  {
    Serial.println("done");
  }
  
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  
  delay(500);
}

void setup() {
  Serial.begin (115200);

   Serial.println("Setting up motor pins");
   pinMode(AFORWARD, OUTPUT);
   pinMode(ABACKWARD, OUTPUT);
   pinMode(BFORWARD, OUTPUT);
   pinMode(BBACKWARD, OUTPUT);


  Serial.println ();

    /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  } else {
    Serial.println("ADXL345 detected");
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  } else {
    Serial.println("LSM303 detected");
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
   input_pitch = 278.0;
   setpoint_pitch = 278.0;

   myPID.SetMode(AUTOMATIC);
}  // end of setup

void loop() {

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_event_t bmp_event;
    sensors_vec_t   orientation;

    /* Calculate pitch and roll from the raw accelerometer data */
  
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);
    if( dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation)){
        input_pitch = orientation.heading;
        Serial.print("Current mag pitch: ");
        Serial.print( input_pitch );
        Serial.println("");
    }

    if( input_pitch > setpoint_pitch ){
        double diff = input_pitch - setpoint_pitch;
        input_pitch = setpoint_pitch - diff;
        myPID.Compute();
        forwardWSpeed(output_motor_speed);
    } else {
        myPID.Compute();
        backwardWSpeed(output_motor_speed);
    }
     
    Serial.print("output speed: ");
    Serial.print( output_motor_speed );
    Serial.println(""); 

}

void turnRight(){
   digitalWrite(AFORWARD, LOW);
   digitalWrite(ABACKWARD, LOW);
   
   digitalWrite(BFORWARD, HIGH);
   digitalWrite(BBACKWARD, LOW);
}

void turnLeft(){
   digitalWrite(AFORWARD, HIGH);
   digitalWrite(ABACKWARD, LOW);
   
   digitalWrite(BFORWARD, LOW);
   digitalWrite(BBACKWARD, LOW);
}

void rotateLeft(){
   digitalWrite(AFORWARD, HIGH);
   digitalWrite(ABACKWARD, LOW); 
  
   digitalWrite(BFORWARD, LOW);
   digitalWrite(BBACKWARD, HIGH); 
}

void rotateRight(){
   digitalWrite(AFORWARD, LOW);
   digitalWrite(ABACKWARD, HIGH); 
  
   digitalWrite(BFORWARD, HIGH);
   digitalWrite(BBACKWARD, LOW); 
}
