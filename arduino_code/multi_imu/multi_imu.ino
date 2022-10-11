#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define TCA_ADDR 0x70
#define BNO055_ADDR 0x28 

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, BNO055_ADDR);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, BNO055_ADDR);

float angle[2] = {0, 0}; // {IMU 1, IMU 2}
float cal[2] = {0, 0}; // {IMU 1, IMU 2}
bool cal_state[2] = {false, false}; // {IMU 1, IMU 2}
float start_time = 0; // {current time, start time}

// Select I2C BUS
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(TCA_ADDR);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup(void) {
  Serial.begin(115200);
//  Serial.println("Orientation Sensor Test"); Serial.println("");

  Wire.begin();
  
  /* Initialise the sensor */
  TCA9548A(0);
  if(!bno1.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno1.setExtCrystalUse(true);
  
  TCA9548A(1);
  if(!bno2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno2.setExtCrystalUse(true);
  start_time = millis();
}

void loop(void) {
  
  TCA9548A(0);
  imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);

  if ((millis() - start_time) > 2000) {
    if (cal_state[0] == false){
      cal[0] = euler1.x();
      cal_state[0] = true;
    }
  }

  angle[0] = euler1.x() - cal[0];
  
  TCA9548A(1);
  imu::Vector<3> euler2 = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);

  if ((millis() - start_time) > 2000) {
    if (cal_state[1] == false) {
      cal[1] = euler2.x();
      cal_state[1] = true;
    }
  }

  
  angle[1] = euler2.x() - cal[1];

  if (Serial.available() > 0){
    while (Serial.available() > 0){
      Serial.read();
    }
    Serial.print(angle[0]);
    Serial.print(",");
    Serial.println(angle[1]);
  } //else Serial.println(1);
}
