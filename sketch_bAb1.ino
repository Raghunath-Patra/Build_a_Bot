#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Stepper myStepper(stepsPerRevolution, 5, 3, 4, 2);
Stepper myStepper1(stepsPerRevolution, 8, 10, 9, 11);

int reset = 0;
int invalid_counts = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
// uint8_t new_address = 0x30;
// lox.setAddress(new_address);
  while (! Serial) {
    Serial.println("not connected");
  delay(1);
  }
  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1); // Stop execution if sensor fails
  }
  myStepper.setSpeed(15);   // Motor 1 speed (60 RPM)
  myStepper1.setSpeed(15);
}

void loop() {
  
  int zCounts = 1024 ;
  

  //digitalWrite(zDir, LOW);
  for (int j = 0; j < zCounts; j++) {
    
    for (int i = 0; i < stepsPerRevolution/2; i++) {
      
      double senseDistance = readToFSensor();
      if(senseDistance > 25){
        invalid_counts++;
      }
      Serial.print(j); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.println(senseDistance);
      rotateMotor(myStepper1, 2);
      delay(5);
      if(invalid_counts >= 5){
      delay(500);
      break;
    }
    }
    if(invalid_counts){
      delay(500);
      break;
    }
    rotateMotor(myStepper, stepsPerRevolution);
    reset++;
    delay(500);
    invalid_counts = 0;
  }

  Serial.println("SCAN COMPLETE"); 
  resetFunction(reset, stepsPerRevolution );
  while (true);  // Stop scanning permanently
}

void rotateMotor(Stepper &motor, int steps) {
  for (int i = 0; i < steps; i++) {
    motor.step(-1);
    delay(5);
  }
}

void resetFunction(int resetCount, int steps){
  while(resetCount--){
    for (int i = 0; i < steps; i++) {
      myStepper.step(1);
      delay(5);
    }
  }
}

double readToFSensor() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter / 10.0;  // Convert mm to cm
  } else {
    return -1;  // Invalid measurement
  }
}
