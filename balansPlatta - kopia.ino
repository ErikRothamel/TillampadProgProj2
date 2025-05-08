#include "Adafruit_VL53L0X.h"
#include "Servo.h"

#include "VL53L1X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo servo;


const int servoPin = 3;

float distance = 0;

double sp = 0, pv, ev;
double Kp = 0.2, Ki = 0.03, Kd = 0.03;

double I = 0;
double prevEv;

float filteredDistance = 0;
float alpha = 0.1; // Justerbar lågpassfilterfaktor


void setup() {
  servo.attach(servoPin);
  Serial.begin(115200);
  servo.write(90);

  //SetROI(4, 4)

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

  // start continuous ranging
  lox.startRangeContinuous();


}

void loop() {
  
  if (lox.isRangeComplete()) {
    float controlSignal = pidFunc();         // Få PID-reglerad styrsignal
    int servoValue = constrain(90 + controlSignal, 0, 180);  // Justera kring mittläget

    servo.write(servoValue);                 // Skriv till servot
    Serial.print("Distance: ");
    Serial.print(pv);
    Serial.print("  Servo: ");
    Serial.println(servoValue);
  }


}

float servoMove(){
  float out;
  out = pidFunc();
  //Serial.print(" ");
  //Serial.println();



  return out;
}


float getDistance() {
  if (lox.isRangeComplete()) {
    float raw = map(lox.readRange(), 0, 200, -100, 100);
    filteredDistance = alpha * raw + (1 - alpha) * filteredDistance;
  }
  return filteredDistance;
}

float pidFunc() {
  pv = getDistance();
  double out;
  ev = pv - sp;
  
  double D;
  double evHold;
  I = I + ev;
  D = ev;
  
  double evDiff;
  evDiff = prevEv - ev;
  prevEv = ev;
  
  out = Kp * ev + Ki * I + Kd * evDiff;

  return out;
}









