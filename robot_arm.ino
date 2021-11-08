#include <Wire.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>

uint8_t BASE = 0;
uint8_t BOTTOM_ARM = 1;
uint8_t TOP_ARM = 2;
#define GRIPPER 3

#define SERVO_FREQ 50
// Servo move ranges
#define BASE_SERVOMIN 220
#define BASE_SERVOMAX 380
#define BOTTOM_SERVOMIN 245
#define BOTTOM_SERVOMAX 360
#define TOP_SERVOMIN 230
#define TOP_SERVOMAX 380


#define MOVE_TIME_SEC 2
#define PAUSE_TIME 1
#define SLEEPTIME_MILLIS 10

rampInt baseRamp;
rampInt bottomRamp;
rampInt topRamp;

// Using the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int pulselength(float degrees, int servoMin, int servoMax){
  // Input range is a decimal with only small range. So we map with 10 times 
  // precision to target range to avoid jittering.
  return map(degrees*10, 0, 180*10, servoMin, servoMax);
}

void moveServos() {
  uint16_t x = baseRamp.getValue();
  Serial.print(x);
  Serial.print(',');
  pwm.setPWM(BASE,0,x);
  x = bottomRamp.getValue();
  Serial.print(x);
  Serial.print(',');
  pwm.setPWM(BOTTOM_ARM,0,x);
  x = topRamp.getValue();
  Serial.print(x);
  Serial.print('\n');
  pwm.setPWM(TOP_ARM,0,x);   
}

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Arm Test");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  // Initialize all Ramps to center
  baseRamp.go(pulselength(90, BASE_SERVOMIN, BASE_SERVOMAX),5);
  bottomRamp.go(pulselength(90, BOTTOM_SERVOMIN, BOTTOM_SERVOMAX),5);
  topRamp.go(pulselength(90, TOP_SERVOMIN, TOP_SERVOMAX),5);  
  // Wait until center positions initialized
  delay(10);
  // pwm.setPWM(GRIPPER,0,375);  
}

void updateServoPositions(){
  baseRamp.update();
  bottomRamp.update();
  topRamp.update(); 
  moveServos();  
}

byte positionIndex = 0;
byte baseIndexes[] = {0,180,90,90,90,90,90,90,90};
byte bottomIndexes[] = {90,90,90,0,180,90,90,90,90};
byte topIndexes[] = {90,90,90,90,90,90,0,180,90};

void positionUpdate() {
  if (!baseRamp.isRunning()){
    Serial.print("Position reached. Moving to next position.");
    delay(PAUSE_TIME*1000);
    baseRamp.go(pulselength(baseIndexes[positionIndex], BASE_SERVOMIN, BASE_SERVOMAX), MOVE_TIME_SEC*1000);
    bottomRamp.go(pulselength(bottomIndexes[positionIndex], BOTTOM_SERVOMIN, BOTTOM_SERVOMAX), MOVE_TIME_SEC*1000);
    topRamp.go(pulselength(topIndexes[positionIndex], TOP_SERVOMIN, TOP_SERVOMAX), MOVE_TIME_SEC*1000);
    positionIndex++;
    if (positionIndex >= sizeof(baseIndexes)){
      positionIndex=0;      
    }
  }
}

void loop() {  
  bottomRamp.go(pulselength(0, BOTTOM_SERVOMIN, BOTTOM_SERVOMAX),5);
  delay(10);
  updateServoPositions(); 
  delay(5000);
  bottomRamp.go(pulselength(180, BOTTOM_SERVOMIN, BOTTOM_SERVOMAX),5);
  delay(10);
  updateServoPositions(); 
  delay(5000);

  //updateServoPositions();   
  //positionUpdate();
  //simulateKinematics();
  
  //delay(SLEEPTIME_MILLIS);
}
