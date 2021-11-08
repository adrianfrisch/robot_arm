/*
 * Inverse kinematics for the 2 axis of the robot arm. 
 * 
 * Formulas for calculation are from 
 * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
 * 
 */
#define lowerArmLength 14
#define upperArmLength 14

float lowerArmSquare = lowerArmLength*lowerArmLength;
float upperArmSquare = upperArmLength*upperArmLength;
float denominator = 2*lowerArmLength*upperArmLength;
float rad;
float getUpperAngle(float x, float y) {
  // store in local rad variable to use in lower Angle calculation
  rad = acos((x*x+y*y-lowerArmSquare-upperArmSquare)/denominator);
  // convert to degrees and return
  return rad * 57296 / 1000;  
}

float getLowerAngle(float x, float y) {
  float radLow = atan(y/x)+atan((upperArmLength*sin(rad))/(lowerArmLength+upperArmLength*cos(rad)));
  // convert to degrees and return
  return radLow * 57296 / 1000;
}

/*
 * Method to step through the whole theoretically covered area of the robot arm 
 * (both arms fully extended = farthest reachable point -> max for x and y)
 * 
 * This also includes points not reachable when x+y > 24 where we will get NaN values. 
 */
void simulateKinematics() {
   for (int x = 0; x < lowerArmLength+upperArmLength ; x++) {
    for (int y = 0; y < lowerArmLength+upperArmLength ; y++) {
      float uA = getUpperAngle(x,y);
      float lA = getLowerAngle(x,y);
      Serial.print(x);
      Serial.print(',');
      Serial.print(y);
      Serial.print(',');
      Serial.print(uA);
      Serial.print(',');
      Serial.print(lA);
      Serial.print('\n');
      delay(100);
    }
  }
}
