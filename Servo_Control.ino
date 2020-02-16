//This function maps the Servo Angles and sends the commands to the servos

void SetServoPosition() {

  // Set X Positions
  pwm.setPWM(X1SERVO, 0, ConvertToCounts(XFromPi + X1DegComp));
  pwm.setPWM(X2SERVO, 0, ConvertToCounts(XFromPi + X2DegComp));
//  Serial.print("X ");
//    Serial.println((-1*(XFromPi + X2DegComp)));

  // Set Y Positions
  pwm.setPWM(Y1SERVO, 0, ConvertToCounts(YFromPi + Y1DegComp));
  pwm.setPWM(Y2SERVO, 0, ConvertToCounts(-1*(YFromPi + Y2DegComp)));

  // Set Laser On/Off and Set Eyelid Position
  if (L1FromPi == 0 and L2FromPi == 0) {
    digitalWrite(LaserPin, LOW);
    pwm.setPWM(L1SERVO, 0, ConvertToCounts(L1FromPi + L1DegComp + LidCloseComp)); //Bottom Eyelid
    pwm.setPWM(L2SERVO, 0, ConvertToCounts(L2FromPi + L2DegComp + LidCloseComp)); //Top Eyelid
  }
  else {
  digitalWrite(LaserPin, HIGH);
  pwm.setPWM(L1SERVO, 0, ConvertToCounts(L1FromPi + L1DegComp)); //Bottom Eyelid
  pwm.setPWM(L2SERVO, 0, ConvertToCounts(L2FromPi + L2DegComp)); //Top Eyelid
  }
//    Serial.println("L1FromPi =");
//    Serial.println(L1FromPi);
//    Serial.println("L2FromPi =");
//    Serial.println(L2FromPi);
  delay(5);                           // waits for the servo to get there
}

//This function converts degree values to servo "counts" as required by the Adafruit_PWMServo library

float ConvertToCounts(float Degrees) {
  float Counts;
  Counts = MidPtCounts + (CtsPerDeg * Degrees);
//  Serial.print("Counts ");
//  Serial.println(Counts);
    
           return Counts;
          
}
