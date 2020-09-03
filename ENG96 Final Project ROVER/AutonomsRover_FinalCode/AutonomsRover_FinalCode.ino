

// MOSI = 51, MISO = 50, SCK = scl = 52, CS =  = 53
// sda = data // scl goes to scl,
// Get Gyroscope up and running

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float yaw = 0;
float GyroZ, gyroAngleZ, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

int leftSpeed = 255; //255
int rightSpeed = 225; //150
const int roverLength = 12; //check this value
int desired_yaw = 0;

float distanceFL;
float distanceFR;
float distanceL;
float distanceR;
float distanceB;
bool wallOnLeft;
bool wallOnRight;
bool wallInFront;
bool wallBehind;

#define leftIn1 26 // bottom left wheel 
#define leftIn2 28 //
#define leftIn3 24
#define leftIn4 22
#define leftENA A0
#define leftENB A1

#define rightIn1 52
#define rightIn2 50
#define rightIn3 46 // bottom right wheel 
#define rightIn4 48 //
#define rightENA A15
#define rightENB A14

#define leftTrig A2
#define leftEcho A3
#define rightTrig A12
#define rightEcho A13
#define frontLeftTrig A4
#define frontLeftEcho A5
#define frontRightTrig A10
#define frontRightEcho A11
#define backTrig A7
#define backEcho A6



void setup()
{
  Serial.begin(19200);

  pinMode(leftIn1, OUTPUT); pinMode(leftIn2, OUTPUT);
  pinMode(leftIn3, OUTPUT); pinMode(leftIn4, OUTPUT);
  pinMode(leftENA, OUTPUT); pinMode(leftENB, OUTPUT);

  pinMode(rightIn1, OUTPUT); pinMode(rightIn2, OUTPUT);
  pinMode(rightIn3, OUTPUT); pinMode(rightIn4, OUTPUT);
  pinMode(rightENA, OUTPUT); pinMode(rightENB, OUTPUT);

  pinMode(leftTrig, OUTPUT); pinMode(leftEcho, INPUT);
  pinMode(rightTrig, OUTPUT); pinMode(rightEcho, INPUT);
  pinMode(frontLeftTrig, OUTPUT); pinMode(frontLeftEcho, INPUT);
  pinMode(frontRightTrig, OUTPUT); pinMode(frontRightEcho, INPUT);
  pinMode(backTrig, OUTPUT); pinMode(backEcho, INPUT);

  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU); // Start communication
  Wire.write(0x6B);  // Communicate with register 6B
  Wire.write(0x00); // Reset by placing a 0 into the 6B register
  Wire.endTransmission(true);
  calculate_IMU_error();


}
bool flag = true;
int count = 0;
void loop()
{
  forward();
/*
  forward();
  if (readFLDistance() < 7 || readFRDistance() < 7)
  { stop();
  
    delay(100);
    if (readLDistance() > readRDistance()) //if able to go around object to the left
    {
      left();
      delay(100);
      double startingDistance = 0;
      for (int k = 0; k < 100; k++)
      {
        startingDistance += readBDistance();
      }
      startingDistance = (startingDistance / 100);
      //double check this


      while ((readBDistance() - startingDistance) < 3 && readFLDistance() > 7) //overshoots 6 inches, so actually goes 14 inches-- the distance of the rover
      {

        forward();
      }
      stop();

      arcRight();
      forward();
      delay(200);
      stop();
      delay(200);
      //need to figure out this number


    }
    else if (readRDistance() > readLDistance())
    {
      right();
      delay(100);
      double startingDistance = 0;
      for (int k = 0; k < 100; k++)
      {
        startingDistance += readBDistance();
      }
      startingDistance = (startingDistance / 100);
      //double check this


      while ((readBDistance() - startingDistance) < 3 && readFLDistance() > 7) //if all the way across, this expression is 11.2/// one length away from wall is 22.7
      {

        forward();
      }
      stop();

      arcLeft();
      //need to figure out this number
      forward();
      delay(200);
      stop();  //go forward to pass the object
      delay(200);
    }
  }
*/
  if( (readRDistance()+readLDistance()) > 50) //now we are in the big area
  {
    float startLeft = readLDistance();
    float startRight = readRDistance();
    while((3+readRDistance()) > startRight && (3+readLDistance()) > startLeft && readFLDistance > 7)
      {
        forward();
      }
      stop();
      backward();
      delay(100);
      stop();
      
  
      
  }

  

}

float readLDistance()
{
  digitalWrite(leftTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(leftTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrig, LOW);
  distanceL = pulseIn(leftEcho, HIGH) * 0.0133 / 2;

  return distanceL;
}
float readRDistance()
{
  digitalWrite(rightTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(rightTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrig, LOW);
  distanceR = pulseIn(rightEcho, HIGH) * 0.0133 / 2;

  return distanceR;
}
float readFLDistance()
{


  // Ensure that the trigger pin is not sending out anything
  digitalWrite(frontLeftTrig, LOW);
  delayMicroseconds(2);

  // Activate the trigger pin for 10 microseconds
  digitalWrite(frontLeftTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontLeftTrig, LOW);


  distanceFL = pulseIn(frontLeftEcho, HIGH) * 0.0133 / 2;

  return distanceFL;
}
float readFRDistance()
{

  digitalWrite(frontRightTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(frontRightTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontRightTrig, LOW);

  distanceFR = pulseIn(frontRightEcho, HIGH) * 0.0133 / 2;

  // Return this distance value
  return distanceFR;

}
float readBDistance()
{
  digitalWrite(backTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(backTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(backTrig, LOW);

  distanceB = pulseIn(backEcho, HIGH) * 0.0133 / 2;

  // Return this distance value
  return distanceB;
}

void forward()
{

  Serial.println("Moving Forward");
  analogWrite(leftENA, leftSpeed);
  digitalWrite(leftIn1, LOW);
  digitalWrite(leftIn2, HIGH);
  analogWrite(leftENB, leftSpeed);
  digitalWrite(leftIn3, LOW);
  digitalWrite(leftIn4, HIGH);


  analogWrite(rightENA, rightSpeed);
  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, HIGH);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(rightIn3, LOW);
  digitalWrite(rightIn4, HIGH);
}

void backward()
{
  Serial.println("Moving Backward");

  analogWrite(leftENA, leftSpeed);
  digitalWrite(leftIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  analogWrite(leftENB, leftSpeed);
  digitalWrite(leftIn3, HIGH);
  digitalWrite(leftIn4, LOW);

  analogWrite(rightENA, rightSpeed);
  digitalWrite(rightIn1, HIGH); 
  digitalWrite(rightIn2, LOW);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(rightIn3, HIGH);
  digitalWrite(rightIn4, LOW);
}

void arcRight() // You will need to fill in this function
{
  double startingAngle = read_yaw();
  analogWrite(leftENA, leftSpeed); //going forwards
  digitalWrite(leftIn1, LOW); //backleft wheel doesnt spin
  digitalWrite(leftIn2, HIGH);
  analogWrite(leftENB, leftSpeed);
  digitalWrite(leftIn3, LOW);
  digitalWrite(leftIn4, HIGH);

  analogWrite(rightENA, rightSpeed); //going backwards
  digitalWrite(rightIn1, LOW); //front right wheel doesnt spin, will turn slower?
  digitalWrite(rightIn2, LOW);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(rightIn3, HIGH);
  digitalWrite(rightIn4, LOW);
  while ((abs(startingAngle - read_yaw())) < 77)
  {

  }
  stop();

}
void right() // You will need to fill in this function
{
  double startingAngle = read_yaw(); 
  analogWrite(leftENA, leftSpeed); //going forwards
  digitalWrite(leftIn1, LOW); //backleft wheel doesnt spin
  digitalWrite(leftIn2, HIGH);
  analogWrite(leftENB, leftSpeed);
  digitalWrite(leftIn3, LOW);
  digitalWrite(leftIn4, HIGH);

  analogWrite(rightENA, rightSpeed); //going backwards
  digitalWrite(rightIn1, HIGH); //front right wheel doesnt spin, will turn slower?
  digitalWrite(rightIn2, LOW);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(rightIn3, HIGH);
  digitalWrite(rightIn4, LOW);
  while ((abs(startingAngle - read_yaw())) < 79)
  {

  }
  stop();

}

void left() // You will need to fill in this function
{
  double startingAngle = read_yaw();
  analogWrite(leftENA, leftSpeed);
  analogWrite(leftENB, leftSpeed);
  analogWrite(rightENA, rightSpeed);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(leftIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn3, HIGH); 
  digitalWrite(leftIn4, LOW);


  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, HIGH);
  digitalWrite(rightIn3, LOW);
  digitalWrite(rightIn4, HIGH);
  while ((abs(startingAngle - read_yaw())) < 84.5)
  {

  }
  stop();

}

void correct() // You will need to fill in this function
{
  double startingAngle = read_yaw();
  analogWrite(leftENA, leftSpeed);
  analogWrite(leftENB, leftSpeed);
  analogWrite(rightENA, rightSpeed);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(leftIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn3, HIGH); 
  digitalWrite(leftIn4, LOW);


  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, HIGH);
  digitalWrite(rightIn3, LOW);
  digitalWrite(rightIn4, HIGH);
  while ((abs(startingAngle - read_yaw())) < 6)
  {

  }
  stop();

}
void arcLeft() // You will need to fill in this function
{
  double startingAngle = read_yaw();
  analogWrite(leftENA, leftSpeed);
  analogWrite(leftENB, leftSpeed);
  analogWrite(rightENA, rightSpeed);
  analogWrite(rightENB, rightSpeed);
  digitalWrite(leftIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn3, LOW); //front left doesnt spin
  digitalWrite(leftIn4, LOW);


  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, HIGH);
  digitalWrite(rightIn3, LOW);
  digitalWrite(rightIn4, HIGH);
  while ((abs(startingAngle - read_yaw())) < 106)
  {

  }
  stop();

}

void stop()
{
  // Print a message to the serial monitor saying that the rover has stopped
  Serial.println("Stopped");
  // Switch leftENA and rightENB to low so that all motors stop turning
  analogWrite(leftENA, LOW);
  analogWrite(leftENB, LOW);

  analogWrite(rightENA, LOW);
  analogWrite(rightENB, LOW);
}

float read_yaw() {
  previousTime = currentTime; // Use the time when read_yaw was last called as the previous time
  currentTime = millis(); // Update the value of the current time
  elapsedTime = (currentTime - previousTime) / 1000; // Find the elapsed time and divide by 1000 to get seconds

  Wire.beginTransmission(MPU); // Start Communication
  Wire.write(0x47); // Start with the first of the two registers that hold the GYRO_ZOUT data (There are GYRO_ZOUT_H and GYRO_ZOUT_L)
  Wire.endTransmission(false); // End the communication
  Wire.requestFrom(MPU, 2, true); // Read the values of the two bytes that contain the data we need
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // Combine the two bytes by left shifting the first 8 bits and adding on the second set of 8 bits
  // Correct the outputs with the calculated error values
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)

  // Find the angle in degrees by multiplying the angular velocity, which is in deg/s, by time to get the change in angle and then add that to the initial angular position
  yaw =  yaw + GyroZ * elapsedTime;

  // Print the values on the serial monitor
  return yaw;
}


float calculate_IMU_error() {
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU); // Start communication
    Wire.write(0x47); // Start with the first of the two registers that hold the GYRO_ZOUT data (There are GYRO_ZOUT_H and GYRO_ZOUT_L)
    Wire.endTransmission(false); // End the communication
    Wire.requestFrom(MPU, 2, true); // Read the values of the two bytes that contain the data we need
    GyroZ = Wire.read() << 8 | Wire.read(); // Combine the two bytes by left shifting the first 8 bits and adding on the second set of 8 bits
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0); // Sum the average of the readings so that they can be averaged
    c++;
  }
  GyroErrorZ = GyroErrorZ / 200;  // Average the gyro readings so that we know what the 0 point is
  return GyroErrorZ;
}
