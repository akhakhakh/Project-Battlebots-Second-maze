// Line sensors
#define NUM_SENSORS 8  
int _sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int _sensorValues[NUM_SENSORS];
bool _whereIsLine[NUM_SENSORS];  
int _black = 0;

// Motor pins
#define MOTORA_PIN1 11 
#define MOTORA_PIN2 10
#define MOTORB_PIN1 9
#define MOTORB_PIN2 8

// Define gripper pin
#define GRIPPER 5
#define MOTOR_SPEED 180

#define TIME_OPEN 1550
#define TIME_OPENMAX 1800
#define TIME_CLOSE 975

// Rotation sensor
#define SENSOR_ROTATION1 12  
#define SENSOR_ROTATION2 13 

// variables
const int _sensorDataInterval = 500;
int _timerColor = 0;

bool _startedRace = false;


int _rotationsSr1, _rotationsSr2 = 0;

// Interrupts for rotation sensors
void count_RotationsSensor1() 
{
  _rotationsSr1++;
}

void count_RotationsSensor2() 
{
  _rotationsSr2++;
}

// Motor control functions
void moveForward() 
{
    digitalWrite(MOTORA_PIN1, 0);
    digitalWrite(MOTORA_PIN2, MOTOR_SPEED);
    digitalWrite(MOTORB_PIN1, MOTOR_SPEED);
    digitalWrite(MOTORB_PIN2, 0);
}

void turnRight()
{
  analogWrite(MOTORA_PIN1, MOTOR_SPEED);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, MOTOR_SPEED);
  analogWrite(MOTORB_PIN2, 0);
}

//Stops the right wheel
void rightStop() 
{
  digitalWrite(MOTORB_PIN1, HIGH);
  digitalWrite(MOTORB_PIN2, HIGH);
}

//Stops the left wheel
void leftStop() 
{
  digitalWrite(MOTORA_PIN1, HIGH);
  digitalWrite(MOTORA_PIN2, HIGH);
}

//Stops both wheels
void allStop() 
{
  leftStop();
  rightStop();
}

//Makes the left wheel go forward
void leftForward(int speed) 
{
  analogWrite(MOTORA_PIN2, speed);
  analogWrite(MOTORA_PIN1, 0);
}

//Makes the right wheel go forward
void rightForward(int speed) 
{
  analogWrite(MOTORB_PIN1, speed);
  analogWrite(MOTORB_PIN2, 0);
}

//Makes the right wheel go backwards
void rightBackward(int speed) 
{
  analogWrite(MOTORB_PIN2, speed);
  analogWrite(MOTORB_PIN1, 0);
}

//Makes the left wheel go backwards
void leftBackward(int speed) 
{
  analogWrite(MOTORA_PIN1, speed);
  analogWrite(MOTORA_PIN2, 0);
}
// Sets the speed for how fast the right motor should be going depending on certain conditions
void setRightMotor(int speed) 
{
  if (speed > 0) 
  {
    rightForward(speed);
  } 
  else if (speed < 0) 
  {
    rightBackward(speed * -1);
  } 
  else 
  {
    rightStop();
  }
}
// Sets the speed for how fast the left motor should be going depending on certain conditions
void setLeftMotor(int speed) 
{
  if (speed > 0) 
  {
    leftForward(speed);
  } 
  else if (speed < 0) 
  {
    speed = speed * -1;
    leftBackward(speed);
  } 
  else 
  {
    leftStop();
  }
}
//Sets the speed of both motors
void setBothMotor(int speed) 
{
  setLeftMotor(speed);
  setRightMotor(speed);
}
//Sets the speed of both motors based on different speeds
void setMotors(int speed1, int speed2)
{
  setLeftMotor(speed1);
  setRightMotor(speed2);
}

void Gripper(int pulse) 
{
    digitalWrite(GRIPPER, HIGH);  // Activate gripper
    delayMicroseconds(pulse);     // Hold for the given pulse duration
    digitalWrite(GRIPPER, LOW);   // Deactivate gripper
    delay(20);                    // Small delay to ensure signal is processed
}
// Line following functions
void calibrate() 
{
  delay(2000);
  // Use Analog 4 pin for the calibration
  _black = analogRead(A4) - 150;
  
  if (millis() - _sensorDataInterval >= _timerColor) 
  {
    _timerColor = millis();
  }
}
//reads the black color
void read_color() 
{
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    _sensorValues[i] = analogRead(_sensorPins[i]);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(_sensorValues[i]);
  }
}
//reads the white color
void read_bool_color() 
{
  Serial.print("Line detected: ");
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    _whereIsLine[i] = analogRead(_sensorPins[i]) > _black;
    Serial.print(_whereIsLine[i]);
    Serial.print(" ");
  }
  Serial.println();
}
//main function for following the lines
void followLine() 
{
  

  read_bool_color();

  //Main maze-solving logic - Most important rule of solving mazes is to always go one direction when presented with a branching path
  if (_whereIsLine[3] || _whereIsLine[4]) 
  {
    //only one direction
    if (_whereIsLine[7]) 
    { 
      setMotors(-220, 200);
    }
    //keep moving forward
    else
    {
      setBothMotor(200);
    } 
  }
  else if (_whereIsLine[5] || _whereIsLine[6]) 
  { // Line is slightly to the right
    setMotors(-130, 210);
  }
  else if (_whereIsLine[7]) 
  { // Line is far right
    setMotors(-230, 200);
  }
  else if (_whereIsLine[2] || _whereIsLine[1]) 
  { // Line is slightly to the left
    setMotors(210, -130);
  }
  else if (_whereIsLine[0]) 
  { // Line is far left
    setMotors(200, -230);
  }
  else if (_whereIsLine[0] && _whereIsLine[1] && _whereIsLine[2] && _whereIsLine[3] && _whereIsLine[4] && _whereIsLine[5] && _whereIsLine[6] && _whereIsLine[7])
  {
    allStop();
  }
  else if (!_whereIsLine[0] && !_whereIsLine[1] && !_whereIsLine[2] && !_whereIsLine[3] && !_whereIsLine[4] && !_whereIsLine[5] && !_whereIsLine[6] && !_whereIsLine[7])
  {
    // Line detected by no sensors
    setMotors(170,-170);
  }
}

void setup() 
{

  // motor as outputs
  pinMode(MOTORA_PIN1, OUTPUT);
  pinMode(MOTORA_PIN2, OUTPUT);
  pinMode(MOTORB_PIN1, OUTPUT);
  pinMode(MOTORB_PIN2, OUTPUT);
  pinMode(GRIPPER, OUTPUT);
  digitalWrite(GRIPPER, LOW);
  Serial.begin(9600);
  
  // rotation sensor as inputs
  pinMode(SENSOR_ROTATION1, INPUT);
  pinMode(SENSOR_ROTATION2, INPUT);
  
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(SENSOR_ROTATION1), count_RotationsSensor1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_ROTATION2), count_RotationsSensor2, FALLING);
  
  // Initialize and calibrate
  allStop();  // Stop motors before calibration
  calibrate();  // Set _black color threshold
}

void loop() 
{
  while (!_startedRace) 
  {
    delay(100);
    Gripper(TIME_OPEN);
    delay(100);
    moveForward();
    delay(1000);
    allStop();
    calibrate();
    Gripper(TIME_CLOSE);
    turnRight();
    delay(600);
    _startedRace = true;
  }
  // Main line following logic
  followLine();
}