
// Rat Car
// For Elegoo Smart Robot Car v4 kit: Arduino Uno R3 and Elegoo SmartCar-Shield-V1.1 with TB6612 motor driver, HC-SR04 ultrasonic distance sensor and MPU6050 inertial sensor module

// Based on the design at https://www.instructables.com/Rat-Operated-Vehicle/ but the code it links to at https://github.com/LElizabethCrawford/RatCarCode/blob/master/Rat_Car_Code.ino
// was for a V3 Elegoo Smart Car Kit and has been rewritten here to work with the V4 kit as currently sold in 2025.

// The code here replaces all the default Elegoo programs for the Smart Car V4 kit. The line tracking sensor is not used and need not be fitted when building the kit as it's
// cable is repurposed as described below to access the analog input pins for the drivers controls, the inertial sensor may be fitted but is not currently used in this code,
// the ultrasonic distance sensor is used (but without using the servo motor to turn it to point in different directions, it is simply left pointing straight ahead) for collision
// avoidance but this could be easily disabled by simply setting OBSTACLE_AVOIDANCE_DISTANCE to 0 in the "USER CONFIGURABLE VALUES" section below or commenting out or removing the 
// call to getObstacleDistance() in the main "loop()" section of the code.

// I recommend tapping into the A0, A1 and A2 pins and a +5v supply for the floor plate by cutting and repurposing the cable for the line tracking module and soldering it to a small piece of 
// stripboard or similar to hold the 3x 1M Ohm resistors required. I then soldered dupont wires to the other sides of the resistors to allow for easy connection/disconnection of the drivers 
// controls and compartment.

// For the Arduino IDE setup, install “Smartcar shield” by Dimitris Platis from the Library Manager tab in the IDE, this will also install it’s dependencies 
// which are “Servo” by Michael Margolis, “ESP32 AnalogWrite” by ERROPiX and “ServoESP32” by Jaroslav Paral and provide all the necessary libraries to compile and flash this code.
// Select the USB to serial port your Arduino shows up as (eg. /dev/tty.usbserial-01 or similar) and choose "Arduino Uno" as the board type for an Uno R3 as supplied with the Elegoo kit V4.

// The speeds defined here (MAXSPEED values) seem sufficient for a medium to large rat without being too fast for initial training, but you may need to increase them if the car doesn't
// have enough power to move away from a standing start with heavier rats or once the driver has more experience and can safely control the car at higher speeds.

// The various Serial.print("Something: "); and Serial.println(SomeVariable); calls have been left in place but commented out so they can be easily uncommented for initial debugging and
// calibration using the serial monitor to see what values are being reported.

// The controlThrehold chosen may be quite individual to your build of the car, I found that the values detected when not touching the control wires can be significantly affected by
// noise from the USB connection, and also by what was connected to the control side of the circuit (ie just the dupont wires, or them plus the actual control wires the rat uses to drive)
// so although using the serial monitor is helpful to find an approximate starting point for controlThresold it will also be necessary to test the final configuration and iteratively tweak 
// the controlThrehold value to be as low as possible so the controls are sensitive and always respond to being touched, whilst also setting the value high enough to avoid uncommanded motion.

// loopTime is somewhat arbitrarily set to 100ms here but it seems about right, this value will interact with the ACCELERATION value chosen, as each iteration through the control loop
// once every loopTime will result in a change of ACCELERATION in the commanded speed, so a shorter loopTime will increase the speed at which the car accelerates and decelerates, as will
// an increase ACCELERATION value.

// This version of the code performs rotation movements (when one of the L/R direction controls is activated without forward motion being commanded) with an overall slight backwards motion,
// to facilitate the rats turning/reversing away from an obstacle that they have driven up against preventing an on-the-spot rotation. The movestate 3 and movestate 5 code can be seen to still
// command a forward movement of the wheels on the side of the car opposite the rotation direction, but since it is slower than the backward motion of the wheels on the side of the rotation
// direction the overall effect is a slightly backwards motion during the rotation which is sufficient to move away from even an obstacle touching the front of the car.

// Paul Bryden-Bradley


// -----------------------------------------------------------------------------------------------------------------------------------
// USER CONFIGURABLE VALUES SECTION

// Control sensitivity:
int controlThreshold = 150;  // minimum ADC reading for touch sensor to activate, around 150 seems sufficient
                             // increase if uncommanded motions occur and decrease if controls are too insensitive

// Acceleration and turn sharpness
int ACCELERATION = 10;  // 1 feels laggy, 50 feels lurchy.
int TURN_OFFSET = 30;  // Adjust how sharp it turns, about a third of MAXSPEED is the maximum practical value before the
                       // differences between left and right motors begin to make one turn direction sharper than the other

// Max speed of wheels during forward or turning motion, defined for each side separately to allow correcting uncommanded turns
// Adjust both values together proportionately to increase or decrease the maximum overall speed
int MAXSPEED_LEFTWHEELS = 120;     // Maximum speed of left wheels PWM min-max = 0-255
int MAXSPEED_RIGHTWHEELS = 120;     // Maximum speed of right wheels PWM min-max = 0-255

// Max speed of wheels during rotating motion, defined for each side separately to allow correcting for differences between motors
// Adjust both values together proportionately to increase or decrease the maximum overall speed of rotating motion
int MAXSPEED_LEFTWHEELS_ROTATION = 100;     // Maximum speed of left wheels during rotation, PWM min-max = 0-255
int MAXSPEED_RIGHTWHEELS_ROTATION = 100;     // Maximum speed of right wheels during rotation, PWM min-max = 0-255

// Obstacle avoidance 
#define OBSTACLE_AVOIDANCE_DISTANCE 20 // Distance to an obstacle (in centimeters) below which we initiate obstacle avoidance measures
                                       // If we are closer to an obstacle than this distance, only rotating motion is allowed

// Time between control loop iterations
int loopTime = 100; // Time in milliseconds between iterations of the main control loop that accelerates/decelerates the car.

// END OF USER CONFIGURABLE VALUES SECTION
// -----------------------------------------------------------------------------------------------------------------------------------


// -----------------------------------------------------------------------------------------------------------------------------------
// PIN DEFINITIONS AND VARIABLE DECLARATIONS

//TB6612 with pins renamed from SRC4.0 DeviceDriverSet_xxx0.h
#define PWM_RightWheels 5        // Controls PWM speed of right wheels, range 0-255
#define PWM_LeftWheels 6         // Controls PWM speed of left wheels, range 0-255
#define Direction_LeftWheels 8   // Controls direction of left motors, HIGH = FORWARD, LOW = REVERSE
#define Direction_RightWheels 7  // Controls direction of right motors, HIGH = FORWARD, LOW = REVERSE
#define Motor_STBY 3             // Controls motor standby, HIGH = Motors Enabled, LOW = Motors Disabled

//HC-SR04 ultrasonic distance sensor
#define ULTRASONIC_TRIG_PIN 13      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ULTRASONIC_ECHO_PIN 12      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ULTRASONIC_MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//Mode switch push button on Elegoo Smart Car Shield
#define modeSwitch 2             // Mode Switch input

// Control inputs
const byte forwardInputPin = A0;     // the forward touch sensor
const byte leftInputPin = A1;     // the left touch sensor
const byte rightInputPin = A2;     // the right touch sensor

// Decaration of variables

int forwardInputVal = 0;
int leftInputVal = 0;
int rightInputVal = 0;

int CURRENTSPEED_LEFTWHEELS = 0;   // Current speed of left wheels PWM min-max = 0-255
int CURRENTSPEED_RIGHTWHEELS = 0;   // Current speed of right wheels PWM min-max = 0-255
int TARGETSPEED_LEFTWHEELS = 0;   // Target speed of left wheels PWM min-max = 0-255
int TARGETSPEED_RIGHTWHEELS = 0;   // Target speed of right wheels PWM min-max = 0-255

int moveState = 0;    // 0:stop, 1: forward, 2: turn left, 3: rotate left, 4: turn right, 5: rotate right.
int oldMoveState = 0;

int obstacleDistance = 0; // Ultrasonic sensed distance to obstacle

bool LEFT_FORWARD = true;
bool RIGHT_FORWARD = true;

// END OF PIN DEFINITIONS AND VARIABLE DECLARATIONS
// -----------------------------------------------------------------------------------------------------------------------------------


// Init function that runs once at power on
void setup() {
  
  Serial.begin(9600);  // Serial monitor

  // Setup motor IO pins  
  pinMode(PWM_RightWheels, OUTPUT);
  pinMode(PWM_LeftWheels, OUTPUT);
  pinMode(Direction_LeftWheels, OUTPUT);
  pinMode(Direction_RightWheels, OUTPUT);
  pinMode(Motor_STBY, OUTPUT);

  // Setup ultrasonic sensor IO pins
  pinMode(ULTRASONIC_ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);

  // Enable and init the motors
  digitalWrite(Motor_STBY, HIGH);  //Enable Motors to run
  digitalWrite(Direction_LeftWheels,HIGH); // Set left wheels to go forwards initially
  digitalWrite(Direction_RightWheels,HIGH); // Set right wheels to go forwards initially
  
  // Set motors to zero speed initially
  digitalWrite(PWM_RightWheels, LOW);   // Set zero speed on right wheels
  digitalWrite(PWM_LeftWheels, LOW);   // Set zero speed on left wheels

  // Wait for mode switch to be pressed for testing
  // while (digitalRead(modeSwitch) == 1) ; // Do nothing until mode switch is pressed and state becomes 0


}

// Main loop
void loop() {

  getObstacleDistance();

  moveState = checkInputs(); // 0: stop, 1: forward, 2: turn left, 3: rotate left, 4: turn right, 5: rotate right.

  if(obstacleDistance <= OBSTACLE_AVOIDANCE_DISTANCE && moveState == 1){ // if moveState 1 (forward) is active but we are near to an obstacle
    // Serial.print("Stopped from forward motion due to obstacleDistance ");
    // Serial.print(obstacleDistance);
    // Serial.println("cm");
    moveState = 0; // Stop
  } else if (obstacleDistance <= OBSTACLE_AVOIDANCE_DISTANCE && moveState == 2){ // if moveState 2 (turn left) is active but we are near to an obstacle
    // Serial.print("Stopped from turn left motion due to obstacleDistance ");
    // Serial.print(obstacleDistance);
    // Serial.println("cm");
    moveState = 0; // Stop
  } else if (obstacleDistance <= OBSTACLE_AVOIDANCE_DISTANCE && moveState == 4){ // if moveState 4 (turn right) is active but we are near to an obstacle
    // Serial.print("Stopped from turn right motion due to obstacleDistance ");
    // Serial.print(obstacleDistance);
    // Serial.println("cm");
    moveState = 0; // Stop
  } 
  else 
  {    
    // If obstacle avoidance not triggered leave moveState at whatever checkInputs() returned 
  }

  // Serial.print("moveState ");
  // Serial.println(moveState);

  if (oldMoveState != moveState)
    setTarget(moveState);

  move();

  oldMoveState = moveState;

  delay(loopTime); // Delay loopTime ms until checking the inputs and updating the target speeds again

}

// Function to check the control inputs, returns moveState - 0: stop, 1: forward, 2: turn left, 3: rotate left, 4: turn right, 5: rotate right.
int checkInputs()
{
  leftInputVal = analogRead(leftInputPin);
  delay(10);
  leftInputVal = analogRead(leftInputPin);

  forwardInputVal = analogRead(forwardInputPin);
  delay(10);
  forwardInputVal = analogRead(forwardInputPin);

  rightInputVal = analogRead(rightInputPin);
  delay(10);
  rightInputVal = analogRead(rightInputPin);

//  Serial.print("forward: ");
//  Serial.println(forwardInputVal);
//  
//  Serial.print("left: ");
//  Serial.println(leftInputVal);
//  
//  Serial.print("right: ");
//  Serial.println(rightInputVal);
//  delay(250);

  if(leftInputVal > controlThreshold && forwardInputVal > controlThreshold){
    moveState = 2;
  } else if (rightInputVal > controlThreshold && forwardInputVal > controlThreshold){
    moveState = 4;
  } else if (leftInputVal > controlThreshold){
    moveState = 3;
  } else if (rightInputVal > controlThreshold){
    moveState = 5;
  } else if (forwardInputVal > controlThreshold){
    moveState = 1;
  } else {
    moveState = 0;  // Touching none is stop.
  }

  //  Serial.println(moveState);

  return (moveState);
}

// Function to set the target speed of the wheels - accepts the desired moveState as an argument, 0: stop, 1: forward, 2: turn left, 3: rotate left, 4: turn right, 5: rotate right.
void setTarget(int)
{
  switch(moveState){
    case 0:
      //Serial.println("moveState 0 (stop) activated in function setTarget");
      TARGETSPEED_LEFTWHEELS = 0;
      TARGETSPEED_RIGHTWHEELS = 0;
      break;
    
    case 1:
      //Serial.println("moveState 1 (forward) activated in function setTarget");
      TARGETSPEED_LEFTWHEELS = MAXSPEED_LEFTWHEELS;
      TARGETSPEED_RIGHTWHEELS = MAXSPEED_RIGHTWHEELS;
      break;

   case 2:
      //Serial.println("moveState 2 (turn left) activated in function setTarget");
      TARGETSPEED_LEFTWHEELS = MAXSPEED_LEFTWHEELS - TURN_OFFSET;
      TARGETSPEED_RIGHTWHEELS = MAXSPEED_RIGHTWHEELS;
      break;

   case 3:
      //Serial.println("moveState 3 (rotate left) activated in function setTarget");
      TARGETSPEED_LEFTWHEELS = MAXSPEED_LEFTWHEELS_ROTATION * -1;
      TARGETSPEED_RIGHTWHEELS = MAXSPEED_RIGHTWHEELS_ROTATION * 0.5;
      break;

   case 4:
      //Serial.println("moveState 4 (turn right) activated in function setTarget");
      TARGETSPEED_LEFTWHEELS = MAXSPEED_LEFTWHEELS;
      TARGETSPEED_RIGHTWHEELS = MAXSPEED_RIGHTWHEELS - TURN_OFFSET;
      break;

   case 5:
      //Serial.println("moveState 5 (rotate right) activated in function setTarget");
      TARGETSPEED_LEFTWHEELS = MAXSPEED_LEFTWHEELS_ROTATION * 0.5;
      TARGETSPEED_RIGHTWHEELS = MAXSPEED_RIGHTWHEELS_ROTATION * -1;
      break;
  }
}

// Function to move the car
void move()
{
  // Accelerate or decelerate the right wheels towards their target speed
  if(CURRENTSPEED_RIGHTWHEELS < TARGETSPEED_RIGHTWHEELS){
    CURRENTSPEED_RIGHTWHEELS +=ACCELERATION;
  } 
  if(CURRENTSPEED_RIGHTWHEELS > TARGETSPEED_RIGHTWHEELS){
    CURRENTSPEED_RIGHTWHEELS -=ACCELERATION;
  }
  // Accelerate or decelerate the left wheels towards their target speed
  if(CURRENTSPEED_LEFTWHEELS < TARGETSPEED_LEFTWHEELS){
    CURRENTSPEED_LEFTWHEELS +=ACCELERATION;
  }
  if(CURRENTSPEED_LEFTWHEELS > TARGETSPEED_LEFTWHEELS){
    CURRENTSPEED_LEFTWHEELS -=ACCELERATION;
  }

  // If necessary, switch the left wheels direction by setting state of Direction_LeftWheels pin, HIGH = FORWARD, LOW = REVERSE
  if(CURRENTSPEED_LEFTWHEELS < 0 && LEFT_FORWARD){ // If LEFT_FORWARD is true (left wheels are going forwards) and the new commanded speed for the left wheels is negative set them to go backwards and update the LEFT_FORWARD boolean to match
      digitalWrite(Direction_LeftWheels,LOW); // Set left wheels to go backwards
      LEFT_FORWARD = false;   
  }
  if(CURRENTSPEED_LEFTWHEELS > 0 && !LEFT_FORWARD){ // If LEFT_FORWARD is false (left wheels are going backwards) and the new commanded speed for the left wheels is positive set them to go forwards and update the LEFT_FORWARD boolean to match
      digitalWrite(Direction_LeftWheels,HIGH); // Set left wheels to go forwards
      LEFT_FORWARD = true; 
  }

  // If necessary, switch the right wheels direction by setting state of Direction_RightWheels pin, HIGH = FORWARD, LOW = REVERSE
  if(CURRENTSPEED_RIGHTWHEELS < 0 && RIGHT_FORWARD){ // If RIGHT_FORWARD is true (right wheels are going forwards) and the new commanded speed for the right wheels is negative set them to go backwards and update the RIGHT_FORWARD boolean to match
      digitalWrite(Direction_RightWheels,LOW); // Set right wheels to go backwards
      RIGHT_FORWARD = false;   
  }
  if(CURRENTSPEED_RIGHTWHEELS > 0 && !RIGHT_FORWARD){ // If RIGHT_FORWARD is false (right wheels are going backwards) and the new commanded speed for the right wheels is positive set them to go forwards and update the RIGHT_FORWARD boolean to match
      digitalWrite(Direction_RightWheels,HIGH); // Set right wheels to go forwards
      RIGHT_FORWARD = true;
  }

  // Write the new commanded speeds to the PWM output pins
  analogWrite(PWM_LeftWheels, abs(CURRENTSPEED_LEFTWHEELS)); 
  analogWrite(PWM_RightWheels, abs(CURRENTSPEED_RIGHTWHEELS));

  // Serial.print("Left wheels forward:");
  // Serial.println(LEFT_FORWARD);      
  // Serial.print("Right wheels forward:");
  // Serial.println(RIGHT_FORWARD);
    

  // Serial.print("Left wheels speed:");
  // Serial.println(CURRENTSPEED_LEFTWHEELS);
  // Serial.print("Right wheels speed:");
  // Serial.println(CURRENTSPEED_RIGHTWHEELS);
}

// Function to poll the ultrasonic sensor and return the distance to the nearest obstacle
void getObstacleDistance()
{
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  obstacleDistance = ((unsigned int)pulseIn(ULTRASONIC_ECHO_PIN, HIGH) / 58);
  if (obstacleDistance > ULTRASONIC_MAX_DISTANCE)
  {
    obstacleDistance = ULTRASONIC_MAX_DISTANCE;
  }

  //  Serial.print(obstacleDistance);
  //  Serial.println("cm");

  return (obstacleDistance);
}
