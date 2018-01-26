/*

Michael Britt-Crane 2016.4.13
update to remote functionality. can now pause at any time (incuding during coasting) 
when paused light flashes purple then extinguishes
switched jog buttons from left & right arrows to up and down arrows

This program will run the stoc SeaGlide glider with no advanced add-ons. 
learn more at http://SeaGlide.net there you will find all of the source files, a bill of materials, instructions, and lessons. 
 
*/
//SEAGLIDE VARS
#include <IRremote.h>                // include the IRremote library: http://github.com/shirriff/Arduino-IRremote
#include <Servo.h>                   // include the stock "Servo" library for use in this sketch
Servo myservo;                       // create a new Servo object called myservo

// Constants
static int minCoast =  1000;         // if the pot is turned all the way to the counter-clockwise the glider will coast for 1 seccond
static int maxCoast = 20000;         // if the pot is turned all the way to the clockwise the glider will coast for 10 secconds
static byte servoDiveCommand = 0;    // this is the angle value that the dive method sends to the servo
static byte servoRiseCommand = 180;  // this is the angle value that the rise method sends to the servo
//static byte countsPrev = 6;          // 5 or 11
static int sampleInterval = 2000;       // sample interval if you have sensors attached. (not in use by default)

static int riseDriveTime = 12000;       // This variable determines the distance the plunger travels when pushing water out of the BE
                                        // Make adjustments only with a fully charged battery. The plunger should travel to the end of the syringe
static int pausedBlinkInterval = 1500;  // the pause time between short blinks when the buoyancy engine is paused 

// Pins 
static byte SERVO_PIN = 10;          // the pin that the "continuous rotation servo" is attached to, this motor drives the buoyancy engine
static byte DIVE_STOP = 11;          // the pin that the dive limmit switch (round push button) is attached to
static byte pausePin = 9; 
//static byte RISE_STOP_SENSOR = A0;   // the pin that the reflectance sensor is attached to. This sensor detects the edge of the plunger mass
static byte POT_PIN = A3;            // the pin that the wiper of the little orange trim pot is attached to
static byte RECV_PIN = 2;            // IR reciever signal pin
static byte encoderPin = 12;

static byte RED_LED = 9;             // these are the three pins that the RED
static byte GREEN_LED = 6;           //                                   GREEN
static byte BLUE_LED = 5;            //                               and BLUE LED cathodes are attached to
static byte LED_BASE = 7;            // this is the pin that the "common anode" of the RGB LED is attached to

// IR definitions
IRrecv irrecv(RECV_PIN);
decode_results results;
#define PAUSE 0xFD807F 
#define ONE 0xFD08F7
#define TWO 0xFD8877
#define THREE 0xFD48B7
#define UP 0xFDA05F
#define DOWN 0xFDB04F
#define LEFT 0xFD10EF
#define RIGHT 0xFD50AF
/*                                      not currently in use
#define FOUR 0xFD28D7
#define FIVE 0xFDA857
#define SIX 0xFD6897
#define SEVIN 0xFD18E7
#define EIGHT 0xFD9867
#define NINE 0xFD58A7
#define ENTER 0xFD906F
#define VOLUP 0xFD40BF
#define VLOLDOWN 0xFD00FF
*/

//RUDDER VARS
Servo myRudder;  // create servo object to control a servo

// GYROSCOPE VARS

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Define output form
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
float x = 0;
float y = 0;
float z = 0;
#define INTERRUPT_PIN 8  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


//SETUP
void setup() {                       // begin setup method
  //RUDER SETUP
  myRudder.attach(4);  // attaches the servo on pin 9 to the servo object
  //SEAGLIDE SETUP
  Serial.begin(115200);               // fire up the serial port. This allows us to print values to the serial console
 
  IRsetup();                         // Start the Infa-Red reciever
  pinMode(POT_PIN, INPUT);           // initialize the potentiometer, this pot will determine the coast time turn it right to coast longer
  pinMode(SERVO_PIN, OUTPUT);        // initialize the continuous rotation servo, this motor drives the buoyancy engine
  pinMode(DIVE_STOP, INPUT_PULLUP);  // initialize the dive stop switch, and turn on the internal pull-up resistor. This limmit switch is lets the Arduino know when the buoyancy engine reaches the end of its travle in the dive direction
  pinMode(pausePin, INPUT_PULLUP);   // initialize the dive stop switch, and turn on the internal pull-up resistor. This limmit switch is lets the Arduino know when the buoyancy engine reaches the end of its travle in the dive direction
  pinMode(RED_LED, OUTPUT);          // initialise the RED
  pinMode(GREEN_LED, OUTPUT);        //                GREEN
  pinMode(BLUE_LED, OUTPUT);         //            and BLUE pins on the LED
  pinMode(LED_BASE, OUTPUT);         // initialize the common pin of the LED 

  // initialize RGB LED
  ledRGB_Write(0, 0, 0);             // set the R, G, & B LEDs to OFF
  digitalWrite(LED_BASE, HIGH);      // set the LED Base pin to HIGH this LED it is a common anode, providing +5V to all 3 LEDs
  readPot(POT_PIN);
  delay(50);                        // wait for 0.2 sec

 
  //GYROSCOPE SETUP
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
   
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready ... dont want to require console input at this time
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  */
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}                                    // end setup method

// MAIN LOOP
void loop(){   
  //allow time to put in water
  delay(1000);
  //allow gyro to stabilize (millis is time since arduino began running)
  while(millis() < 2000){
    gyroScope();
  }          
  rudder(90); //rudder control
  dive(0);                     // DIVE-DIVE-DIVE: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
  pause(readPot(POT_PIN), 1);     // read the pot and delay bassed on it's position, coast
  rise(riseDriveTime); //150   // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  pause(readPot(POT_PIN)*1.1, 0); // Read the pot and delay bassed on it's position, coast     
} 
// END MAIN LOOP


// Dive: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
void dive(int time){                           
  ledRGB_Write(255, 0, 0);                      // set LED to RED to indicate that the glider is diving
  Serial.println("diving");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on "SERVO_PIN" to the servo object so that we can command the servo to turn
  myservo.write(servoDiveCommand);              // drive servo clockwise, take in water & pull weight forward (pull counterweight & plunger towards servo, at the bow of the glider)
  if (time == 0){
    while (digitalRead(DIVE_STOP) == HIGH){       // keep checking the DIVE_STOP pin to see if the button is pressed
      //gyro check
      gyroScope();
      //adjust rudder
      if (checkIR(0)){
        myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
        myservo.write(servoDiveCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
        ledRGB_Write(255, 0, 0);                      // set LED to RED to indicate that the glider is diving
      }
      // wait...                                  // just keep checking: when the button is pressed, continue to the next line
    }
  }
  else{
    unsigned long currentMillis = millis();
    long previousMillis = currentMillis;
    while (currentMillis - previousMillis < time && digitalRead(DIVE_STOP) ) { 
      //gyro check
      gyroScope();
      //adjust rudder
      currentMillis = millis();   
    }   
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  Serial.println("coasting (dive)");            // print status change to the serial port
  ledRGB_Write(255, 80, 0);                     // set LED to ORANGE to indicate that the glider is coasting in a dive
}                                               // end of method

// Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
void rise(int time){      // , byte cnts){                  
  ledRGB_Write(0, 200, 0);                      // set LED to GREEN to indicate that the glider is rising
  Serial.println("rising");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
  myservo.write(servoRiseCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
  unsigned long currentMillis = millis();
  long previousMillis = currentMillis;
  while (currentMillis - previousMillis < time) { 
    currentMillis = millis();  
    //gyro check
    gyroScope();
    //adjust rudder
    if (checkIR(0)){
      myservo.attach(SERVO_PIN);                // attaches the servo on SERVO_PIN to the servo object
      myservo.write(servoRiseCommand);          // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
      ledRGB_Write(0, 200, 0);                  // set LED to GREEN to indicate that the glider is rising
    }    
    // wait...                                  // just keep checking, when the sensor sees the edge, continue to the next line
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  Serial.println("coasting (rise)");            // print status change to the serial port
  ledRGB_Write(0, 0, 255);                      // set LED to BLUE to indicate that the glider is coasting in a rise
}                                               // end of method

//pause method
void pause(int pauseTime, boolean divingCoast){
  unsigned long currentMillis = millis();
  unsigned long previousMillis = currentMillis;
  unsigned long previousMillis2 = previousMillis;
  while (currentMillis - previousMillis < pauseTime) { 
    //gyro check
    gyroScope();
    //adjust rudder
    currentMillis = millis();  
    if (currentMillis - previousMillis2 > sampleInterval){
        //Put sensor print statements here
        previousMillis2 = currentMillis;
    }
    if(divingCoast){
      if (checkIR(1)){
        myservo.attach(SERVO_PIN);                // attaches the servo on SERVO_PIN to the servo object
        myservo.write(servoRiseCommand);          // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
        ledRGB_Write(0, 200, 0);                  // set LED to GREEN to indicate that the glider is rising
      }
    }else{ 
      if (checkIR(0)){
        myservo.attach(SERVO_PIN);                // attaches the servo on SERVO_PIN to the servo object
        myservo.write(servoRiseCommand);          // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
        ledRGB_Write(0, 200, 0);                  // set LED to GREEN to indicate that the glider is rising
      }
    }
  }//end while
}

// This method takes care of the details of setting a color and intensity of the RGB LED
void ledRGB_Write(byte R, byte G, byte B){      
  analogWrite(RED_LED, 255-R);                  // These are backwards because you write low values to turn these LEDs on
  analogWrite(GREEN_LED, 255-G);                // This method reverses the counterintuitive nature of the LEDs
  analogWrite(BLUE_LED, 255-B);                 // If using common anode rather than common anode LEDs remove the "255-"es
}                                               // end of method

// this method reads a potentiometer to determine the pause time
int readPot(int potPin){                        
  int potValue = analogRead(potPin);            // Read the Potentiometer
  int pauseTime = map(potValue, 0, 1023, minCoast, maxCoast); // scale the value to the diveDriveTime range defined by minDriveTime & maxDriveTime
  Serial.print("Coast Time: ");                 // print a lable to the serial port
  Serial.println(pauseTime);                    // print the pause time value to the serial port
  return pauseTime;                             // return the pause time, an intiger (int) value
}                                               // end of method

void checkReed(){
  while (!digitalRead(pausePin)){
    //pause...
    digitalWrite(13, 1);
  }
    digitalWrite(13, 0);
}

void IRsetup(){
  irrecv.enableIRIn();
}

void flashPurp(int t){
    ledRGB_Write(2, 0, 250);
    delay(t);
    ledRGB_Write(0, 0, 0);  
}

boolean checkIR(boolean coasting){
  if (checkPause()){
    myservo.detach();
    ledRGB_Write(0, 0, 0);  
    delay(150);
    flashPurp(200);
    boolean paused = true;
    unsigned long previousMillis = millis();
    while (paused){
      if (previousMillis+pausedBlinkInterval < millis()){
        flashPurp(50);    
        previousMillis = millis();
      }
      if (irrecv.decode(&results)) {
        if (results.value == PAUSE){
          Serial.println("PLAY");
          delay(100);
          flashPurp(50);
          paused = false;
        }
        if (results.value == UP){
          Serial.println("up");
          rise(700);          
          flashPurp(50);
        }
        irrecv.resume();
        if (results.value == DOWN){
          Serial.println("Left");
          dive(700);          
          flashPurp(50);
        }
        irrecv.resume();
        if (results.value == TWO){
          Serial.println("2");
          flash(2, 150);
          dive(0);
          delay(150);
          rise(riseDriveTime/2);
          flashPurp(50);
        }
        irrecv.resume();
      }      
    }
//    ledRGB_Write(60, 50, 50);
    if (coasting){
      Serial.println("paused durring coasting, Diving for safety");
      dive(0);                     // DIVE-DIVE-DIVE: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down    return true;
      pause(readPot(POT_PIN), 0);     // read the pot and delay bassed on it's position, coast
    }
    return true;
  }
  else{
    return false;  
  }
}

boolean checkPause(){
  if (irrecv.decode(&results)) {
    if (results.value == PAUSE) {
      Serial.println("PAUSE");
      irrecv.resume();
      return true;
    }
    else{
      irrecv.resume();
      return false;
    }
  }
}

void flash(int flashes, int time){
  for(int i = flashes; i > 0; i--){
    digitalWrite(13, 1);
    delay(time);
    digitalWrite(13, 0);
    delay(time);
  }
}

//Gyroscope main loop
void gyroScope(){
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
  //  while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
  //  }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            x = ypr[0] * 180/M_PI;
            y = ypr[1] * 180/M_PI;
            z = ypr[2] * 180/M_PI;
            Serial.print(x);
            Serial.print("\t");
            Serial.print(y);
            Serial.print("\t");
            Serial.println(z);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void rudder(int val){
  myRudder.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
/*
        if (results.value == ONE){
            Serial.println("1");
            flash(1, 150);
            dive(1000);
            rise(1);
        }
        if (results.value == TWO){
            Serial.println("2");
            flash(2, 150);
            rise(10);
            delay(150);
            dive(0);
            rise(riseDriveTime/2);
        }
        if (results.value == THREE){
            Serial.println("3");
            flash(3, 150);
            dive(0);
            delay(150);
            rise(1000);
        }
 */

