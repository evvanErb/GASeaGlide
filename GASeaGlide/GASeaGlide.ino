/*

Michael Britt-Crane 2016.4.13
update to remote functionality. can now pause at any time (incuding during coasting) 
when paused light flashes purple then extinguishes
switched jog buttons from left & right arrows to up and down arrows

This program will run the stoc SeaGlide glider with no advanced add-ons. 
learn more at http://SeaGlide.net there you will find all of the source files, a bill of materials, instructions, and lessons. 
 
*/

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
static byte IR_GND = 3;              // IR middle pin, ground
static byte IR_PWR = 4;              // IR power pin
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

void setup() {                       // begin setup method
  Serial.begin(9600);                // fire up the serial port. This allows us to print values to the serial console
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
}                                    // end setup method

void loop(){                   // begin main loop
  dive(0);                     // DIVE-DIVE-DIVE: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
  pause(readPot(POT_PIN), 1);     // read the pot and delay bassed on it's position, coast
  rise(riseDriveTime); //150   // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  pause(readPot(POT_PIN)*1.1, 0); // Read the pot and delay bassed on it's position, coast
}                              // end main loop

void dive(int time){                                    // Dive: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
  ledRGB_Write(255, 0, 0);                      // set LED to RED to indicate that the glider is diving
  Serial.println("diving");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on "SERVO_PIN" to the servo object so that we can command the servo to turn
  myservo.write(servoDiveCommand);              // drive servo clockwise, take in water & pull weight forward (pull counterweight & plunger towards servo, at the bow of the glider)
  if (time == 0){
    while (digitalRead(DIVE_STOP) == HIGH){       // keep checking the DIVE_STOP pin to see if the button is pressed
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
      currentMillis = millis();   
    }   
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  Serial.println("coasting (dive)");            // print status change to the serial port
  ledRGB_Write(255, 80, 0);                     // set LED to ORANGE to indicate that the glider is coasting in a dive
}                                               // end of method


void rise(int time){      // , byte cnts){                  // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  ledRGB_Write(0, 200, 0);                      // set LED to GREEN to indicate that the glider is rising
  Serial.println("rising");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
  myservo.write(servoRiseCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
  unsigned long currentMillis = millis();
  long previousMillis = currentMillis;
  while (currentMillis - previousMillis < time) { 
    currentMillis = millis();  
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

void pause(int pauseTime, boolean divingCoast){
  unsigned long currentMillis = millis();
  unsigned long previousMillis = currentMillis;
  unsigned long previousMillis2 = previousMillis;
  while (currentMillis - previousMillis < pauseTime) { 
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

void ledRGB_Write(byte R, byte G, byte B){      // This method takes care of the details of setting a color and intensity of the RGB LED
  analogWrite(RED_LED, 255-R);                  // These are backwards because you write low values to turn these LEDs on
  analogWrite(GREEN_LED, 255-G);                // This method reverses the counterintuitive nature of the LEDs
  analogWrite(BLUE_LED, 255-B);                 // If using common anode rather than common anode LEDs remove the "255-"es
}                                               // end of method

int readPot(int potPin){                        // this method reads a potentiometer to determine the pause time
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
  pinMode(IR_GND, OUTPUT);
  pinMode(IR_PWR, OUTPUT);
  digitalWrite(IR_GND, 0);
  digitalWrite(IR_PWR, 1);
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

