//for seervo
#include <ServoTimer2.h>
//for the button
#include <ezButton.h>
// for the sound board
#include <Adafruit_Soundboard.h>

#include <SoftwareSerial.h>

// constants won't change
const int BUTTON_PIN = 2; // Arduino pin connected to button's pin
const int SERVO_PIN = 3; // Arduino pin connected to servo's pin
const int GreenLED_PIN    = 4; // Arduino pin connected to GREEN LED's pin
const int RedLED_PIN = 5; // Arduino Pin connected to RED LED's pin
const int YellowLED1_PIN = 6; // Arduino Pin connected to Yellow LED's pin
const int YellowLED2_PIN = 7; // Arduino Pin connected to Yellow LED's pin
const int YellowLED3_PIN = 8; // Arduino Pin connected to Yellow LED's pin

const byte ledPinsArray[] = {6,7,8};
const unsigned long oneSecond=1000;
byte LEDcount;

ServoTimer2 servo;

// soundboard pins and setup
// Reset pin
#define SFX_RST 9
// Receive pin
#define SFX_RX 10
// Send pin
#define SFX_TX 11
// Busy pin
const int ACT = 12;    // this allows us to know if the audio is playing

SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);
Adafruit_Soundboard sfx = Adafruit_Soundboard( &ss, NULL, SFX_RST);

// variables will change:
int closedAngle = 750; //closed angle
int openAngle = 1500; //open angle
int angle = (closedAngle);  // the current angle of servo motor

int greenledState = LOW;     // the current state of LED
int redledState = HIGH;
int yellowledState = HIGH;

// audio track names on soundboard
char TrapTrack[] =   "T01     WAV";


// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long currentTime = millis();
unsigned long elapsedTime = 0;
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long previousMillis = 0;
unsigned long previousRedMillis = 0;        // will store last time LED was updated
unsigned long previousYellowMillis = 0;        // will store last time LED was updated

// constants won't change:
const long redinterval = 200;           // interval at which to blink (milliseconds)
const long yellowinterval = 400;           // interval at which to blink (milliseconds)

int buttonState = 0;         // variable for reading the pushbutton status

// Track to play
uint8_t n = 0; //0

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize serial
  Serial.begin(115200);
  Serial.println("Adafruit Sound Board!");
  
  // softwareserial at 9600 baud
  ss.begin(9600);
  // can also do Serial1.begin(9600)

  if (!sfx.reset()) {
    Serial.println("Not found");
    while (1);
  }
  Serial.println("SFX board found");  
 
  servo.attach(SERVO_PIN);          // attaches the servo to the servo object
  pinMode(GreenLED_PIN, OUTPUT);        // set arduino pin to output mode
  pinMode(RedLED_PIN, OUTPUT);          // set arduino pin to output mode
  pinMode(YellowLED1_PIN, OUTPUT);       // set arduino pin to output mode
  pinMode(YellowLED2_PIN, OUTPUT);       // set arduino pin to output mode
  pinMode(YellowLED3_PIN, OUTPUT);       // set arduino pin to output mode

  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);
  //button.setDebounceTime(50);       // set debounce time to 50 milliseconds
 
  servo.write(closedAngle); 
  digitalWrite(GreenLED_PIN, greenledState);
  digitalWrite(RedLED_PIN, redledState);
  digitalWrite(YellowLED1_PIN, yellowledState);
  digitalWrite(YellowLED2_PIN, yellowledState);
  digitalWrite(YellowLED3_PIN, yellowledState);
}
void loop() {

      // Notice that none of the action happens in loop() apart from reading millis()
      //   it just calls the functions that have the action code

  currentMillis = millis();   // capture the latest value of millis()
                              //   this is equivalent to noting the time from a clock
                              //   use the same time for all LED flashes to keep them synchronized
  
             // call the functions that do the work
  // buttonloop();
   blinkRed();
   blinkYellow();
   playaudio();
}
void buttonloop() {
buttonState = digitalRead(BUTTON_PIN);
   if (buttonState == HIGH) {
    Serial.println("The button is pressed");

        // change angle of servo motor
    if(angle == (closedAngle))
      angle = (openAngle);
     
    else
    if(angle == (openAngle))
      angle = (closedAngle);

    servo.write(angle); // control servo motor arccoding to the angle
    
    // toggle state of LED
    greenledState = !greenledState;
    // control LED arccoding to the toggled state
    digitalWrite(GreenLED_PIN, greenledState); 
    
   }
    
 }  
void blinkRed() {
      unsigned long currentMillis = millis();

    if (greenledState == HIGH) {
      if (currentMillis - previousRedMillis >= redinterval) {
    // save the last time you blinked the LED
    previousRedMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (redledState == LOW) {
      redledState = HIGH;
    } else {
      redledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(RedLED_PIN, redledState);
      } 
     }
     else{
      digitalWrite(RedLED_PIN,HIGH);}
}

void blinkYellow() {
  currentTime = millis();
  elapsedTime = currentMillis - previousMillis;
  
      unsigned long currentMillis = millis();

    if (greenledState == HIGH) {
      if (currentMillis - previousYellowMillis >= yellowinterval) {
    // save the last time you blinked the LED
    previousYellowMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (yellowledState == LOW) {
      yellowledState = HIGH;
    } else {
      yellowledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(YellowLED1_PIN, yellowledState);
    digitalWrite(YellowLED2_PIN, yellowledState);
    digitalWrite(YellowLED3_PIN, yellowledState);
      } 
     }
  
  
  else if (elapsedTime >= oneSecond){
    previousMillis = previousMillis + oneSecond;
    LEDcount = LEDcount +1;
    if (LEDcount == 4){
    LEDcount = 0;
    digitalWrite (ledPinsArray[0], LOW); // assume Low = off, High = on
    digitalWrite (ledPinsArray[1], LOW);
    digitalWrite (ledPinsArray[2], LOW);
  }
// crude method:
    if (LEDcount == 1){
    digitalWrite (ledPinsArray[0], HIGH);
    digitalWrite (ledPinsArray[1], LOW);
    digitalWrite (ledPinsArray[2], LOW);
  }
    if (LEDcount == 2){
    digitalWrite (ledPinsArray[0], HIGH);
    digitalWrite (ledPinsArray[1], HIGH);
    digitalWrite (ledPinsArray[2], LOW);
  }
    if (LEDcount == 3){
    digitalWrite (ledPinsArray[0], HIGH);
    digitalWrite (ledPinsArray[1], HIGH);
    digitalWrite (ledPinsArray[2], HIGH);
    }
  } // end time check
} // end loop

void playaudio() {
  sfx.playTrack(n);
  sfx.playTrack((uint8_t)n);
}
