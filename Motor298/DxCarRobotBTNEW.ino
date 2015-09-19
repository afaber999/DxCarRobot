#include <Servo.h>
#include <Wire.h>
#include "Motor_298.h"
#include "pitches.h"
#include "irrecvnec.h"
#include "gy273.h"

#define BT_ENABLE (1)

static const int distancelimit = 25;      
static const int sidedistancelimit = 12; 

static const byte servo_angle_FL  = 170;
static const byte servo_angle_NL  = 130;
static const byte servo_angle_C   = 90;
static const byte servo_angle_NR  = 50;
static const byte servo_angle_FR  = 10;

int motorSpeed = 230;

//Motor A 
int speedPinA = 5;
int speedPinB = 6;


const int USServoPin = 9;
const int USTriggerPin = 12;
const int USEchoPin = 13;

const int IRRecvPin = 8;

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed


static Servo myservo;
static Gy273 gy273;
static Motor_298 motor;



const int LED_PIN = 13;
int randomValue = 0;
// 1000 ms of no data transmission required before and after the escape sequence
int cmdDelay = 1000; 
// Change these two paramaters to anything you want
int pin = 1234;
char* name = "AutoRobot";

void PlaySound(int sound);


// Motor driver L298N driver
// 
// EN DIR1 DIR2
// 0    0     0  Fast stop
// 0    1     0  Forwards
// 0    0     1  Backwards
// 0    1     1  Fast stop
// 1    x     x  Free running


long isrcount = 0;
IrRecvNec irrecv(8);

const uint8_t IR_KEY_UP    = 0xB9; 
const uint8_t IR_KEY_LEFT  = 0xBB; 
const uint8_t IR_KEY_OK    = 0xBF; 
const uint8_t IR_KEY_RIGHT = 0xBC; 
const uint8_t IR_KEY_DOWN  = 0xEA; 
const uint8_t IR_KEY_1     = 0xE9; 
const uint8_t IR_KEY_2     = 0xE6; 
const uint8_t IR_KEY_3     = 0xF2; 
const uint8_t IR_KEY_4     = 0xF3; 
const uint8_t IR_KEY_5     = 0xE7; 
const uint8_t IR_KEY_6     = 0xA1; 
const uint8_t IR_KEY_7     = 0xF7; 
const uint8_t IR_KEY_8     = 0xE3; 
const uint8_t IR_KEY_9     = 0xA5; 
const uint8_t IR_KEY_STAR  = 0xBD; 
const uint8_t IR_KEY_0     = 0xAD; 
const uint8_t IR_KEY_HASH  = 0xB5;

static void PrintKey(uint8_t keyCode)
{
  switch (keyCode )
  {
    case IR_KEY_UP    : Serial.print("IR_KEY_UP    "); break;
    case IR_KEY_LEFT  : Serial.print("IR_KEY_LEFT  "); break; 
    case IR_KEY_OK    : Serial.print("IR_KEY_OK    "); break; 
    case IR_KEY_RIGHT : Serial.print("IR_KEY_RIGHT "); break; 
    case IR_KEY_DOWN  : Serial.print("IR_KEY_DOWN  "); break; 
    case IR_KEY_1     : Serial.print("IR_KEY_1     "); break; 
    case IR_KEY_2     : Serial.print("IR_KEY_2     "); break; 
    case IR_KEY_3     : Serial.print("IR_KEY_3     "); break; 
    case IR_KEY_4     : Serial.print("IR_KEY_4     "); break; 
    case IR_KEY_5     : Serial.print("IR_KEY_5     "); break; 
    case IR_KEY_6     : Serial.print("IR_KEY_6     "); break; 
    case IR_KEY_7     : Serial.print("IR_KEY_7     "); break; 
    case IR_KEY_8     : Serial.print("IR_KEY_8     "); break; 
    case IR_KEY_9     : Serial.print("IR_KEY_9     "); break; 
    case IR_KEY_STAR  : Serial.print("IR_KEY_STAR  "); break; 
    case IR_KEY_0     : Serial.print("IR_KEY_0     "); break; 
    case IR_KEY_HASH  : Serial.print("IR_KEY_HASH  "); break;  }
}

// ISR timer 2
ISR(TIMER2_COMPA_vect)
{
  isrcount++;
  irrecv.Sample();
}


// This function sets up timer2 to trigger an ISR every 300 us.
static void setup_timer2()
{
  cli();                  // Disable global interrupts
  TCCR2A = 0;             // Clear timer2's control registers
  TCCR2B = 0;
  TIMSK2 = 0;             // ...and interrupt mask register (just in case)
  TCNT2 = 0;              // Pre-load the timer to 0
  OCR2A = 149;            // Set output compare register to 149
  
  TCCR2A |= _BV(WGM21);   // Turn on CTC mode (Clear Timer on Compare match)
  TCCR2B |= 0b011;        // Set prescaler to 32 (starts timer) 
  TIMSK2 |= _BV(OCIE2A);  // Enable timer compare interrupt 
  sei();                  // Re-enable global interrupts
}


void setup()
{  
  setup_timer2();
 
  // setup bluetooth modile
  pinMode(LED_PIN, OUTPUT);
  // Turn on LED to signal programming start
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(9600); 
  
  #if ( BT_ENABLE == 1 )
    delay(cmdDelay);
    Serial.print("AT");
    delay(cmdDelay);
    Serial.print("AT+PIN"); 
    Serial.print(pin); 
    delay(cmdDelay);
    Serial.print("AT+NAME");
    Serial.print(name); 
    delay(cmdDelay);
  #endif
  
  // Turn off LED to signal programming end
  digitalWrite(LED_PIN, LOW); 
  
  myservo.attach(USServoPin);
  myservo.write(servo_angle_C);
  delay(250);

  pinMode (USTriggerPin, OUTPUT);
  pinMode (USEchoPin, INPUT) ;

  motor.stop(Motor_298::Motor_LR);
  gy273.Init();

  Serial.println("Robot versie 0.51");

}



int idx = 0;
int cnt = 0;
int inp = 'S';

bool ledOn = false;


int rs = 0;

void SetLEDs(int la )
{
   pinMode(A0, OUTPUT);
   pinMode(A1, OUTPUT);

  switch ( la )
  {
    case 0:
      digitalWrite(A0,0);
      digitalWrite(A1,0);
      break;
    case 1:
      digitalWrite(A0,1);
      digitalWrite(A1,0);
      break;
    case 2:
      digitalWrite(A0,0);
      digitalWrite(A1,1);
      break;
    case 3:
      digitalWrite(A0,1);
      digitalWrite(A1,1);
      break;      
  }
}


long previousMillis = -1000*10;// -1000*10=-10sec. to read the first value. If you use 0 then you will take the first value after 10sec.  
long interval = 1000*10;       // interval at which to read battery voltage, change it if you want! (10*1000=10sec)
unsigned long currentMillis;   //unsigned long currentMillis;

int prevInp = 0;

void loop() 
{

    currentMillis = millis();
    if(currentMillis - (previousMillis) > (interval)) 
    {
       previousMillis = currentMillis; 
       Serial.println(rs);    
    }
     
  if (Serial.available() > 0) 
  {
    inp = Serial.read();
  }

  if ( inp != prevInp )
  {
    prevInp = inp;
    switch ( inp )
    {
  
      case '0':
        motorSpeed = 150;
      break;
      case '1':
        motorSpeed = 180;
      break;
      case '2':
        motorSpeed = 200;
      break;
      case '3':
        motorSpeed = 230;
      break;
      case '4':
        motorSpeed = 255;
      break;
  
  
      /***********************Forward****************************/
      case 'F': 
            motor.onFwd(Motor_298::Motor_LR,motorSpeed);
            break;
  
      /**********************Forward Left************************/
      case 'G': 
            motor.fwdLeft(motorSpeed);
            break;
            
      /**********************Forward Right************************/
      case 'I': 
            motor.fwdRight(motorSpeed);
            break;
  
      /***********************Backward****************************/
      case 'B': 
            motor.onRev(Motor_298::Motor_LR, motorSpeed);
            break;
  
      /**********************Backward Left************************/
      case 'H': 
            motor.revLeft(motorSpeed);
            break;
  
      /**********************Backward Right************************/
      case 'J': 
            motor.revRight(motorSpeed);
            break;
  
      /********************** Left************************/
      case 'L': 
            motor.turnLeft(motorSpeed);
            break;
  
      /********************** Right************************/
      case 'R': 
            motor.turnRight(motorSpeed);
            break;
                  
      /********************** Lights ************************/
      case 'W': 
            ledOn = !ledOn;
            SetLEDs( ledOn ? 3 : 0 );
            inp = 'n';
            break;
  
      /********************** STOP ************************/
      case 'S': 
            motor.stop(Motor_298::Motor_LR);
            break;
            
    }
  }
}


