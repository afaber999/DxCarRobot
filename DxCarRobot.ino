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

int defaultSpeed = 230;

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

  motor.stop(255);
  gy273.Init();

  Serial.println("Robot versie 0.53");

  while (Serial.available() > 0) 
  {
    int inp = Serial.read();
  }
}



int idx = 0;
int cnt = 0;
int inp = 0;


static const unsigned int NO_MOTOR_STOP = 4294967295;

static unsigned long motor_stop_time = NO_MOTOR_STOP;
static unsigned long lastirrecv = millis();
static unsigned long next_action_time  = NO_MOTOR_STOP;


void ServoAtAngle(int angle)
{
  Serial.println("Set servo to angle ");
  Serial.print(angle, DEC);
  myservo.write( angle ); 
}

static long SetMotorStopTime(long intervalInMs)
{
   motor_stop_time = millis() + intervalInMs;
}

static long SetNextActionTime(long intervalInMs)
{
   next_action_time = millis() + intervalInMs;
}


bool autodrive = false;


bool ledOn = false;

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


long distance;
long duration;

int PingDistanceOnce()
{
  /* The following trigPin/echoPin cycle is used to determine the
  distance of the nearest object by bouncing soundwaves off of it. */ 
  digitalWrite(USTriggerPin, LOW); 
  delayMicroseconds(2); 
  
  digitalWrite(USTriggerPin, HIGH);
  delayMicroseconds(10); 
   
  digitalWrite(USTriggerPin, LOW);
  duration = pulseIn(USEchoPin, HIGH);
   
  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration/58.2;
   
  return (int)(distance + 0.5);
}

int PingDistance()
{
  long sum = PingDistanceOnce();
  sum += PingDistanceOnce();
  sum += PingDistanceOnce();
  sum += PingDistanceOnce();
  sum /= 4;
  return (int)sum;
}

static int action_state = -1;  

static int dist_FL = 0;
static int dist_NL = 0;
static int dist_C  = 0;
static int dist_NR = 0;
static int dist_FR = 0;

bool isStopped = false;

void SelfDrive()
{
    int dist = PingDistance();

    if( dist_C < distancelimit) 
    {

      SetLEDs(0);
      motor.stop(0); 
    }
    
    switch ( action_state )
    {
      case 0:	  
      case 1:	  
      case 2:	  
      case 3:	  
      case 4:	  
        dist_C = dist;
        
        if( dist_C >= distancelimit) 
        {
          SetLEDs(3);    
          motor.onFwd(Motor_298::Motor_LR,180); 
        }
      break;
        
      case 5:	  
        myservo.write(servo_angle_FL);
      break;
        
      break;
      case 6:
        dist_FL = dist;
        myservo.write(servo_angle_FR);
       delay(150);
      break;
      case 7:
        dist_FR = dist;
        myservo.write(servo_angle_C);
      break;            
      case 8:       
        dist_C = dist;
        
        Serial.print("dist_FR:  "); Serial.println (dist_FR, DEC);
        Serial.print("dist_C:   "); Serial.println (dist_C, DEC);
        Serial.print("dist_FL:  "); Serial.println (dist_FL, DEC);
    
        if ( dist_C > distancelimit )
        {
          // keep going forwaard
        }
        else if(  (dist_FL > sidedistancelimit) ) 
        {
            motor.stop(Motor_298::Motor_LR);
            motor.turnLeft(255);
            SetLEDs(1);
            Serial.print("turn left long"); 
            delay(500);
            motor.stop(Motor_298::Motor_LR);
            SetLEDs(0);
            
            
        } else if( ( dist_FR > sidedistancelimit )) 
        {
            motor.stop(Motor_298::Motor_LR);          
            motor.turnRight(255);
            SetLEDs(1);
            Serial.println("turn right long"); 
            delay(500);
            motor.stop(Motor_298::Motor_LR);
            SetLEDs(0);
          }
          else 
          {
            motor.stop(255);

            Serial.println("turn around: "); 
            motor.turnRight(255);
            SetLEDs(1);
            delay(800);
            motor.stop(Motor_298::Motor_LR);
            SetLEDs(0);
        }	
        //delay(15000);
       break;       
    }
    
    action_state++;
    if ( action_state > 8 ) action_state=0;  
    //Serial.print("action state set to  "); Serial.println (action_state, DEC);
    //SetNextActionTime(150);
    delay(150);
}

static void ExecuteAction( uint8_t code )
{
  switch( code )
  {
    case IR_KEY_UP:
      ServoAtAngle(0);
      motor.onFwd(Motor_298::Motor_LR,200);
      SetLEDs(3);
      SetMotorStopTime(900);
    break;
    case IR_KEY_DOWN:
      ServoAtAngle(0);
      motor.onRev(Motor_298::Motor_LR,200);
      SetLEDs(3);
      SetMotorStopTime(900);
    break;
    case IR_KEY_LEFT: 
      ServoAtAngle(0);
      motor.turnLeft(255);
      SetLEDs(1);
      SetMotorStopTime(400);
    break;
     case IR_KEY_RIGHT: 
      ServoAtAngle(0);
      motor.turnRight(255);
      SetLEDs(2);
      SetMotorStopTime(400);
    break;
    case IR_KEY_OK:
      autodrive = false;
      SetMotorStopTime(0);
    break;
    case IR_KEY_STAR:
      autodrive = true;
      action_state = 0;
     // SetNextActionTime(0);
    break;
    case IR_KEY_HASH:
      autodrive = true;
      action_state = 0;
     // SetNextActionTime(0);
    break;
  
    case IR_KEY_0: 
      motor.stop(Motor_298::Motor_LR);
      motor.turnLeft(255);
      delay(1500);
      motor.turnRight(255);
      delay(1500);
      
      Serial.print("Servo Snelle draai");
      myservo.write( servo_angle_FR ); 
      delay(500);
      myservo.write( servo_angle_C ); 
      delay(500);
      myservo.write( servo_angle_FL ); 
      delay(500);
      motor.stop(Motor_298::Motor_LR);
      myservo.write( servo_angle_C ); 
    break;
  
    case IR_KEY_7:
      Serial.print("Servo langzamer ");
      for ( idx = 0; idx < 180; idx += 10 ) 
      {
        myservo.write( idx ); 
        delay(50);
      }
      for ( idx = 180; idx > 0; idx -= 10 ) 
      {
        myservo.write( idx ); 
        delay(50);
      }
      myservo.write( servo_angle_C ); 
    break;
  } 
}



int snd = 0;

void loop() 
{
  if ( millis() > motor_stop_time )
  {
    motor.stop(Motor_298::Motor_LR);
    SetLEDs(0);
    autodrive = false;
    motor_stop_time = NO_MOTOR_STOP;
  }


  // ========================= IR RECV ==========================================================
  uint8_t irValue = irrecv.GetCode();
  if ( irValue > 0  )
  {
    Serial.print(irValue,HEX ); Serial.print(" " ); PrintKey(irValue); Serial.println(" " );
    ExecuteAction(irValue);
  }
 
  if (Serial.available() > 0) 
  {
    inp = Serial.read();
    
    switch ( inp )
    {
   case 'c': 
        for ( int i = 0; i< 100; i++ )
        {
          int offsetX = 30;
          int offsetY = 130;
          Gy273::Vector3i vec = gy273.Update();
  
          Serial.print(vec.X + offsetX);
          Serial.print("\t");
          Serial.print(vec.Y + offsetY);
          Serial.print("\t");
          Serial.print(vec.Z);
          Serial.println();
          delay(200);
        }
        break;

      case 'v': 
        for ( int i = 0; i< 100; i++ )
        {
          int offsetX = 30;
          int offsetY = 130;
          float angle = gy273.GetHeading(offsetX, offsetY);
  
          Serial.print(angle);
          Serial.println();
          delay(200);
        }

      /***********************Forward****************************/
      case 'F': 
            motor.onFwd(Motor_298::Motor_LR,defaultSpeed);
            break;

      /**********************Forward Left************************/
      case 'G': 
            motor.fwdLeft(defaultSpeed);
            break;
            
      /**********************Forward Right************************/
      case 'I': 
            motor.fwdRight(defaultSpeed);
            break;

      /***********************Backward****************************/
      case 'B': 
            motor.onRev(Motor_298::Motor_LR,defaultSpeed);
            break;

      /**********************Backward Left************************/
      case 'H': 
            motor.revLeft(defaultSpeed);
            break;

      /**********************Backward Right************************/
      case 'J': 
            motor.revRight(defaultSpeed);
            break;

      /********************** Left************************/
      case 'L': 
            motor.turnLeft(defaultSpeed);
            break;

      /********************** Right************************/
      case 'R': 
            motor.turnRight(defaultSpeed);
            break;
                  
      /********************** Lights ************************/
      case 'W': 
            ledOn = !ledOn;
            SetLEDs( ledOn ? 3 : 0 );
            break;

      /********************** STOP ************************/
      case 'S': 
            motor.stop(Motor_298::Motor_LR);
            break;


      case 'g':
          motor.revLeft(defaultSpeed);
          SetMotorStopTime(900);
      break;
      case 'h':
          motor.revRight(defaultSpeed);
          SetMotorStopTime(900);
      break;
      

      case '+':
        defaultSpeed += 10;
        if ( defaultSpeed >255) defaultSpeed = 255;
        analogWrite (speedPinA, defaultSpeed); 
        analogWrite (speedPinB, defaultSpeed);          
      break;

      case '-':
        defaultSpeed -= 10;
        if ( defaultSpeed <0 ) defaultSpeed = 0;
        analogWrite (speedPinA, defaultSpeed); 
        analogWrite (speedPinB, defaultSpeed);          
      break;
      
      case 'z':
      case '*':
          Serial.print("IR_KEY_STAR");
          autodrive = true;
          action_state = 0;
        //  SetNextActionTime(0);
      break;
      
      case '!': 
          Serial.println("Set servo to angle FR ");
          myservo.write( servo_angle_FR ); 
          break;
      case '@': 
          Serial.println("Set servo to angle NR ");
          myservo.write( servo_angle_NR ); 
          break;
      case '#': 
          Serial.println("Set servo to angle C ");
          myservo.write( servo_angle_C ); 
          break;
      case '$': 
          Serial.println("Set servo to angle NL ");
          myservo.write( servo_angle_NL ); 
          break;
      case '%': 
          Serial.println("Set servo to angle NL ");
          myservo.write( servo_angle_FL ); 
          break; 
      case '^':
        Serial.print("Servo langzamer ");
        for ( idx = 0; idx < 180; idx += 10 ) 
        {
          myservo.write( idx ); 
          delay(50);
        }
        for ( idx = 180; idx > 0; idx -= 10 ) 
        {
          myservo.write( idx ); 
          delay(50);
        }
        myservo.write( 90 ); 
        break;
      case '(':
        Serial.print("Servo Snelle draai");
        myservo.write( servo_angle_FR ); 
        delay(500);
        myservo.write( servo_angle_FL ); 
        delay(500);
        myservo.write( servo_angle_FR ); 
        delay(500);
        myservo.write( servo_angle_C ); 
        break;

     case 'p':
         Serial.println("Ultraound ping");
         for  (idx=0;idx<10; idx++ )
         {     
           int  distance = PingDistance();
            Serial.println(distance);
           delay(150);         
         }
        break;
        
    default:
//          SetMotorStopTime(0);
//             Serial.println("Unknown command");
     break;
    }
  }
    
  if ( autodrive )
  {
    SelfDrive();
  }
}


