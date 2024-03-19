//#define A0; // RIGHT PHOTOTRANSISTOR
//#define A1; // MIDDLE PHOTOTRANSISTOR
//#define A2; // LEFT PHOTOTRANSISTOR
//#define A5; // RIGHT PHOTOTRANSISTOR
//#define A6; // 4 WAY DIP SWITCH ANALOG READ
//#define A7; // BATTERY ANALOG READ
//#define A4; // UNSURE 

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define __digitalPinToPortReg(P) (((P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) (((P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) (((P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) (((P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

// general macros/defines
#if !defined(BIT_READ)
#define BIT_READ(value, bit) ((value) & (1UL << (bit)))
#endif
#if !defined(BIT_SET)
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))
#endif
#if !defined(BIT_CLEAR)
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#if !defined(BIT_WRITE)
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#define fast_write_pin(P, V) BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V));
#define fast_read_pin(P) ((int)(((BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ? HIGH : LOW)))

#else
#define fast_write_pin(P, V) digitalWrite(P, V)
#define fast_read_pin(P) digitalRead(P)
#endif

const int RED_LEDS = 12; // RED LEDS
const int RED_LED_H_BRIDGE = 13; // RED LED H BRIDGE

const int INDICATOR_LED_R = 6; // INDICATOR LED RIGHT 
const int INDICATOR_LED_L = 11; // INDICATOR LED LEFT

const int ENCODER_R_A = 3; // ENCODER RIGHT A (ticks first when motor forward)
const int ENCODER_R_B = 5; // ENCODER RIGHT B (ticks first when motor backward) FWD

const int ENCODER_L_A = 4; // ENCODER LEFT A (ticks first when motor forward)
const int ENCODER_L_B = 2; // ENCODER LEFT B (ticks first when motor backward)

const int SPEED_MOTOR_L = 9; // PWM MOTOR LEFT 
const int SPEED_MOTOR_R = 10; // PWM MOTOR RIGHT 

const int DIR_MOTOR_L = 7; // DIRECTION MOTOR LEFT 
const int DIR_MOTOR_R = 8; // DIRECTION MOTOR RIGHT 

// Directions
const int FORWARDS_L = 1;
const int FORWARDS_R = 0;
const int REVERSE_L = 0;
const int REVERSE_R = 1;

const int RIGHT_SENSOR = A0;
const int LEFT_SENSOR = A2;
const int MIDDLE_SENSOR = A1;

volatile int rightEncoderPos = 0; // Counts for right encoder position
volatile int leftEncoderPos = 0; // Counts for left encoder position

volatile bool interruptOccurred = false;
volatile unsigned long startTime;
volatile unsigned long endTime;
volatile unsigned long duration;
volatile unsigned long prevDuration = 0;
volatile int uptick = 1;
volatile bool isActive;

void setup() {
  Serial.begin(9600);
  
  pinMode(RED_LEDS, OUTPUT);
  pinMode(RED_LED_H_BRIDGE, OUTPUT);
  pinMode(INDICATOR_LED_R, OUTPUT);
  pinMode(INDICATOR_LED_L, OUTPUT);

  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  pinMode(SPEED_MOTOR_L, OUTPUT);
  pinMode(SPEED_MOTOR_R, OUTPUT);
  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), interruptHandlerLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), interruptHandlerRight, CHANGE);
}

void interruptHandlerLeft() {
  if (interruptOccurred) {
    if(uptick == 4){
      endTime = micros();
      uptick = 1;
    }
    if(uptick == 3 && fast_read_pin(ENCODER_L_A) == LOW && isActive == true){ // If A active when B trigger and on 3rd pulse is low then forward
          leftEncoderPos++;
          isActive = false;
      } else if(uptick == 2 && fast_read_pin(ENCODER_L_A) == HIGH && isActive == false){ // If A not active when B trigger and also active on second tick we are in reverse
          leftEncoderPos--;
      }
  } else {
    if(uptick == 1){
      startTime = micros();
      if(fast_read_pin(ENCODER_L_A) == HIGH){
        isActive = true;  
      } else{
        isActive = false;
      }
    }
    uptick++;
  }
  interruptOccurred = !interruptOccurred;
}

void interruptHandlerRight() {
  if (interruptOccurred) {
    if(uptick == 4){
      endTime = micros();
      uptick = 1;
    }
    if(uptick == 2 && fast_read_pin(ENCODER_R_B) == HIGH && isActive == true){ // If B active when A trigger and 2nd pulse is high we go forward
          rightEncoderPos++;
          isActive = false;
      } else if(uptick == 2 && fast_read_pin(ENCODER_R_B) == HIGH && isActive == false){ // If B not active on A trigger and B active on 2nd pulse we in reverse
          rightEncoderPos--;
      }
  } else {
    if(uptick == 1){
      startTime = micros();
      if(fast_read_pin(ENCODER_R_B) == HIGH){
        isActive = true;  
      } else{
        isActive = false;
      }
    }
    uptick++;
  }
  interruptOccurred = !interruptOccurred;
}

void loop(){
  //Serial.println(ticks());
  String l = String(leftEncoderPos);
  String r = String(rightEncoderPos);

  Serial.println("left: " + l + " right: " + r);

  /*digitalWrite(DIR_MOTOR_R, FORWARDS_R);
  analogWrite(SPEED_MOTOR_R, 150);

  digitalWrite(DIR_MOTOR_L, FORWARDS_L);
  analogWrite(SPEED_MOTOR_L, 150);*/

  //
}
