//nano PWM: (D11?,) D10, D9, D6, D5, D3

#include <Servo.h>

#define STRAIGHT 93 //servo setting, 90 degrees was a bit off
#define LEFT 120    //servo setting
#define RIGHT 60    //servo setting
#define IN1 3       //needs to be PWM 
#define IN2 6  //needs to be PWM

Servo servo; //Servo [NAME OF SERVO];

/************************************************************/
/*                      SENSOR PINS                         */
/************************************************************/           
/* Left sensor:         Front sensor:         Right sensor: */
/* TRIG pin: D4         TRIG pin: D11         TRIG pin: D8  */
/* ECHO pin: D10        ECHO pin: D7          ECHO pin: D9  */
/************************************************************/
uint8_t trig_pin_left = 4;    //MAYBE CHANGE TRIG/ECHO PINS TO #DEFINE?
uint8_t echo_pin_left = 10;

uint8_t trig_pin_front = 11;
uint8_t echo_pin_front = 7;

uint8_t trig_pin_right = 8;
uint8_t echo_pin_right = 9;

uint16_t distance_left = 0;
uint16_t distance_front = 0;
uint16_t distance_right = 0;

uint8_t safety_distance = 20;   //distance in cm
uint8_t surrounded = 0;  //used as a flag if all sensors measures close objects (1 for true, 0 for false)
uint8_t counter = 0;


  /************************************************************/
  /*                  TRIGGER SENSOR FUNCTION                 */
  /************************************************************/
  /*  - Sets TRIG pin HIGH for 10 microseconds to send burst  */
  /*    of ultrasonic soundwaves at 40kHz.                    */
  /*  - TRIG pin MUST be HIGH for 10 microseconds.            */
  /************************************************************/
void trigger_sensor(uint8_t trig_pin) {
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
}

  /************************************************************/
  /*                CALCULATE DISTANCE FUNCTION               */
  /************************************************************/
  /*  - Distance is calculated by dividing the length of time */ 
  /*    ECHO pin has been HIGH by 58.                         */
  /*  - Safety feature: If ECHO pin has been HIGH for         */
  /*    30000 milliseconds, it shuts off (faulty measurement).*/
  /************************************************************/
uint16_t calculate_distance(uint8_t echo_pin) {
  uint16_t distance = 0;
  distance = ((pulseIn(echo_pin, HIGH, 30000))/58);
  return distance;
}

  /************************************************************
  *                     DC MOTOR FUNCTIONS                    *
  *************************************************************
  * - analogWrite is needed for PWM, which controls the speed
  * - By setting IN1 LOW, the direction is forward. By
  *   setting IN2 LOW (with analogWRite on IN1) the direction
  *   is backwards (reversing)
  * - Speed variable is 0-255, where 255 is max speed
  * - You probably want to brake engine before switching
  *   direction, to reduce high current flow
  * - Disengaging the engine is supposed to just let it roll
  *   to a stop.         *
  ************************************************************/
void drive(uint8_t speed) {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, speed);
}

void reverse(uint8_t speed) {
  digitalWrite(IN2, LOW);
  analogWrite(IN1, speed);
}

void brake(void) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void disengage(void) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

  /************************************************************
  *                   TURNING FUNCTIONS                       *
  *************************************************************
  *Text*
  ************************************************************/
void turn_left(void) {
  servo.write(LEFT);
  delay(100);
  servo.write(STRAIGHT);
}

void turn_right(void) {
  servo.write(RIGHT);
  delay(100);
  servo.write(STRAIGHT);
}


void setup() {
  /************************************************************/
  /*                  SET INPUT/OUTPUT PINS                   */
  /************************************************************/
  /*  - TRIG pins need to be outputs.                         */
  /*  - ECHO pins need to be inputs.                          */
  /************************************************************/
  Serial.begin(9600);

  servo.attach(5); //Servo connected to pin 5 (PWM)

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(trig_pin_left, OUTPUT);
  pinMode(trig_pin_front, OUTPUT);
  pinMode(trig_pin_right, OUTPUT);
  pinMode(echo_pin_left, INPUT);
  pinMode(echo_pin_front, INPUT);
  pinMode(echo_pin_right, INPUT);

}

void loop() {
  /************************************************************/
  /*          RUN ALL SENSORS AND CALCULATE DISTANCES         */
  /************************************************************/
  /*  - IMPORTANT! There must be a delay of AT LEAST 60 ms    */
  /*    between each sensor trigger, otherwise errors might   */
  /*    occur.                                                */
  /************************************************************/
  trigger_sensor(trig_pin_left);
  distance_left = calculate_distance(echo_pin_left);
  delay(60);    

  trigger_sensor(trig_pin_front);
  distance_front = calculate_distance(echo_pin_front);
  delay(60);

  trigger_sensor(trig_pin_right);
  distance_right = calculate_distance(echo_pin_right);
  delay(60);

  servo.write(STRAIGHT);
  drive(100);

  if ((distance_front < safety_distance) && (distance_left > safety_distance)) {
    turn_left();
    //we might need another if statement here to steer the car back to where it was heading?
  } else if ((distance_front < safety_distance) && (distance_right > safety_distance)) {
    turn_right();
    //same as above
  } else if ((distance_front < safety_distance) && (distance_left < safety_distance) && (distance_right < safety_distance)) {
    brake();
    counter = 0;
    surrounded = 1;   

    while (surrounded) {
      trigger_sensor(trig_pin_front);
      distance_front = calculate_distance(echo_pin_front);
      delay(60);

      if (distance_front > safety_distance) {
        drive(100);
        surrounded = 0;
      } else if (counter >= 30) {
        reverse(100);
        delay(1000);
        brake();
        surrounded = 0;
      }

      counter++;    //every 60ms (60ms because of the sensor delay), the counter variable increases by 1. When the counter is 30, 30*60ms has passed (1800ms = 1.8s).    
    }


  }

  

  /************************************************************/
  /*                          PRINTS                          */
  /************************************************************/
  /*  - Prints sensor distances to serial monitor.            */
  /*  - This is just for validation purposes, not to be used  */
  /*    in finished product.                                  */
  /************************************************************/
/*  Serial.println("Left\tFront\tRight");
  Serial.print(distance_left);
  Serial.print("\t");
  Serial.print(distance_front);
  Serial.print("\t");
  Serial.println(distance_right); */
}
  //Comment box template:

  /************************************************************
  *[TITLE HERE]*
  *************************************************************
  *Text*
  ************************************************************/
