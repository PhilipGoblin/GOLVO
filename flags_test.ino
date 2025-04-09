//nano PWM: (D11?,) D10, D9, D6, D5, D3
  /************************************************************
  *               SERVO SETTINGS AND H-BRIDGE                 *
  *************************************************************
  * - Servo normally uses 0-180 degrees for right-left,       *
  *   but the car is mechanically restriced to 60-120, with   *
  *   93 being straight ahead.                                *
  * - IN1 and IN2 needs to be connected to PWM pins. We use   *
  *   pins 3 and 6 because they SHOULD be using a different   *
  *   timer than the built-in servo library for Arduino       *
  ************************************************************/ 
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
#define TRIG_LEFT 4
#define ECHO_LEFT 10

#define TRIG_FRONT 11
#define ECHO_FRONT 7

#define TRIG_RIGHT 8
#define ECHO_RIGHT 9

#define SENSOR_DELAY 200
#define MAX_DISTANCE 100
#define ERROR_DISTANCE 666

uint16_t distance_left = 0;
uint16_t distance_front = 0;
uint16_t distance_right = 0;

uint8_t safety_distance = 20;   //distance in cm
uint8_t surrounded = 0;  //used as a flag if all sensors measures close objects (1 for true, 0 for false)
uint8_t counter = 0;

uint8_t left_turn = 0;
uint8_t right_turn = 0;
uint8_t reversing = 0;

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
  distance = ((pulseIn(echo_pin, HIGH, 50000))/58);

  if (distance == 0 || distance > MAX_DISTANCE) {
    return ERROR_DISTANCE;
  } else if (distance <= MAX_DISTANCE) {
    return distance;
  }

}

  /************************************************************
  /*                     DC MOTOR FUNCTIONS                    *
  /*************************************************************
  /* - analogWrite is needed for PWM, which controls the speed
  /* - By setting IN1 LOW, the direction is forward. By
  /*   setting IN2 LOW (with analogWRite on IN1) the direction
  /*   is backwards (reversing)
  /* - Speed variable is 0-255, where 255 is max speed
  /* - You probably want to brake engine before switching
  /*   direction, to reduce high current flow
  /* - Disengaging the engine is supposed to just let it roll
  /*   to a stop. I don't think it's needed                   */
  /************************************************************/
void drive(uint8_t speed) {
  digitalWrite(IN2, LOW);
  analogWrite(IN1, speed);
}

void reverse(uint8_t speed) {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, speed);
}

void brake(void) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}
/*
void disengage(void) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
*/
  /************************************************************/
  /*                   TURNING FUNCTIONS                      */ 
  /************************************************************/
  /*Text*/
  /************************************************************/
void turn_left(void) {
  servo.write(LEFT);
  delay(SENSOR_DELAY);
  servo.write(STRAIGHT);
}

void turn_right(void) {
  servo.write(RIGHT);
  delay(SENSOR_DELAY);
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

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);

}

void loop() {
  /************************************************************/
  /*          RUN ALL SENSORS AND CALCULATE DISTANCES         */
  /************************************************************/
  /*  - IMPORTANT! There must be a delay of AT LEAST 60 ms    */
  /*    between each sensor trigger, otherwise errors might   */
  /*    occur.                                                */
  /************************************************************/
  trigger_sensor(TRIG_LEFT);
  distance_left = calculate_distance(ECHO_LEFT);
  delay(SENSOR_DELAY);

  trigger_sensor(TRIG_FRONT);
  distance_front = calculate_distance(ECHO_FRONT);
  delay(SENSOR_DELAY);

  trigger_sensor(TRIG_RIGHT);
  distance_right = calculate_distance(ECHO_RIGHT);
  delay(SENSOR_DELAY);

  //drive(60);

  if (((distance_front < safety_distance) && (distance_front != ERROR_DISTANCE)) && ((distance_left > safety_distance) && (distance_left != ERROR_DISTANCE))) {
  //  servo.write(LEFT); //maybe make smooth turn?
  
    left_turn = 1;
  } else if (((distance_front < safety_distance) && (distance_front != ERROR_DISTANCE)) && ((distance_right > safety_distance) && (distance_right != ERROR_DISTANCE))) {
  //  servo.write(RIGHT);
    right_turn = 1;
  } else if (((distance_front < safety_distance) && (distance_front != ERROR_DISTANCE)) && ((distance_left < safety_distance) && (distance_left != ERROR_DISTANCE)) && ((distance_right < safety_distance) && (distance_right != ERROR_DISTANCE))) {
  //  brake();
    counter = 0;
    surrounded = 1;   //SURROUNDED WILL NEED TO BE IN EVERY FLAG? 
  }

  while (left_turn) {
    trigger_sensor(TRIG_FRONT);
    distance_front = calculate_distance(ECHO_FRONT);
    delay(SENSOR_DELAY);
    
    if ((distance_front > safety_distance) && (distance_front != ERROR_DISTANCE)) {
  //    servo.write(STRAIGHT);
      left_turn = 0;
    } /*else if ((distance_front < safety_distance) && (distance_right > safety_distance)) {
      servo.write(RIGHT);
      right_turn = 1;
      left_turn = 0;
    } */

  }

  while (right_turn) {
    trigger_sensor(TRIG_FRONT);
    distance_front = calculate_distance(ECHO_FRONT);
    delay(SENSOR_DELAY);

    if ((distance_front > safety_distance) && (distance_front != ERROR_DISTANCE)) {
  //    servo.write(STRAIGHT);
      right_turn = 0;
    }
  }

  while (surrounded) {
    trigger_sensor(TRIG_FRONT);
    distance_front = calculate_distance(ECHO_FRONT);
    delay(SENSOR_DELAY);
    Serial.println("REVERSING");

    if ((distance_front > safety_distance) && (distance_front != ERROR_DISTANCE)) {
      //drive(60);
      surrounded = 0;
    } else if (counter >= 30) {
      //reverse(60);
      reversing = 1;
      surrounded = 0;
    } else {
      counter++;
    }

  }
/*
  while (reversing) {

  }
*/


  

  /************************************************************/
  /*                          PRINTS                          */
  /************************************************************/
  /*  - Prints sensor distances to serial monitor.            */
  /*  - This is just for validation purposes, not to be used  */
  /*    in finished product.                                  */
  /************************************************************/
  Serial.println("Left\tFront\tRight");
  Serial.print(distance_left);
  Serial.print("\t");
  Serial.print(distance_front);
  Serial.print("\t");
  Serial.println(distance_right); 

}
  //Comment box template:

  /************************************************************/
  /*[TITLE HERE]*/
  /************************************************************/
  /*Text*/
  /************************************************************/
