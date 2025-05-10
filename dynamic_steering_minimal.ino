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

#define STRAIGHT 94 //servo setting, 90 degrees was a bit off
#define LEFT 120    //servo setting
#define RIGHT 60    //servo setting
#define SERVO_PIN 5
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

#define SENSOR_DELAY 100
#define MAX_DISTANCE 50
#define ERROR_DISTANCE 666
#define SAFETY_DISTANCE 10

#define SPEED 60

int16_t servo_position = 0; //int to be able to handle negative numbers

uint16_t distance_left = 0;
uint16_t distance_front = 0;
uint16_t distance_right = 0;


  /************************************************************/
  /*      TRIGGER SENSOR AND CALCULATE DISTANCE FUNCTION      */
  /************************************************************/
  /*  - Sets TRIG pin HIGH for 10 microseconds to send burst  */
  /*    of ultrasonic soundwaves at 40kHz.                    */
  /*  - TRIG pin MUST be HIGH for 10 microseconds.            */
  /*  - Distance is calculated by dividing the length of time */ 
  /*    ECHO pin has been HIGH by 58. (pulseIn(x, y, z) has a */
  /*    third option, a safety feature, where if ECHO pin has */
  /*    been HIGH for z milliseconds, it stops measuring      */
  /*  - There must be a delay of AT LEAST 60ms between each   */
  /*    distance measure.                                     */
  /*  - Filters faulty and unnecessary measurements (0 and >50*/
  /*    cm). MAX_DISTANCE is the furthest measurement we're   */
  /*    currently interested in.                              */
  /************************************************************/
uint16_t measure_distance(uint8_t trig_pin, uint8_t echo_pin) {
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  uint16_t distance = 0;
  distance = ((pulseIn(echo_pin, HIGH))/58);
  delay(SENSOR_DELAY);

  if (distance == 0 || distance > MAX_DISTANCE) {
    return MAX_DISTANCE;
  } else if (distance <= MAX_DISTANCE) {
    return distance;
  }

}

  /************************************************************/
  /*                     DC MOTOR FUNCTIONS                   */
  /************************************************************/
  /* - analogWrite is needed for PWM, which controls the speed*/
  /* - By setting IN1 LOW, the direction is forward. By       */
  /*   setting IN2 LOW (with analogWRite on IN1) the direction*/
  /*   is backwards (reversing)                               */
  /* - Speed variable is 0-255, where 255 is max speed        */
  /* - You probably want to brake engine before switching     */
  /*   direction, to reduce high current flow                 */
  /* - Disengaging the engine is supposed to just let it roll */
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
void disengage(void) {        //FUNCTION PROBABLY NOT NEEDED
  digitalWrite(IN1, LOW);       
  digitalWrite(IN2, LOW);
}
*/
  /************************************************************/
  /*                SERVO ADJUSTMENT FUNCTION                 */ 
  /************************************************************/
  /*  - ISSUE: Unless both distance left and right are        */
  /*    updated, the servo currently won't do anything if     */
  /*    one sensor suddenly measures <10cm. However, if you   */
  /*    slowly bring an object towards the sensor from a      */
  /*    longer distance, the servo position updates correctly.*/
  /************************************************************/

int16_t servo_adjust(uint8_t distance_left, uint8_t distance_right) {
  int16_t servo_position = ((distance_left - distance_right) + STRAIGHT);   //EX: 50 - 0 + 90 = 140   =
                                                                            // =  140 > 120 = 120 = MAX LEFT

                                                                            //EX2: 35 - 50 + 90 = 75   =
                                                                            //  = 75 !> 120, 75 !< 60 = A BIT RIGHT
                                                                            // 

                                                                            //EX3: 7 - 40 + 90 = 27   =
                                                                            //  = 27 !> 120, 27 < 60 = MAX RIGHT
  if (distance_right < SAFETY_DISTANCE) {
    distance_right = 0;
  }

  if (distance_left < SAFETY_DISTANCE) {
    distance_left = 0;
  }

  if (servo_position > LEFT) {          //LEFT = 120      
    servo_position = LEFT;
  } else if (servo_position < RIGHT) {  //RIGHT = 60
    servo_position = RIGHT;
  } else {
    return servo_position;
  }
}

void setup() {
  /************************************************************/
  /*                  SET INPUT/OUTPUT PINS                   */
  /************************************************************/
  /*  - TRIG pins need to be outputs.                         */
  /*  - ECHO pins need to be inputs.                          */
  /************************************************************/
  Serial.begin(9600);

  servo.attach(SERVO_PIN); //Servo needs to be connected to a PWM pin

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

  distance_left = measure_distance(TRIG_LEFT, ECHO_LEFT);
  distance_front = measure_distance(TRIG_FRONT, ECHO_FRONT);
  distance_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT);
  servo_position = servo_adjust(distance_left, distance_right);
  //drive(SPEED);
  servo.write(servo_position);

//  if (distance_front < 10) {    //temporary if case, to easily stop the vehicle while testing on track. To be replaced with if case below when it works.
//    brake();
//  }
/*
  if (distance_front < SAFETY_DISTANCE) {
    brake();
    delay(1000);
    distance_front = measure_distance(TRIG_FRONT, ECHO_FRONT);

    if (distance_front > SAFETY_DISTANCE) {
      drive(SPEED);
    } else if (distance_front < SAFETY_DISTANCE) {
      servo.write(STRAIGHT);
      delay(60);
      reverse(SPEED);
      delay(666);
      brake();
      delay(100);
      drive(SPEED);
    } 

  
  } */
  /************************************************************/
  /*                          PRINTS                          */
  /************************************************************/
  /*  - Prints sensor distances to serial monitor.            */
  /*  - This is just for validation purposes, not to be used  */
  /*    in finished product.                                  */
  /************************************************************/
 /* Serial.println("Left\tFront\tRight\tServo");
  Serial.print(distance_left);
  Serial.print("\t");
  Serial.print(distance_front);
  Serial.print("\t");
  Serial.print(distance_right);
  Serial.print("\t");
  Serial.println(servo_position);
  */
} 
  //Comment box template:

  /************************************************************/
  /*[TITLE HERE]*/
  /************************************************************/
  /*Text*/
  /************************************************************/
