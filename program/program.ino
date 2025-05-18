#include <Servo.h>
#include <IRremote.h>

#define STOP 0
#define ADVANCE 1
#define TURN_RIGHT 2
#define TURN_LEFT 3

#define FRONT 120
#define TO_RIGHT 30
#define TO_LEFT 210

Servo myservo;                              // create servo object to control a servo

int IN1 = 12;
int IN2 = 11;
int IN3 = 10;
int IN4 = 2;
int ENA = 13;
int ENB = 8;

int pos = 0;                                // variable to store the servo position

int trigPin = 7;                            // TRIG pin
int echoPin = 6;                            // ECHO pin
float duration_us, distance_cm;
float left_distance, right_distance;

int car_state = STOP;

int nb_measure = 10;
long measure;

void setup() {
    int ENB  = OUTPUT;
    int IN4 = OUTPUT;
    int IN3 = OUTPUT;
    int IN2 = OUTPUT;
    int IN1 = OUTPUT;
    int ENA = OUTPUT;

    Serial.begin (9600);                    // begin serial port
    pinMode(trigPin, OUTPUT);               // configure the trigger pin to output mode
    pinMode(echoPin, INPUT);                // configure the echo pin to input mode

    myservo.attach(9);                      // attaches the servo on pin 9 to the servo object
    myservo.write(FRONT);                   // tell servo to go to position in variable 'pos'
}

void loop() {

}
