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


void stopp() {
    // Function to stop the robot
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    Serial.println("stop");
}

void advance() {
    // Function to advance the robot
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 130);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 130);
    Serial.println("advance");
}

void turn_right() {
    // Function to turn right the robot
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 130);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 130);
    delay(250);
    Serial.println("turn right");
    stopp();
}

void turn_left() {
    // Function to turn left the robot
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 130);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 130);
    delay(250);
    Serial.println("turn left");
    stopp();
}

float measure_distance() {
    digitalWrite(trigPin, LOW);             // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    delayMicroseconds(5);

    digitalWrite(trigPin, HIGH);            // generate 10-microsecond pulse to TRIG pin
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    duration_us = pulseIn(echoPin, HIGH);   // measure duration of pulse from ECHO pin
    distance_cm = 0.0171 * duration_us;     // calculate the distance between obstacle and robot

    print_distance("", distance_cm, 100);   // For DEBUGING

    return distance_cm;
}

long measure_distance_left_right(int nb_measure) {
    // Function to measure the distance to the left and right if the robot encounters an obstacle
    measure = 0;
    for (int i = 0; i < nb_measure; i++) {
        digitalWrite(trigPin, LOW);     // generate 10-microsecond pulse to TRIG pin
        delayMicroseconds(10);
        digitalWrite(trigPin, HIGH);
        
        measure = measure + pulseIn(echoPin, HIGH);// measure duration of pulse from ECHO pin
        delay(100);
    }
    return measure;
}

void print_distance(String side, float distance, int time = 2000) {
    // Function to display the distance between the robot and an obstacle
    Serial.print(side + "distance : ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(time);
}

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
    distance_cm  = measure_distance();      // measures the distance in front of robot

    if (distance_cm < 50) {
        // 50 represents the distance in cm between obstacle and robot. If this distance is less than 50, the robot stops
        stopp();
        car_state = STOP;
    } else {
        advance();
        car_state = ADVANCE;
    }

    if (car_state == STOP)
    {
        myservo.write(TO_RIGHT);            // directs the servo motor in front
        delay(1000);

        measure = measure_distance_left_right(nb_measure);
        duration_us = measure / nb_measure;

        right_distance = 0.0171 * duration_us;// calculate the distance on the right

        print_distance("right", right_distance, 2000);

        myservo.write(TO_LEFT);              // directs the servo motor on the left
        delay(1000);

        measure = measure_distance_left_right(nb_measure);
        duration_us = measure / nb_measure;

        left_distance = 0.0171 * duration_us; // calculate the distance on the left
        
        print_distance("left", left_distance, 2000);

        if (left_distance > right_distance) {
            // Turn left if distance between obstacle and robot is greater on the left than on the right
            turn_left();
            print_distance("left", left_distance, 0);
            Serial.print("turn left");
        } else {
            // Turn right if distance between obstacle and robot is greater on the right than on the left
            turn_right();
            print_distance("right", right_distance, 0);
            Serial.print("turn right");
        }

        myservo.write(FRONT);               // directs the servo motor in front
        delay(1000);
        car_state = ADVANCE;
    }
}
