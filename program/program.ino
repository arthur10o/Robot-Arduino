#include <Servo.h>
#include <IRremote.h>

#define STOP 0
#define ADVANCE 1
#define TURN_RIGHT 2
#define TURN_LEFT 3

struct Direction_servo_motor {
    const char* name;
    int angle;
};

Direction_servo_motor FRONT = {"front", 120};
Direction_servo_motor TO_RIGHT = {"right", 30};
Direction_servo_motor TO_LEFT = {"left", 210};

Servo myservo;

/*int IN1 = 12;
int IN2 = 11;
int IN3 = 10;
int IN4 = 2;
int ENA = 13;
int ENB = 8;*/

int trigPin = 9;
int echoPin = 10;

float duration_us, distance_cm;
float left_distance, right_distance;

int car_state = STOP;
Direction_servo_motor servo_state = FRONT;

int nb_measure = 10;

void setup() {
    Serial.begin(9600);

    /*pinMode(ENB, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(ENA, OUTPUT);*/

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    myservo.attach(11);
    myservo.write(servo_state.angle);
}

long measure_distance(int nb_measure, const char* direction) {
    long total_measure = 0;

    for (int i = 0; i < nb_measure; i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH, 30000);
        total_measure += duration;
        delay(100);
    }

    if (total_measure == 0) {
        Serial.print("Error: no signal for ");
        Serial.println(direction);
        return 0;
    }

    duration_us = total_measure / nb_measure;
    long distance = 0.0171 * duration_us;

    Serial.print("Distance ");
    Serial.print(direction);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");

    return distance;
}


void loop() {
    float distance = measure_distance(nb_measure/5, servo_state.name);
}
