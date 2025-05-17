#include <Servo.h>
#include <IRremote.h>

#define ARRET 0
#define AVANCE 1
#define TOURNE_DROITE 2
#define TOURNE_GAUCHE 3

#define DEVANT 120
#define A_DROITE 30
#define A_GAUCHE 210

Servo myservo;                        // create servo object to control a servo

int IN1 = 12;
int IN2 = 11;
int IN3 = 10;
int IN4 = 2;
int ENA = 13;
int ENB = 8;

int pos = 0;                        // variable to store the servo position

int trigPin = 7;                    // TRIG pin
int echoPin = 6;                    // ECHO pin
float duration_us, distance_cm;
float distance_gauche, distance_droite;

int car_state = ARRET;

int nb_mesure = 10;
long mesure;