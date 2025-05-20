#include <Servo.h>
#include <IRremote.h>

#define STOP 0
#define ADVANCE 1
#define TURN_RIGHT 2
#define TURN_LEFT 3

Servo myservo;

int IN1 = 6;
int IN2 = 7;
int IN3 = 12;
int IN4 = 13;
int ENA = 5;
int ENB = 2;

int trigPin = 9;
int echoPin = 10;

int nb_measure = 10;
int minimum_distance = 60;          // distance in cm at which robot must stop

long left_distance, right_distance;

unsigned long lastMillis = 0;
const long interval = 150;         // Waiting time in ms between distance measurements.

long quickselect(long arr[], int left, int right, int k);
int partition(long arr[], int left, int right);

struct Car_state {
    int last_moove;
};

Car_state car_state;

struct Direction_servo_motor {
    const char* name;
    int angle;
    int last_position;
};

Direction_servo_motor FRONT = {"front", 120};
Direction_servo_motor TO_RIGHT = {"right", 115};
Direction_servo_motor TO_LEFT = {"left", 125};
Direction_servo_motor servo_state = FRONT;

Direction_servo_motor direction_to_check[] = {
    TO_RIGHT,
    FRONT,
    TO_LEFT,
    FRONT,
};

const int nb_direction = sizeof(direction_to_check) / sizeof(direction_to_check[0]);
int distance[nb_direction];

const int MAX_CACHED_ANGLES = 10;
int cached_angles[MAX_CACHED_ANGLES];
long cached_distances[MAX_CACHED_ANGLES];
int cached_count = 0;

long measure_distance(int nb_measure, const char* direction) {
    /**
    * Measures the distance between the robot and an obstacle using an ultrasonic sensor.
    * The function performs multiple measurements and returns the average value.
    * 
    * @param nb_measure Number of individual distance measurements to average.
    * @param direction  Direction label.
    * @return median in centimeters (as a long integer).
    */
    long measures[20];
    int count = min(nb_measure, 20);

    for (int i = 0; i < count; i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH, 25000);
        if(duration == 0) {
            measures[i] = 999;
        } else {
            measures[i] = 0.0171 * duration;
        }
    }
    
    long median;

    if (count % 2 == 0) {
        long m1 = quickselect(measures, 0, count - 1, count / 2);
        long m2 = quickselect(measures, 0, count - 1, count / 2 - 1);
        median = (m1 + m2) / 2;
    } else {
        median = quickselect(measures, 0, count - 1, count / 2);
    }

    if (median == 0) {
        Serial.print("Error: no signal for ");
        Serial.println(direction);
        return 0;
    }

    /*Serial.print("Distance ");
    Serial.print(direction);
    Serial.print(": ");                             // DEBUG
    Serial.print(median);
    Serial.println(" cm");*/

    return median;
}

int partition(long arr[], int left, int right) {
    /**
    * Partitions an array around a pivot for use in the quickselect algorithm.
    * Elements smaller than the pivot are moved to the left, and larger to the right.
    *
    * @param arr   The array to partition.
    * @param left  Starting index of the subarray.
    * @param right Ending index of the subarray (pivot element).
    * @return The index position of the pivot after partitioning.
    */
    long pivot = arr[right];
    int i = left;
    for (int j = left; j < right; j++) {
        if (arr[j] < pivot) {
            long tmp = arr[i];
            arr[i] = arr[j];
            arr[j] = tmp;
            i++;
        }
    }
    long tmp = arr[i];
    arr[i] = arr[right];
    arr[right] = tmp;
    return i;
}

long quickselect(long arr[], int left, int right, int k) {
    /**
    * Selects the k-th smallest element from the array using the Quickselect algorithm.
    * This is used for efficiently finding the median in unsorted data.
    *
    * @param arr   The array to search.
    * @param left  Starting index of the subarray.
    * @param right Ending index of the subarray.
    * @param k     The index (0-based) of the desired smallest element.
    * @return The k-th smallest element in the array.
    */
    if (left == right) return arr[left];
    int pivotIndex = partition(arr, left, right);
    if (k == pivotIndex) return arr[k];
    else if (k < pivotIndex) return quickselect(arr, left, pivotIndex - 1, k);
    else return quickselect(arr, pivotIndex + 1, right, k);
}

long get_cached_distance(int angle) {
    for (int i = 0; i < cached_count; i++) {
        if (cached_angles[i] == angle) {
            return cached_distances[i];
        }
    }
    return -1;
}

void add_cached_distance(int angle, long distance) {
    if (cached_count < MAX_CACHED_ANGLES) {
        cached_angles[cached_count] = angle;
        cached_distances[cached_count] = distance;
        cached_count++;
    }
}

void stopp() {
    /**
    * Stops the robot by disabling all motor outputs.
    */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    Serial.println("stop");
}

void advance() {
    /**
    * Moves the robot forward by activating both motors in forward direction.
    */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 130);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 130);
    Serial.println("advance");
}

void turn_right(int time) {
    /**
    * Rotates the robot to the right by running the motors in opposite directions.
    * A short delay is used to complete the turn before stopping.
    * @param time Rotation time in milliseconds. The higher the value, the longer the robot rotates.
    */
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 130);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 130);
    delay(time);
    Serial.println("turn right");
    stopp();
}

void turn_left(int time) {
    /**
    * Rotates the robot to the left by running the motors in opposite directions.
    * A short delay is used to complete the turn before stopping.
    * @param time Rotation time in milliseconds. The higher the value, the longer the robot rotates.
    */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 130);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 130);
    delay(time);
    Serial.println("turn left");
    stopp();
}

void setup() {
    /**
    * Initializes the robot's hardware configuration.
    * Sets up pins, attaches the servo motor, and defines initial movement state.
    */
    Serial.begin(9600);

    pinMode(ENB, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    myservo.attach(11);
    myservo.write(servo_state.angle);

    car_state.last_moove = STOP;
    delay(1000);
}

void loop() {
    /**
    * Main control loop of the robot.
    * If moving forward, constantly checks distances to obstacles.
    * If stopped, scans surroundings and chooses a direction with the most space to resume movement.
    */
    if(car_state.last_moove == ADVANCE) {
        bool obstacle_detected = false;
        cached_count = 0;
        for (int i = 0; i < nb_direction; i++) {
            servo_state = direction_to_check[i];
            myservo.write(servo_state.angle);
            servo_state.last_position = servo_state.angle;
            delay(150);

            long cached = get_cached_distance(servo_state.angle);
            if (cached == -1) {
                cached = measure_distance(nb_measure / 5, servo_state.name);
                add_cached_distance(servo_state.angle, cached);
            }

            distance[i] = cached;

            if (distance[i] <= minimum_distance || distance[i] >= 300) {
                obstacle_detected = true;
                break;
            }
        }
        if (obstacle_detected) {
            car_state.last_moove = STOP;
            stopp();
        } else {
            advance();
        }
    } else if(car_state.last_moove == STOP) {
        stopp();

        car_state.last_moove = STOP;

        for (int i = 0; i < sizeof(direction_to_check) / sizeof(direction_to_check[0]); i++) {
            servo_state = direction_to_check[i];
            myservo.write(servo_state.angle);
            servo_state.last_position = servo_state.angle;
            delay(150);

            long cached = get_cached_distance(servo_state.angle);
            if (cached == -1) {
                cached = measure_distance(nb_measure, servo_state.name);
                add_cached_distance(servo_state.angle, cached);
            }

            distance[i] = cached;
        }

        long max_dist = max(left_distance, right_distance);

        if (max_dist > minimum_distance) {
            if (max_dist == left_distance) turn_left(255);
            else if (max_dist == right_distance) turn_right(255);
            car_state.last_moove = ADVANCE;
        } else {
            Serial.println("Area blocked !");
            stopp();
            car_state.last_moove = STOP;
            cached_count = 0;
        }
    }
}