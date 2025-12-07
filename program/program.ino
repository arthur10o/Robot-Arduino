#include <Servo.h>
#include <IRremote.h>

// ============================= PIN CONFIGURATION ============================= //

const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 12;
const int IN4 = 13;
const int ENA = 5;
const int ENB = 2;

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// ============================= DEFINITION OF CAR STATES ============================= //

enum CarDirection {
    STOP = 0,
    ADVANCE = 1,
    BACKWARD = -1,
    TURN_RIGHT = 2,
    TURN_LEFT = 3
};

// ============================= DATA STRUCTURES ============================= //

struct WheelState {
    int speed;              // speed in percentage from 0 to 255
    int direction;          // 1 = forward, -1 = backward, 0 = stop
};

struct CarState {
    int car_direction;      // Global direction of the car (STOP, ADVANCE, TURN LEFT, TURN RIGHT)
    int time_direction;     // Time duration for the current direction
    WheelState left;
    WheelState right;
};

struct DirectionServoMotor {
    const char* name;
    int angle;
    int last_position;
};

struct ServoMotor {
    DirectionServoMotor current_servo_state;
    long left_distance, right_distance;
    long slight_midle_left_distance, slight_midle_right_distance;
    long slight_left_distance, slight_right_distance;
};

// ============================= OBJECTS AND GLOBAL VARIABLES ============================= //

Servo my_servo;
ServoMotor my_servo_motor;
CarState car_state;

const DirectionServoMotor FRONT = {"front", 120};
const DirectionServoMotor RIGHT = {"right", 50};
const DirectionServoMotor LEFT = {"left", 190};
const DirectionServoMotor SLIGHT_MIDLE_LEFT = {"slight_midle_left", 133};
const DirectionServoMotor SLIGHT_MIDLE_RIGHT = {"slight_midle_right", 96};
const DirectionServoMotor SLIGHT_LEFT = {"slight_left", 166};
const DirectionServoMotor SLIGHT_RIGHT = {"slight_right", 73};

const DirectionServoMotor SERVO_MOTOR_DIRECTIONS[] = {
    RIGHT,
    SLIGHT_RIGHT,
    SLIGHT_MIDLE_RIGHT,
    FRONT,
    SLIGHT_MIDLE_LEFT,
    SLIGHT_LEFT,
    LEFT
};

const int direction_number = sizeof(SERVO_MOTOR_DIRECTIONS) / sizeof(SERVO_MOTOR_DIRECTIONS[0]);
int list_of_distances[direction_number];

int measure_number = 10;
int minimum_distance = 60;          // distance in cm at which robot must stop

const int MAX_CACHED_ANGLES = 10;
int cached_angles[MAX_CACHED_ANGLES];
long cached_distances[MAX_CACHED_ANGLES];
int cached_count = 0;

float kalman_filter_estimate = 0;
float estimate_covariance = 1;
float process_noise = 0.1;
float measurement_noise = 1.0;

// ============================= UTILITY FUNCTIONS ============================= //

void set_wheel_state(WheelState &_wheel, int _speed, int _direction) {
    /**
    * Sets the speed and direction of a wheel.
    * @param _wheel : Reference to the WheelState object to update.
    * @param _speed : Speed value to set.
    * @param _direction : Direction value to set.
    */
    _wheel.speed = _speed;
    _wheel.direction = _direction;
}

void set_car_speed(int _speed_left, int _speed_right) {
    /**
    * Sets the speed of the car by updating the speed of each wheel.
    * @param _speed_left : Speed for the left wheel.
    * @param _speed_right : Speed for the right wheel.
    */
    set_wheel_state(car_state.left, _speed_left, car_state.left.direction);
    set_wheel_state(car_state.right, _speed_right, car_state.right.direction);
}

void set_car_direction(int _left_direction, int _right_direction, int _direction, int _time) {
    /**
    * Sets the overall direction of the car and updates both wheels accordingly.
    * @param _left_direction : The new direction for the left wheel.
    * @param _right_direction : The new direction for the right wheel.
    * @param _direction : The new direction for the car.
    * @param _time : Duration for which the car should maintain this direction.
    */
    car_state.car_direction = _direction;
    car_state.time_direction = _time;
    set_wheel_state(car_state.left, car_state.left.speed, _left_direction);
    set_wheel_state(car_state.right, car_state.right.speed, _right_direction);
}

void move_car() {
    /**
    * Executes the movement of the car based on its current direction state.
    * Calls the appropriate movement function (advance, stop, turn left/right, backward).
    */
    if (car_state.car_direction == ADVANCE) {
        advance(car_state.left.speed, car_state.right.speed);
    } else if (car_state.car_direction == STOP) {
        stopp();
    } else if (car_state.car_direction == BACKWARD) {
        backward(car_state.left.speed, car_state.right.speed);
    } else if (car_state.car_direction == TURN_LEFT) {
        turn_left(car_state.time_direction, car_state.left.speed, car_state.right.speed);
    } else if (car_state.car_direction == TURN_RIGHT) {
        turn_right(car_state.time_direction, car_state.left.speed, car_state.right.speed);
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
    Serial.println("STOP");
}

void advance(int _left_speed, int _right_speed) {
    /**
    * Moves the robot forward by activing both motors in forward direction.
    * @param _left_speed : Speed for the left motor (0-255).
    * @param _right_speed : Speed for the right motor (0-255).
    */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, _left_speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, _right_speed);
    Serial.println("ADVANCE");
}

void backward(int _left_speed, int _right_speed) {
    /**
    * Moves the robot backwards for a given time.
    * @param _left_speed : Speed for the left motor (0-255).
    * @param _right_speed : Speed for the right motor (0-255).
    */
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, _left_speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, _right_speed);
    stopp();
    Serial.println("BACKWARD");
}

void turn_right(int _time, int _left_speed, int _right_speed) {
    /**
    * Rotates the robot to the right by running the motors in opposite directions.
    * A short delay is used to complete the turn before stopping.
    * @param _time : Rotation time in milliseconds. The higher the value, the longer the robot rotates.
    * @param _left_speed : Speed for the left motor (0-255).
    * @param _right_speed : Speed for the right motor (0-255).
    */
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, _left_speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, _right_speed);
    delay(_time);
    Serial.println("TURN RIGHT");
    stopp();
}

void turn_left(int _time, int _left_speed, int _right_speed) {
    /**
    * Rotates the robot to the left by running the motors in opposite directions.
    * A short delay is used to complete the turn before stopping.
    * @param _time : Rotation time in milliseconds. The higher the value, the longer the robot rotates.
    * @param _left_speed : Speed for the left motor (0-255).
    * @param _right_speed : Speed for the right motor (0-255).
    */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, _left_speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, _right_speed);
    delay(_time);
    Serial.println("TURN LEFT");
    stopp();
}

float kalman_filter(float _measurement) {
    /**
    * Applies a Kalman filter to smooth out the distance measurements from the ultrasonic sensor.
    * This helps reduce noise and improve accuracy of the readings.
    * @param _measurement : The raw distance measurement from the sensor.
    * @return The filtered distance estimate.
    */
    float prediction_estimate = kalman_filter_estimate;
    float kalman_gain = estimate_covariance / (estimate_covariance + measurement_noise);
    kalman_filter_estimate = prediction_estimate + kalman_gain * (_measurement - prediction_estimate);
    estimate_covariance = (1 - kalman_gain) * estimate_covariance + process_noise;
    return kalman_filter_estimate;
}

long quick_select(long _array[], int _left, int _right, int _k) {
    /**
    * Selects the k-th smallest element from the array using the Quickselect algorithm.
    * This is used for efficiently finding the median in unsorted data.
    *
    * @param _array : The array to search.
    * @param _left : Starting index of the subarray.
    * @param _right : Ending index of the subarray.
    * @param _k : The index (0-based) of the desired smallest element.
    * @return The k-th smallest element in the array.
    */
    if (_left == _right) return _array[_left];
    int pivot_index = partition(_array, _left, _right);
    if (_k == pivot_index) return _array[_k];
    else if (_k < pivot_index) return quick_select(_array, _left, pivot_index - 1, _k);
    else return quick_select(_array, pivot_index + 1, _right,  _k);
}

static int partition(long _array[], int _left, int _right) {
    /**
    * Partitions an array around a pivot for use in the quickselect algorithm.chached_count
    * Elements smaller than the pivot are moved to the left, and larger to the right.
    *
    * @param _array : The array to partition.
    * @param _left  : Starting index of the subarray.
    * @param _right : Ending index of the subarray (pivot element).
    * @return The index position of the pivot after partitioning.
    */
    long pivot = _array[_right];
    int i = _left;
    for (int j = _left; j < _right; j++) {
        if (_array[j] <= pivot) {
            long temp = _array[i];
            _array[i] = _array[j];
            _array[j] = temp;
            i++;
        }
    }
    long temp = _array[i];
    _array[i] = _array[_right];
    _array[_right] = temp;
    return i;
}

long measures_distance(int _measure_number, const char* _direction) {
    /**
    * Measure the distance between the robot and an obstacle using an ultrasonic sensor.
    * The function performs multiple measurements and returns the average value.
    *
    * @param _measure_number : Number of individual distance measurements to average.
    * @param _direction : Direction label.
    * @return median in centimeters (as a long integer).
    */
    long measures[_measure_number];
    for (int i = 0; i < _measure_number; i++) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        long duration = pulseIn(ECHO_PIN, HIGH, 25000);
        if (duration == 0) measures[i] = 999;
        else measures[i] = 0.0171 * duration;
    }

    long median;

    if (_measure_number % 2 == 0) {
        long first_median = quick_select(measures, 0, _measure_number - 1, _measure_number / 2);
        long second_median = quick_select(measures, 0, _measure_number - 1, _measure_number / 2 - 1);
        median = (first_median + second_median) / 2;
    } else {
        median = quick_select(measures, 0, _measure_number - 1, _measure_number / 2);
    }

    if (median == 0) {
        Serial.print("Error: no signal for ");
        Serial.println(_direction);
        return 0;
    }

    return median;
}

void update_direction_based_on_distance() {
    /**
    * Updates the car's movement direction based on the distance measured in the current servo direction.
    * If an obstacle is detected within the minimum distance, the car stops; otherwise, it continues to advance.
    */
    long distance = measures_distance(measure_number, my_servo_motor.current_servo_state.name);
    distance = kalman_filter(distance);
    Serial.print("Distance at ");
    Serial.print(my_servo_motor.current_servo_state.name);
    Serial.print(": ");
    Serial.println(distance);
    if (distance <= minimum_distance) {
        Serial.println("Obstacle detected! Stopping the car.");
        car_state.car_direction = STOP;
        move_car();

    } else {
        Serial.println("Path clear. Continuing to advance.");
        car_state.car_direction = ADVANCE;
        car_state.left.speed = 200;
        car_state.right.speed = 200;
        move_car();
    }
}

void add_cached_distance(int _angle, long _distance) {
    /**
    * Stores a new angle-distance pair into the cache to avoid remeasuring.
    * The cache has a limited size and does not overwrite old values.
    *
    * @param _angle : Servo angle to cache.
    * @param _distance : Distance measured at that angle.
    */
    if (cached_count < MAX_CACHED_ANGLES) {
        cached_angles[cached_count] = _angle;
        cached_distances[cached_count] = _distance;
        cached_count++;
    }
}

long get_cached_distance(int _angle) {
    /**
    * Retrieves a previously measured and cached distance for a given servo angle.
    * This helps avoid redundant ultrasonic measurements during a scan.
    *
    * @param _angle : Servo angle for which to retrieve the distance.
    * @return Cached distance in centimeters if found; -1 otherwise.
    */
    for (int i = 0; i < cached_count; i++) {
        if (cached_angles[i] == _angle) {
            return cached_distances[i];
        }
    }
    return -1;
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

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    my_servo_motor.current_servo_state = FRONT;

    my_servo.attach(11);
    my_servo.write(my_servo_motor.current_servo_state.angle);

    car_state.car_direction = STOP;

    delay(1000);
}

// ============================= MAIN FUNCTION ============================= //

void loop() {
    /**
    * Main control loop of the robot.
    * If moving forward, constantly checks distances to obstacles.
    * If stopped, scans surroundings and chooses a direction with the most space to resume movement.
    */
    update_direction_based_on_distance();
    delay(150);
    if (car_state.car_direction == ADVANCE) {
        bool obstacle_detected = false;
        cached_count = 0;
        for (int i = 0; i < direction_number; i++) {
            my_servo_motor.current_servo_state = SERVO_MOTOR_DIRECTIONS[i];
            my_servo.write(my_servo_motor.current_servo_state.angle);
            my_servo_motor.current_servo_state.last_position = my_servo_motor.current_servo_state.angle;
            delay(250);

            long cached = get_cached_distance(my_servo_motor.current_servo_state.angle);
            if (cached == -1) {
                cached = measures_distance(measure_number / 5, my_servo_motor.current_servo_state.name);
                cached = kalman_filter(cached);
                add_cached_distance(my_servo_motor.current_servo_state.angle, cached);
            }
            list_of_distances[i] = cached;

            if (list_of_distances[i] <= minimum_distance || list_of_distances[i] >= 300 || list_of_distances[i] == 0) {
                obstacle_detected = true;
                break;
            }
        }
        if (obstacle_detected) {
            car_state.car_direction = STOP;
        }
        move_car();
   } else  if (car_state.car_direction == STOP) {
        move_car();
        car_state.car_direction = STOP;

        for (int i = 0; i < direction_number; i++) {
            my_servo_motor.current_servo_state = SERVO_MOTOR_DIRECTIONS[i];
            my_servo.write(my_servo_motor.current_servo_state.angle);
            my_servo_motor.current_servo_state.last_position = my_servo_motor.current_servo_state.angle;

            long cached = get_cached_distance(my_servo_motor.current_servo_state.angle);
            if (cached == -1) {
                cached = measures_distance(measure_number, my_servo_motor.current_servo_state.name);
                cached = kalman_filter(cached);
                add_cached_distance(my_servo_motor.current_servo_state.angle, cached);
            }
            list_of_distances[i] = cached;
        }

        my_servo_motor.right_distance = list_of_distances[0];
        my_servo_motor.slight_right_distance = list_of_distances[1];
        my_servo_motor.slight_midle_right_distance = list_of_distances[2];
        my_servo_motor.slight_left_distance = list_of_distances[5];
        my_servo_motor.slight_midle_left_distance = list_of_distances[4];
        my_servo_motor.left_distance = list_of_distances[6];

        long max_distance = max(max(my_servo_motor.left_distance, my_servo_motor.right_distance),
                            max(max(my_servo_motor.slight_left_distance, my_servo_motor.slight_right_distance),
                            max(my_servo_motor.slight_midle_left_distance, my_servo_motor.slight_midle_right_distance)));
        
        if (max_distance > minimum_distance) {
            if (max_distance == my_servo_motor.left_distance) turn_left(car_state.time_direction, 255, 255);
            else if (max_distance == my_servo_motor.slight_left_distance) turn_left(car_state.time_direction, 150, 150);
            else if (max_distance == my_servo_motor.slight_midle_left_distance) turn_left(car_state.time_direction, 100, 100);
            else if (max_distance == my_servo_motor.right_distance) turn_right(car_state.time_direction, 255, 255);
            else if (max_distance == my_servo_motor.slight_right_distance) turn_right(car_state.time_direction, 150, 150);
            else if (max_distance == my_servo_motor.slight_midle_right_distance) turn_right(car_state.time_direction, 100, 100);
            car_state.car_direction = ADVANCE;
        } else {
            car_state.car_direction = STOP;
            cached_count = 0;
        }
        move_car();
    }
}