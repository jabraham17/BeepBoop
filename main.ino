
#include <Arduino.h>
#include <Servo.h>
#include <UCMotor.h>
#include <Wire.h>

UC_DCMotor leftMotorFront(3, MOTOR34_64KHZ);
UC_DCMotor rightMotorFront(4, MOTOR34_64KHZ);
UC_DCMotor leftMotorBack(1, MOTOR34_64KHZ);
UC_DCMotor rightMotorBack(2, MOTOR34_64KHZ);
Servo sensorServo;
Servo clawServo;

#define TRIG_PIN A2
#define ECHO_PIN A3
#define SENSOR_SERVO_PIN 10
#define CLAW_SERVO_PIN 13

// 1 is closed, 0 is open
int clawState;
void clawOpen() { clawServo.write(40); }
void clawClose() { clawServo.write(107); }
void clawToggle() {
    if(clawState == 0)
        clawClose();
    else
        clawOpen();
    clawState = !clawState;
}

float getUltrasonicVal() {
    unsigned char cnt = 0;
    long cm, beginTime, stopTime;
    long waitCount = 0;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, LOW);
    waitCount = 0;
    while(!(digitalRead(ECHO_PIN) == 1)) {
        if(++waitCount >= 30000)
            break;
    }
    beginTime = micros();
    waitCount = 0;
    while(!(digitalRead(ECHO_PIN) == 0)) {
        if(++waitCount >= 30000)
            break;
    }
    stopTime = micros();
    cm = (float)(stopTime - beginTime) / 1000000.0f * 34000.0f / 2.0f;
    return cm;
}

// https://www.researchgate.net/publication/330540064_Kalman_Filter_Algorithm_Design_for_HC-SR04_Ultrasonic_Sensor_Data_Acquisition_System
float K = 0.0; // Gain Parameter
float P = 1.0; // State Variance
float R = 4.0; // Sensor varaiance
float distance;
float rawDistance;
void kalman() {
    float kt = P/(P+R);
    distance = distance + kt*(rawDistance - distance);
    P = (1-kt) * P;
}
void updateDistance() {
    // apply KF here
    rawDistance = getUltrasonicVal();
    distance = distance * .7 + rawDistance * .3;
    Serial.print(rawDistance);
    Serial.print(", ");
    Serial.print(distance);
    Serial.println();
}

int readPing() {
    int Res[3];
    int res;
    int maxvalue;
    int minvalue;

    for(int i = 0; i < 3; i++) {
        Res[i] = getUltrasonicVal();
        delay(10);
    }
    maxvalue = Res[0] >= Res[1] ? Res[0] : Res[1];
    maxvalue = maxvalue >= Res[2] ? maxvalue : Res[2];
    minvalue = Res[0] <= Res[1] ? Res[0] : Res[1];
    minvalue = minvalue <= Res[2] ? minvalue : Res[2];
    res = Res[0] + Res[1] + Res[2] - maxvalue - minvalue;
    return res;
}

void setMotor(UC_DCMotor &motor, int16_t speed) {
    if(speed > 200)
        speed = 200;
    if(speed < -200)
        speed = -200;

    // 2's complement, 1 is neg
    uint8_t sign = (speed >> 15) & 1;
    uint8_t abs_val = sign ? (uint8_t)(-speed) : (uint8_t)speed;

    motor.run(sign ? 0x02 : 0x01);
    motor.setSpeed(abs_val);
}

void drive(int16_t speed) {
    setMotor(leftMotorFront, speed);
    setMotor(leftMotorBack, speed);
    setMotor(rightMotorFront, speed);
    setMotor(rightMotorBack, speed);
}

void rotateTank(int16_t angle) {
    if(angle > 360)
        angle = 360;
    if(angle < -360)
        angle = -360;
    // if 1 rotate left, 0 rotate right
    uint8_t sign = (angle >> 15) & 1;
    uint16_t abs_val = sign ? (-angle) : angle;

    // turn left
    if(sign) {
        setMotor(leftMotorFront, -200);
        setMotor(leftMotorBack, -200);
        setMotor(rightMotorFront, 200);
        setMotor(rightMotorBack, 200);
    }
    // turn right
    else {
        setMotor(leftMotorFront, 200);
        setMotor(leftMotorBack, 200);
        setMotor(rightMotorFront, -200);
        setMotor(rightMotorBack, -200);
    }

    // ON JACOBS FLOOR
    // 90: 475ms
    // 180: 930ms
    // 270: 1375ms
    // 360: 1860ms
    // linear regression: a = 5.111111, b = 10
    // uint16_t wait_time = (uint16_t)(5.111111 * (float)abs_val + 10);

    // ON LAURENS FLOOR
    // 90: 410ms
    // 180: 775ms
    // 270: 1080ms
    // 360: 1450ms
    // linear regression: a = 3.80556, b = 72.5
    uint16_t wait_time = (uint16_t)(3.80556 * (float)abs_val + 72.5);
    // uint16_t wait_time = 775;
    delay(wait_time);
    setMotor(leftMotorFront, 0);
    setMotor(leftMotorBack, 0);
    setMotor(rightMotorFront, 0);
    setMotor(rightMotorBack, 0);
}

int currentSpeed;

void setup() {
    Serial.begin(9600);
    sensorServo.attach(10);
    // clawServo.attach(13);
    // clawOpen(); clawState = 0;

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // dummy values
    distance = 60;
    rawDistance = 60;
}

void loop() {

    updateDistance();

    if(distance > 100)
        currentSpeed = 200;
    else if(distance > 80)
        currentSpeed = 125;
    else
        currentSpeed = 100;

    drive(currentSpeed);

    if(distance < 35) {

        drive(40);
        // clawOpen();
        // while(distance > 16 && distance < 80) {updateDistance();}
        // drive(0);
        // clawClose();
        // delay(400);

        // drive(-currentSpeed);
        // delay(400);
        // drive(0);

        // check the right, check the left
        sensorServo.write(0);
        delay(400);
        int distance_right = readPing();
        sensorServo.write(180);
        delay(400);
        int distance_left = readPing();
        sensorServo.write(90);
        delay(400);

        if(distance_left < 35 && distance_right < 35) {
            rotateTank(180);
        } else {
            if(distance_right > distance_left)
                rotateTank(90);
            else
                rotateTank(-90);
        }
    }
}
