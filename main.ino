
#include <Arduino.h>
#include <Wire.h>
#include <UCMotor.h>
#include <Servo.h>

UC_DCMotor leftMotorFront(3, MOTOR34_64KHZ);
UC_DCMotor rightMotorFront(4, MOTOR34_64KHZ);
UC_DCMotor leftMotorBack(1, MOTOR34_64KHZ);
UC_DCMotor rightMotorBack(2, MOTOR34_64KHZ);
Servo servo; 
void setup() {
  // put your setup code here, to run once:
    servo.attach(10);
    Serial.begin(9600);
}

#define TRIG_PIN A2
#define ECHO_PIN A3
int getUltrasonicVal(void)
  {
      unsigned char cnt = 0;
      long cm, beginTime, stopTime;
      long waitCount = 0;
      pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
      digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(5);
      digitalWrite(TRIG_PIN, LOW);  waitCount = 0;
      while (!(digitalRead(ECHO_PIN) == 1)) {
            if (++waitCount >= 30000)
              break;
      }
      beginTime = micros(); waitCount = 0;
      while (!(digitalRead(ECHO_PIN) == 0)) {
            if (++waitCount >= 30000)
              break;
      }
      stopTime = micros();
      cm  = (float)(stopTime - beginTime) / 1000000 * 34000 / 2; 
      return cm;
}

int readPing(void)
{
  int Res[3];
  int res;
  int maxvalue; 
  int minvalue;
  
  for(int i=0;i<3;i++)
  {
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

void setMotor(UC_DCMotor& motor, int16_t speed) {
    if(speed > 200) speed = 200;
    if(speed < -200) speed = -200;

    //2's complement, 1 is neg
    uint8_t sign = (speed >> 15) & 1;
    uint8_t abs_val = sign ? (uint8_t)(-speed) : (uint8_t)speed;

    motor.run(sign ? 0x02: 0x01);
    motor.setSpeed(abs_val);
}

void drive(int16_t speed) {
    setMotor(leftMotorFront, speed);
    setMotor(leftMotorBack, speed);
    setMotor(rightMotorFront, speed);
    setMotor(rightMotorBack, speed);
}

void rotateTank(int16_t angle) {
    if(angle > 360) angle = 360;
    if(angle < -360) angle = -360;
    //if 1 rotate left, 0 rotate right
    uint8_t sign = (angle >> 15) & 1;
    uint16_t abs_val = sign ? (-angle) : angle;

    //turn left
    if(sign) {
        setMotor(leftMotorFront, -200);
        setMotor(leftMotorBack, -200);
        setMotor(rightMotorFront, 200);
        setMotor(rightMotorBack, 200);
    }
    //turn right
    else {
        setMotor(leftMotorFront, 200);
        setMotor(leftMotorBack, 200);
        setMotor(rightMotorFront, -200);
        setMotor(rightMotorBack, -200);
    }

    //90: 475ms
    //180: 930ms
    //270: 1375ms
    //360: 1860ms
    //linear regression: a = 5.111111, 10
    uint16_t wait_time = (uint16_t)(5.111111 * (float)abs_val + 10);
    delay(wait_time);
    setMotor(leftMotorFront, 0);
    setMotor(leftMotorBack, 0);
    setMotor(rightMotorFront, 0);
    setMotor(rightMotorBack, 0);

    Serial.println(wait_time);
    
}

int currentSpeed;
int distance;

void loop() {
    distance = readPing();

    if(distance > 120) currentSpeed = 200;
    else if(distance > 80) currentSpeed = 150;
    else if(distance > 60) currentSpeed = 100;

    drive(currentSpeed);

    if(distance < 40) {
        drive(50);
        
        //check the right, check the left
        servo.write(0); delay(400);
        int distance_right = readPing();
        servo.write(180); delay(400);
        int distance_left = readPing();
        servo.write(90); delay(400);

        if(distance_right > distance_left) rotateTank(90);
        else rotateTank(-90);

    }
}