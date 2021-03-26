
#include <Arduino.h>
#include <Wire.h>
#include <UCMotor.h>
#include <Servo.h>

UC_DCMotor leftMotorFront(3, MOTOR34_64KHZ);
UC_DCMotor rightMotorFront(4, MOTOR34_64KHZ);
UC_DCMotor leftMotorBack(1, MOTOR34_64KHZ);
UC_DCMotor rightMotorBack(2, MOTOR34_64KHZ);
Servo myservo; 
void setup() {
  // put your setup code here, to run once:
    myservo.attach(10);
    Serial.begin(9600);
}
int servo_position = 0;

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

void drive(int16_t speed, int16_t angle) {
    
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


void loop() {
 /*for (servo_position = 0; servo_position <=180; servo_position +=10){

    myservo.write(servo_position);
    Serial.print(readPing());
    Serial.print(" ");
    Serial.print(myservo.read());
    Serial.println();
    delay(1000);
  }

  for (servo_position=180; servo_position >= 0; servo_position -=10){

    myservo.write(servo_position);
    Serial.print(readPing());
    Serial.print(" ");
    Serial.print(myservo.read());
    Serial.println();
    delay(1000);
  }*/


//1 is forward 2 is backward
/*setMotor(leftMotorFront, 100);
setMotor(leftMotorBack, 100);
setMotor(rightMotorFront, 100);
setMotor(rightMotorBack, 100);*/


rotateTank(45);
delay(5000);
//rotateTank(-180);
//delay(1000);

/*
  //Forward
       // leftMotor1.run(0x01); rightMotor1.run(0x01);
        leftMotor2.run(0x01); rightMotor2.run(0x01);
        leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
        leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
        delay(2000);
   //Backward
        leftMotor1.run(0x02); rightMotor1.run(0x02);
        leftMotor2.run(0x02); rightMotor2.run(0x02);
        leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
        leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
        delay(2000);
    //left
        leftMotor1.run(0x03); rightMotor1.run(0x03);
        leftMotor2.run(0x03); rightMotor2.run(0x03);
        leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
        leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
        delay(2000); 
    //Right
        leftMotor1.run(0x04); rightMotor1.run(0x04);
        leftMotor2.run(0x04); rightMotor2.run(0x04);
        leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
        leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
        delay(2000);      
*/
}