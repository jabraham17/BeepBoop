
#include <Arduino.h>
#include <Wire.h>
#include <UCMotor.h>
#include <Servo.h>

UC_DCMotor leftMotor1(3, MOTOR34_64KHZ);
UC_DCMotor rightMotor1(4, MOTOR34_64KHZ);
UC_DCMotor leftMotor2(1, MOTOR34_64KHZ);
UC_DCMotor rightMotor2(2, MOTOR34_64KHZ);
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

void loop() {
 for (servo_position = 0; servo_position <=180; servo_position +=10){

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
  }


  //Forward
       /* leftMotor1.run(0x01); rightMotor1.run(0x01);
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