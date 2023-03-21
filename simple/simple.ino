#include <Servo.h>

Servo myservo;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  myservo.attach(9);
  Serial.begin(9600);
}

int counter = 0;

void loop() {
  counter++;
  
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);

  myservo.write(180);
  delay(2000);
  myservo.write(0);
}
