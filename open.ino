#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(13);          // Connects servo to pin 13
  Serial.begin(9600);          // Start serial at 9600 baud
  while (!Serial) {
    ; // Wait for Serial to become ready (especially for Leonardo/Micro)
  }
  Serial.println("Ready");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    Serial.print("Received: ");
    Serial.println(command);

    if (command == 'o') {
      Serial.println("Opening gripper...");
      for (int pos = 0; pos <= 180; pos++) {
        myServo.write(pos);
        delay(10);
      }
    } else if (command == 'c') {
      Serial.println("Closing gripper...");
      for (int pos = 180; pos >= 0; pos--) {
        myServo.write(pos);
        delay(10);
      }
    } else {
      Serial.println("Unknown command");
    }
  }
}
