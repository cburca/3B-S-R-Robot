#include <Servo.h>

#define SYRINGE_SERVO_PIN 12
#define GRIP_POS 45
#define UNGRIP_POS 180
#define GRIP_DELAY 700
#define UNGRIP_DELAY 700

#define LIFT_SERVO_PIN 13
#define PICK_POS 150
#define RAISE_POS 0
#define LOWER_DELAY 500
#define RAISE_DELAY 500


bool GripperRaised;
bool GripperClosed;


Servo SyringeServo;
Servo LiftServo;

void setup() {
  SyringeServo.attach(SYRINGE_SERVO_PIN);
  LiftServo.attach(LIFT_SERVO_PIN);

  Serial.begin(9600);
  while (!Serial) {;}

  LiftServo.write(RAISE_POS); 
  SyringeServo.write(UNGRIP_POS);

}

void loop() {

  Serial.println("Type U (up), D (down), G (grip), or R (release), then press Enter:");

  // Wait until something is typed
  while (Serial.available() == 0) {}

  char input = Serial.read();   // Read first character

  // Skip newline characters
  while (input == '\n' || input == '\r') {
    while (Serial.available() == 0) {}
    input = Serial.read();
  }

  if (input == 'G' || input == 'g') {
    SyringeServo.write(GRIP_POS);
    Serial.println("Actuating Gripper...");
  }
  else if (input == 'R' || input == 'r') {
    SyringeServo.write(UNGRIP_POS);
    Serial.println("Releasing Gripper...");
  }
  else if (input == 'U' || input == 'u') {
    LiftServo.write(RAISE_POS); //flip directions for continuous servo
    Serial.println("Raising Gripper...");
  }
  else if (input == 'D' || input == 'd') {
    LiftServo.write(PICK_POS);   //flip directions for continuous servo
    Serial.println("Lowering Gripper...");
  }
  else if (input == 'C' || input == 'c') {
    Serial.println("Cycling...");
    delay(1000);
    LiftServo.write(PICK_POS);     //Lower
    delay(LOWER_DELAY);
    SyringeServo.write(GRIP_POS);   //Grip
    delay(GRIP_DELAY);
    LiftServo.write(RAISE_POS);     //Raise
  }
  else if (input == 'M' || input == 'm') {        //'M' chosen as random key that's far from all the other ones
    for(int i = 0; i < 30; i++) {
      Serial.println("Cycle #");
      Serial.print(i);
      LiftServo.write(PICK_POS);     //Lower
      delay(LOWER_DELAY);
      SyringeServo.write(GRIP_POS);   //Grip
      delay(GRIP_DELAY);
      LiftServo.write(RAISE_POS);     //Raise
      delay(RAISE_DELAY);
      //LiftServo.write(LOWER_POS);   //Lower - If lowering before releasing
      //delay(LOWER_DELAY);
      SyringeServo.write(UNGRIP_POS); //Release
      delay(UNGRIP_DELAY);
      //LiftServo.write(RAISE_POS);         //Raise - If lowering before releasing
      //delay(RAISE_DELAY);
    }
  }
  else {
    Serial.println("Invalid input");
  }
}
