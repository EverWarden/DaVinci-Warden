#include <SCServo.h>
#include <string>

#include "utils.h"
#include "servo.h"
#include "calibration.h"

#define SERVO_RX 16
#define SERVO_TX 17

SCSCL sc;


bool init_dock();

Roll roll(sc, 1);
Grip gripL(sc, 2);
Grip gripR(sc, 3);
Wrist wrist(sc, 4);
Servo* servos[] = {
  &roll,
  &gripL,
  &gripR,
  &wrist
};

void setup()
{
  Serial.begin(115200);                                   // Debug serial monitor
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX); // Servo UART
  sc.pSerial = &Serial1;

  delay(1000);
  Serial.println("Start");
  

  // initialization of the servos to their default position
  move_to_default(servos, 4);


  for (int i = 0; i < 5; i++) {
    debugPrint(1, "Docking in %d seconds\n", 5 - i);
    delay(1000);
  }

  // docking begins
  if(!init_dock()) {
    debugPrint(1, "Docking failed\n");
  } else {
    debugPrint(1, "Docking successful\n");
  }
}

void loop(){
}