#include <SCServo.h>
#include <string>

#include "utils.h"
#include "servo.h"

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
  bool done = false;
  
  while (!done) {
    done = true;
    int move_delays[4] = {};
    for (Servo* s : servos) {
      int d = s->write_pos();
      move_delays[s->get_id() - 1] = d;
      
      if (d > 0) {
        done = false;
      }
    }
    
    int d = *std::max_element(move_delays, move_delays + 4);
    delay(d);

    int backoff_delays[4] = {};
    for (Servo* s : servos) {
      if (s->check_overload()) {
        backoff_delays[s->get_id() - 1] = s->backoff();
      }
    }
    d = *std::max_element(backoff_delays, backoff_delays + 4);
    delay(d);
  }



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

bool init_dock() {
  roll.dock();
  
  debugPrint(1, "Wrist min\n");
  wrist.dock(DockMode::Min);
  
  debugPrint(1, "Grip L(2) min_absolute, Grip R(3) max_absolute\n");
  gripL.dock(DockMode::absoluteMin);
  gripR.dock(DockMode::absoluteMax);
  
  debugPrint(1, "Grip L(2) max_relative, Grip R(3) min_relative\n");
  gripR.dock(DockMode::relativeMin);
  gripL.dock(DockMode::relativeMax);
  
  debugPrint(1, "Wrist max\n");
  wrist.dock(DockMode::Max);
  
  debugPrint(1, "Grip L(2) max_absolute , Grip R(3) min_absolute\n");
  gripR.dock(DockMode::absoluteMin);
  gripL.dock(DockMode::absoluteMax);
  
  debugPrint(1, "Grip L(2) min_relative , Grip R(3) max_relative\n");
  gripL.dock(DockMode::relativeMin);
  gripR.dock(DockMode::relativeMax);

  debugPrint(1, "Wrist mid\n");
  wrist.change_target(wrist.read_mid());
  if (!wrist.move_to_target(1000)) {return false;}
  
  debugPrint(1, "Grip L(2) mid , Grip R(3) mid\n");
  gripR.change_target(gripR.read_mid());
  gripL.change_target(gripL.read_mid());
  if (!gripR.move_to_target(1000)) {return false;}
  if (!gripL.move_to_target(1000)) {return false;}
  
  debugPrint(1, "Wrist max after 1 second\n");
  delay(1000);
  wrist.change_target(wrist.get_max());
  if (!wrist.move_to_target(1000)) {return false;}
  
  debugPrint(1, "Wrist min after 1 second\n");
  delay(1000);
  wrist.change_target(wrist.get_min());
  if (!wrist.move_to_target(1000)) {return false;}
  
  debugPrint(1, "Wrist mid after 1 second\n");
  delay(1000);
  wrist.change_target(wrist.read_mid());
  if (!wrist.move_to_target(1000)) {return false;}
  
  debugPrint(1, "Grip L(2) mid , Grip R(3) mid after 1 second\n");
  delay(1000);
  gripL.change_target(gripL.read_mid());
  gripR.change_target(gripR.read_mid());
  if (!gripL.move_to_target(1000)) {return false;}
  if (!gripR.move_to_target(1000)) {return false;}
  return true;
}