#include "calibration.h"
#include <SCServo.h>
#include "utils.h"

bool move_to_default(Servo* servos[], int count) {
    bool done = false;
    
    while (!done) {
        done = true;
        int move_delays[count] = {};
        for (int i = 0; i < count; i++) {
            Servo* s = servos[i];
            int d = s->write_pos();
            move_delays[s->get_id() - 1] = d;   
            
            if (d > 0) {
                done = false;
            }
        }
        
        int d = *std::max_element(move_delays, move_delays + count);
        delay(d);

        int backoff_delays[count] = {};
        for (int i = 0; i < count; i++) {
            Servo* s = servos[i];
            if (s->check_overload()) {
                backoff_delays[s->get_id() - 1] = s->backoff();
            }
        }
        d = *std::max_element(backoff_delays, backoff_delays + count);
        delay(d);
    }
    return true;
}

bool init_dock(Servo* servos[]) {
    Servo roll = *servos[0];
    Servo gripL = *servos[1];
    Servo gripR = *servos[2];
    Servo wrist = *servos[3];

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