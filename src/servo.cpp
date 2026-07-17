#include "servo.h"
#include "utils.h"
#include <SCServo.h>

Servo::Servo(SCSCL& sc, int id) : sc(sc), id(id) {}

int Servo::short_delay(int i, int speed) {
    return (i * 1000.0) / speed + 20;
}

int Servo::long_delay(int i, int speed) {
    return (i * 1000.0) / speed + 75;
}

int Servo::get_id() {
    return id;
}

int Servo::read_pos() {
    pos = sc.ReadPos(id);
    return pos;
}

void Servo::change_target(int t) {
    target = constrain(t, min_pos, max_pos);
}

int Servo::get_target() {
    return target;
}

void Servo::change_max_load(int l) {
    MAX_LOAD = l;
}

int Servo::get_min() {
    return min_pos;
}

int Servo::get_max() {
    return max_pos;
}

int Servo::read_mid() {
    mid = (absolute_max + absolute_min) / 2;
    debugPrint(2, "Servo [%d] mid updated to %d\n", id, mid);
    return mid;
}

bool Servo::clamp_pos(int &p) {
    if (p < min_pos) {
        p = min_pos;
        return true;
    }

    if (p > max_pos) {
        p = max_pos;
        return true;
    }

    return false;
}

bool Servo::check_overload(int max) {
    load = abs(sc.ReadLoad(id));
    return load > max;
}

bool Servo::check_overload() {
    return check_overload(MAX_LOAD);
}

bool Servo::move_to_target(int speed, int step) {
    pos = sc.ReadPos(id);
    int updatePos = pos;

    if (abs(pos - target) <= POSITION_TOLERANCE)
        return true;

    direction = (target > pos) ? 1 : -1;
    int commandTarget = target + direction * FEED_FORWARD;

    while (abs(pos - target) > POSITION_TOLERANCE) {

        if (abs(updatePos - commandTarget) <= step)
            updatePos = commandTarget;
        else
            updatePos += step * direction;

        debugPrint(2, "Servo [%d] moving to %d with speed %d\n", id, updatePos, speed);

        sc.WritePos(id, updatePos, 0, speed);
        delay(short_delay(step, speed));

        if (check_overload()) {
            debugPrint(
                1,
                "Servo [%d] overloaded at %d and couldnt be moved anymore\n",
                id,
                load
            );

            updatePos -= direction * BACKOFF;

            sc.WritePos(id, updatePos, 0, speed);
            delay(short_delay(BACKOFF, speed));

            return false;
        }

        pos = sc.ReadPos(id);
    }

    return true;
}

int Servo::write_pos(int speed) {
    pos = sc.ReadPos(id);

    if (abs(pos - target) <= POSITION_TOLERANCE) {
        last_pos = pos;
        direction = 0;
        return 0;
    }

    direction = (target > pos) ? 1 : -1;
    int commandTarget = target + direction * FEED_FORWARD;
    int updatePos = pos;

    if (abs(commandTarget - pos) <= DEFAULT_STEP)
        updatePos = commandTarget;
    else
        updatePos += DEFAULT_STEP * direction;

    debugPrint(2, "Servo [%d] moving to %d with speed %d\n", id, updatePos, speed);

    sc.WritePos(id, updatePos, 0, speed);

    last_pos = pos;

    return short_delay(abs(pos - updatePos), speed);
}

int Servo::backoff() {
    int updatePos = pos - direction * BACKOFF;

    sc.WritePos(id, updatePos, 0, default_speed);

    return short_delay(BACKOFF, default_speed);
}

bool Servo::dock(DockMode mode = DockMode::Null) {
    if (mode == DockMode::Null) {return false;}
    read_pos();

    if (mode == DockMode::absoluteMin ||
        mode == DockMode::relativeMin ||
        mode == DockMode::Min) {
        direction = -1;
    } else {
        direction = 1;
    }

    int bound_load = MAX_LOAD;

    if (mode == DockMode::absoluteMin || mode == DockMode::absoluteMax) {
        bound_load -= 75;
        debugPrint(2, "Servo [%d] docking in absolute mode\n", get_id());
    }
    else if (mode == DockMode::relativeMin || mode == DockMode::relativeMax) {
        bound_load -= 125;
        debugPrint(2, "Servo [%d] docking in relative mode\n", get_id());
    }
    else {
        debugPrint(2, "Servo [%d] docking in normal mode\n", get_id());
    }

    for (int i = 0; i <= d300; i += DEFAULT_STEP) {
        pos += DEFAULT_STEP * direction;

        sc.WritePos(get_id(), pos, 0, default_speed);
        delay(short_delay(DEFAULT_STEP, default_speed));

        if (check_overload(bound_load) ||
            pos < BACKOFF ||
            pos > 1023 - BACKOFF) {

            pos -= BACKOFF * direction;

            load = 0;

            sc.WritePos(get_id(), pos, 0, default_speed);
            delay(short_delay(BACKOFF, default_speed));

            debugPrint(1, "Servo [%d] docked at %d\n", get_id(), pos);

            switch (mode) {
                case DockMode::absoluteMin:
                    absolute_min = pos;
                    min_pos = pos;
                    break;

                case DockMode::absoluteMax:
                    absolute_max = pos;
                    max_pos = pos;
                    break;

                case DockMode::relativeMin:
                    relative_min = pos;
                    min_pos = pos;
                    break;

                case DockMode::relativeMax:
                    relative_max = pos;
                    max_pos = pos;
                    break;

                case DockMode::Min:
                    absolute_min = pos;
                    relative_min = pos;
                    min_pos = pos;
                    break;

                case DockMode::Max:
                    absolute_max = pos;
                    relative_max = pos;
                    max_pos = pos;
                    break;

                default:
                    break;
            }

            return true;
        }

        last_pos = pos;
    }

    return false;
}

// CHILDREN
Roll::Roll(SCSCL& sc, int id) : Servo(sc, id) {
    target = default_pos - d45;
}

bool Roll::dock(DockMode mode = DockMode::Null) {
    for (int i = 0; i <= d90; i += DEFAULT_STEP) {
      sc.WritePos(get_id(), default_pos - d45 + i, 0, default_speed);
      delay(short_delay(DEFAULT_STEP, default_speed));

      if (check_overload()) { // ig safety for when a part would be blocked
        debugPrint(1, "Servo [%d] experienced unexpected load of %d is probably docked\n", get_id(), load);
        break;
      } 
      debugPrint(2, "Servo [%d] with load %d\n", get_id(), load);
    }
    sc.WritePos(get_id(), default_pos, 0, default_speed);
    delay(short_delay(d45, default_speed));
    debugPrint(1, "Docked servo [%d]\n", get_id());
    return true;
}


Grip::Grip(SCSCL& sc, int id) : Servo(sc, id) {
    FEED_FORWARD = 15;
    MAX_LOAD = 500;
}


Wrist::Wrist(SCSCL& sc, int id) : Servo(sc ,id) {
    FEED_FORWARD = 25;
    MAX_LOAD = 650;
    BACKOFF = 5 * DEFAULT_STEP;
}