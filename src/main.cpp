#include <SCServo.h>
#include <string>


#define SERVO_RX 16
#define SERVO_TX 17

SCSCL sc;

#define DEBUG_LEVEL 1
void debugPrint(int level, const char* format, ...)
{
    if (level > DEBUG_LEVEL)
        return;

    char buffer[256];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(buffer);
}

enum class DockMode {
    Min,
    Max,
    Null
};

class Servo{
  private:
    int id;
  protected:

    int default_pos = 511;
    int pos = default_pos;
    int last_pos = default_pos;
    int min_pos = 0;
    int max_pos = 1023;
    int mid = 511;
    int direction = 1;

    int load = 0;
    
    int target = default_pos;
    
    int MAX_LOAD = 500;
    int FEED_FORWARD = 0;
    
    static constexpr int default_speed = 1000;
    static constexpr int DEFAULT_STEP = 10;
    static constexpr int POSITION_TOLERANCE = DEFAULT_STEP;
    static constexpr int BACKOFF = 2 * DEFAULT_STEP;

  public:
    static constexpr int d45 = 153;   // int values of popular angles
    static constexpr int d90 = 307;   //
    static constexpr int d180 = 614;  //
    static constexpr int d300 = 1023; //
    
    Servo(int id) : id(id) {}

    int short_delay(int i, int speed) {
      return (i * 1000.0) / speed + 20;
    }
    int long_delay(int i, int speed) {
      return (i * 1000.0) / speed + 75;
    }

    int get_id() {return id;}

    int read_pos() {
      pos = sc.ReadPos(id);
      return pos;
    }

    void change_target(int t) {
      target = constrain(t, min_pos, max_pos);
    }

    int get_target() {return target;}

    void change_max_load(int l) {
      MAX_LOAD = l;
    }

    int get_min() {return min_pos;}
    int get_max() {return max_pos;}
    int update_mid() {
      mid = (max_pos + min_pos) / 2;
      debugPrint(2, "Servo [%d] mid updated to %d\n", id, mid);
      return mid;
    }

    bool clamp_pos(int &p) {
      if (p < min_pos) {
        p = min_pos;
        return true;
      } if (p > max_pos) {
        p = max_pos;
        return true;
      } return false;
    }

    bool check_overload(int max) {
      load = abs(sc.ReadLoad(id));
      if (load > max) {
        return true;
      }
      return false;
    }

    bool check_overload() {
      return check_overload(MAX_LOAD);
    }
    

    bool move_to_target(int speed, int step = DEFAULT_STEP) {
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
          debugPrint(1,
            "Servo [%d] overloaded at %d and couldnt be moved anymore\n",
            id, load);

          updatePos -= direction * BACKOFF;

          sc.WritePos(id, updatePos, 0, speed);
          delay(short_delay(BACKOFF, speed));

          return false;
        }

        pos = sc.ReadPos(id);   // real pos
      }

      return true;
    }

    int update_pos(int speed, int step = DEFAULT_STEP) {
      if (clamp_pos(target)) {
        debugPrint(1, "Servo [%d] target clamped\n", id);
      }

      if (pos == target) {
        last_pos = pos;
        return 1;
      }

      int direction = (target > pos) ? 1 : -1;
      if (abs(target - pos) <= step) {
        pos = target;
      } else {
        pos += step * direction;
      } 

      sc.WritePos(id, pos, 0, speed);
      delay(short_delay(abs(last_pos - pos), speed));

      
      if (check_overload()) {
        debugPrint(1, "Servo [%d] overloaded and couldnt be updated properly\n", id);
        pos -= direction * BACKOFF;
        last_pos = pos;
        sc.WritePos(id, pos, 0, speed);
        delay(short_delay(BACKOFF, speed));
        return -1;
      }
      debugPrint(2, "Servo %d: pos=%d target=%d load=%d\n", id, pos, target, load);
      last_pos = pos;
      return 0;
    }

    int write_pos(int speed = default_speed) { // moves the servo towards the target position by one step and returns the delay
      pos = sc.ReadPos(id);
      if (abs(pos - target) <= POSITION_TOLERANCE) {
        last_pos = pos;
        direction = 0;
        return 0; // already at target
      }

      direction = (target > pos) ? 1 : -1;
      int commandTarget = target + direction * FEED_FORWARD;
      int updatePos = pos;

      if (abs(commandTarget - pos) <= DEFAULT_STEP) {
        updatePos = commandTarget;
      } else {
        updatePos += DEFAULT_STEP * direction;
      }
      debugPrint(2, "Servo [%d] moving to %d with speed %d\n", id, updatePos, speed);
      sc.WritePos(id, updatePos, 0, speed);
      last_pos = pos;
      return short_delay(abs(pos - updatePos), speed);
    }

    int backoff() {
      int updatePos = pos - direction * BACKOFF;
      sc.WritePos(id, updatePos, 0, default_speed);
      return short_delay(BACKOFF, default_speed);
    }

    virtual bool dock(DockMode mode) {
      read_pos();
      direction = (mode == DockMode::Min) ? -1 : 1;

      for (int i = 0; i <= d300; i += DEFAULT_STEP) {
        pos += DEFAULT_STEP * direction;
        sc.WritePos(get_id(), pos, 0, default_speed);
        delay(short_delay(DEFAULT_STEP, default_speed));

        if (check_overload(MAX_LOAD - 100) || pos < BACKOFF || pos > 1023 - BACKOFF) {
          pos -= BACKOFF * direction;
          load = 0;
          sc.WritePos(get_id(), pos, 0, default_speed);
          delay(short_delay(BACKOFF, default_speed));
          debugPrint(1, "Servo [%d] docked at %d\n", get_id(), pos);

          if (mode == DockMode::Min) {
            min_pos = pos;
          } else {
            max_pos = pos;
          }

          return true;
        }
        last_pos = pos;
      }
      return false;
    }
};

class Roll : public Servo {
  public:
  Roll(int id) : Servo(id) {
    target = default_pos - d45;
  }

  bool dock(DockMode mode = DockMode::Null) override {
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
};

class Grip: public Servo {
  public:
  Grip(int id) : Servo(id) {
    FEED_FORWARD = 10;
    MAX_LOAD = 500;
  }
};

class Wrist: public Servo {
  public:
  Wrist(int id) : Servo(id) {
    FEED_FORWARD = 25;
    MAX_LOAD = 650;
  }
};

Roll roll(1);
Grip gripL(2);
Grip gripR(3);
Wrist wrist(4);
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



  delay(5000);

  // docking begins
  roll.dock();
  
  debugPrint(1, "Wrist min\n");
  wrist.dock(DockMode::Min);
  
  debugPrint(1, "Grip L(2) min_absolute, Grip R(3) max_absolute\n");
  gripL.dock(DockMode::Min);
  gripR.dock(DockMode::Max);
  
  debugPrint(1, "Grip L(2) max_relative, Grip R(3) min_relative\n");
  gripR.dock(DockMode::Min);
  gripL.dock(DockMode::Max);
  
  debugPrint(1, "Wrist max\n");
  wrist.dock(DockMode::Max);
  
  debugPrint(1, "Grip L(2) max_absolute , Grip R(3) min_absolute\n");
  gripR.dock(DockMode::Min);
  gripL.dock(DockMode::Max);
  
  debugPrint(1, "Grip L(2) min_relative , Grip R(3) max_relative\n");
  gripL.dock(DockMode::Min);
  gripR.dock(DockMode::Max);

  // debugPrint(1, "Grip R min, Grip L max\n");
  // gripR.dock(DockMode::Min);
  // gripL.dock(DockMode::Max);
  
  // wrist.dock(DockMode::Max);
  // wrist.change_target(wrist.update_mid());

  // if (!wrist.move_to_target(500)) {return;}
  // debugPrint(1, "Wrist pos is %d\n", wrist.read_pos());
  
  // debugPrint(1, "Grip R min, Grip L min\n");
  // gripR.dock(DockMode::Min);
  // gripL.dock(DockMode::Min);
  // gripL.dock(DockMode::Max);
  // gripL.change_target(gripL.get_min());
  // if (!gripL.move_to_target(500)) {return;}
  // gripR.dock(DockMode::Max);
  
  // gripR.change_target(gripR.update_mid());
  // gripL.change_target(gripL.update_mid());
  // if (!gripR.move_to_target(500)) {return;}
  // if (!gripL.move_to_target(500)) {return;}
  
  // if (!wrist.move_to_target(500)) {return;}
  // debugPrint(1, "Wrist pos is %d\n", wrist.read_pos());
}

void loop(){
}