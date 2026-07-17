#pragma once

#include <SCServo.h>

enum class DockMode {
    Min,
    Max,
    absoluteMin,
    absoluteMax,
    relativeMin,
    relativeMax,
    Null
};

class Servo {
private:
    int id;
protected:
    SCSCL& sc;

    int default_pos = 511;
    int pos = default_pos;
    int last_pos = default_pos;

    int min_pos = 0;
    int max_pos = 1023;
    int absolute_min = 0;
    int absolute_max = 1023;
    int relative_min = 0;
    int relative_max = 1023;

    int mid = 511;
    int direction = 1;

    int load = 0;
    
    int target = default_pos;
    
    int MAX_LOAD = 500;
    int FEED_FORWARD = 0;
    int BACKOFF = 2 * DEFAULT_STEP;
    
    static constexpr int default_speed = 1000;
    static constexpr int DEFAULT_STEP = 10;
    static constexpr int POSITION_TOLERANCE = DEFAULT_STEP;

public:
    static constexpr int d45 = 153;   // int values of popular angles
    static constexpr int d90 = 307;   //
    static constexpr int d180 = 614;  //
    static constexpr int d300 = 1023; //

    Servo(SCSCL& sc, int id);

    int short_delay(int i, int speed);
    int long_delay(int i, int speed);

    int get_id();

    int read_pos();

    int get_target();
    void change_target(int t);

    void change_max_load(int l);

    int get_min();
    int get_max();
    int read_mid();

    bool clamp_pos(int &p);

    bool check_overload(int max);
    bool check_overload();

    bool move_to_target(int speed, int step = DEFAULT_STEP); // non async method
    int write_pos(int speed = default_speed); // moves the servo by one step. No delays, for async.
    int backoff(); // backs off the servo by const BACKOFF, opposite to the last udpated direction


    virtual bool dock(DockMode mode = DockMode::Null);
};

class Roll : public Servo {
public:
    Roll(SCSCL& sc, int id);
    bool dock(DockMode mode = DockMode::Null) override;
};

class Grip: public Servo {
public:
    Grip(SCSCL& sc, int id);
};

class Wrist: public Servo {
    public:
    Wrist(SCSCL& sc, int id) : Servo(sc ,id) {
        FEED_FORWARD = 25;
        MAX_LOAD = 650;
        BACKOFF = 5 * DEFAULT_STEP;
    }
};