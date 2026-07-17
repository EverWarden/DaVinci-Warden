#pragma once

#include <SCServo.h>
#include <servo.h>

bool move_to_default(Servo* servos[], int count);

bool init_dock(Servo* servos[], int count);