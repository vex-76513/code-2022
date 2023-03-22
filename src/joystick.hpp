#include "main.h"
#pragma once

class Joystick {
  okapi::Controller c;
  okapi::ControllerAnalog j;

public:
  Joystick(okapi::ControllerAnalog j_id,
           okapi::Controller c_id = okapi::ControllerId::master)
      : c(c_id), j(j_id){};

  double get() {
    const double threshold = 0.001;

    double val = get_raw();
    if (std::abs(val) < threshold) return 0;
    return std::clamp(val, -1.0, 1.0);
  }

  double get_raw() { return c.getAnalog(j); }
};

class IntakeButton {
  okapi::ControllerButton b;
  int pressedCount = 0;
  long lastPressedTime;
  int current_direction = 0;

  bool pressed() { return b.isPressed(); }

  bool longPressed(int howLong = 30) {
    if (pressed()) pressedCount++;
    else pressedCount = 0;

    return pressedCount > howLong;
  }

public:
  IntakeButton(okapi::ControllerDigital iB) : b(iB) {}

  int intakeDirection() {
    // if long pressed, set to go backwards
    if (longPressed()) current_direction = -1;
    // if just pressed, flip from on to off, off to on
    else if (b.changedToPressed()) current_direction = !current_direction;

    return current_direction;
  }
  void off() { current_direction = 0; }

  bool pressedWithinLast(okapi::QTime t) {
    if (pressed()) lastPressedTime = pros::millis();
    return pros::millis() - lastPressedTime < t.convert(1_ms);
  }
};

class PastNLessThanFilter {
  int count;
  double threshold;

public:
  PastNLessThanFilter(double iThresh) : threshold(iThresh){};

  bool get(double i, int minCount) {
    if (i < threshold) count++;
    else count = 0;
    return count > minCount;
  }
};

#define CONCAT(a, b) CONCAT_INNER(a, b)
#define CONCAT_INNER(a, b) a##b
#define UNIQUENAME(base) CONCAT(base, __LINE__)

#define DEBUGPRINT(COUNT, FORMAT_STRING, ...)                                  \
  static long UNIQUENAME(debug_print_count) = 0;                               \
  UNIQUENAME(debug_print_count)++;                                             \
  if (UNIQUENAME(debug_print_count) % COUNT != 0)                              \
  printf(FORMAT_STRING, __VA_ARGS__)

#define LEN(a) (int)(sizeof(a) / sizeof(a[0]))

int _arr3 []= {1, 2, 3};
float _arr4f[] = {1, 2, 3, 4};
static_assert(LEN(_arr3) == 3, "NO LEN");
static_assert(LEN(_arr4f) == 4, "NO LEN");