#include "main.h"
#include "drivetrain.hpp"
#include "intake.hpp"
#include "joystick.hpp"

#include "autonomous.hpp"
#include "pal/auto.h"

using namespace okapi::literals;

okapi::OdomState state;
pros::c::adi_encoder_t yenc;
pros::c::adi_encoder_t xenc;
std::shared_ptr<okapi::XDriveModel> xdrive;
std::shared_ptr<okapi::ChassisController> chassis;

pros::Motor flywheel_motor(FLYWHEEL_PORT, pros::E_MOTOR_GEARSET_36, true);
pros::Motor flywheel_motor2(FLYWHEEL_PORT2, pros::E_MOTOR_GEARSET_36, false);
const bool DEFAULT_INDEXER_STATE = false;

//

void flipflopindexer_task() {
  pros::ADIDigitalOut indexer('D', DEFAULT_INDEXER_STATE);
  while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
    indexer.set_value(!DEFAULT_INDEXER_STATE);
    pros::delay(325);
    indexer.set_value(DEFAULT_INDEXER_STATE);
    pros::delay(175);
  }
}
pros::Task indexer_task(flipflopindexer_task);

const int ROLLER_COLOR_RED = 10, ROLLER_COLOR_BLUE = 240;
// selector
void auto_roller() {
  const int ACCEPTABLE_DIFFERENCE = 10;
  const float TIMEOUT_SECS = 1;

  okapi::OpticalSensor opt(16);
  opt.setLedPWM(100);
  while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
    int WANT = 0;
    if (auto_get_color() == AUTO_COLOR_BLUE) {
      WANT = ROLLER_COLOR_BLUE;
    } else WANT = ROLLER_COLOR_RED; // red on skills, red match, unset

    Intake.lock();
    auto expires = pros::millis() + 1000 * TIMEOUT_SECS;

    Intake.intake_motors.move_velocity(80);
    pros::delay(250);
    while (std::abs(opt.getHue() - WANT) > ACCEPTABLE_DIFFERENCE &&
           pros::millis() < expires) {
      Intake.intake_motors.move_velocity(80);
      pros::delay(10);
    }

    expires = pros::millis() + 1000 * TIMEOUT_SECS;
    while (std::abs(opt.getHue() - WANT) > ACCEPTABLE_DIFFERENCE &&
           pros::millis() < expires) {
      Intake.intake_motors.move_velocity(-80);
      pros::delay(10);
    }
    pros::delay(150);
    Intake.intake_motors.brake();
    Intake.unlock();
  }
}
pros::Task auto_roller_task(auto_roller);


void lcd_auton() {
  pros::lcd::shutdown();
  auto_picker(auto_list, sizeof(auto_list) / sizeof(auto_routine_t));
}

void lcd_driver() {
  auto_clean();
  pros::lcd::initialize();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  okapi::IMU(IMU_PORT).calibrate();
  xenc = pros::c::adi_encoder_init('G', 'H', false);
  yenc = pros::c::adi_encoder_init('E', 'F', false);

  const okapi::AbstractMotor::GearsetRatioPair drive_gearset = {
      okapi::AbstractMotor::gearset::blue, 7. / 3.};

  lcd_auton();
  printf("INITIALIZED\n");
}

void competition_initialize() {
  // lcd_auton();
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // lcd_auton();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

bool SKILLS = true; // true;
// void autonomous() {
//   auto_run();
//  if (SKILLS)
//{
//	intake_motors.move_voltage(9000);
//	arcade(0, -.15, 0);
//	pros::delay(750);

//	arcade(0, .15, 0);
//	pros::delay(600);
//	arcade(0, -.15, 0);
//	pros::delay(750);

//	arcade(0, .15, 0);
//	pros::delay(600);

//	arcade(0, -.15, 0);
//	pros::delay(750);

//	arcade(0, .15, 0);
//	pros::delay(600);
//	intake_motors.move_voltage(0);
//	arcade(0, 0, 0);
//}
// else
//{
//	arcade(0, -.15, 0);
//	pros::delay(750);
//	intake_motors.move_relative(90, 50);
//	arcade(0, .15, 0);
//	pros::delay(600);
//}
// pros::delay(600);
// arcade(0, 0, .1);
// while (imu.get_rotation() < 90)
//{
//	pros::delay(3);
//}
// arcade(0, 0, 0);
// flywheel_motor.move_voltage(FLYWHEEL_REV * 6 * 1200);
// flywheel_motor2.move_voltage(FLYWHEEL_REV * 6 * 1200);
// pros::delay(1700);
// indexer_task.notify();
// pros::delay(1000);
// indexer_task.notify();
// pros::delay(1700);
// flywheel_motor.move_voltage(FLYWHEEL_REV * 0 * 1200);
// flywheel_motor2.move_voltage(FLYWHEEL_REV * 0 * 1200);
//}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void drive_loop() {
  okapi::ControllerButton field_oriented(okapi::ControllerDigital::Y);
  bool currently_field_oriented = false;

  Joystick jStrafe(okapi::ControllerAnalog::leftX),
      jForward(okapi::ControllerAnalog::leftY),
      jYaw(okapi::ControllerAnalog::rightX);
  int count = 0;

  while (true) {
    if (field_oriented.changedToPressed()) currently_field_oriented ^= 1;

    double xSpeed = jStrafe.get(), forwardSpeed = jForward.get(),
           yaw = jYaw.get();

    // count++;
    // if (!(count % 30))
    //	printf("%f %f %f\n", xSpeed, forwardSpeed, yaw);

    // convert controller yaw using curve
    // increases precision on small values
    yaw = std::pow(yaw, 3); // cubic

    if (currently_field_oriented)
      Drivetrain.fieldorientedtoraw(xSpeed, forwardSpeed);

    Drivetrain.arcade(xSpeed, forwardSpeed, yaw);

    pros::delay(10);
  }
}
pros::Task drive_task(drive_loop);

IntakeButton intake_button(okapi::ControllerDigital::R1);
void intake_loop() {
  okapi::ControllerButton roller_up(okapi::ControllerDigital::L1);
  okapi::ControllerButton roller_down(okapi::ControllerDigital::L2);

  pros::Distance disk_distance_sensor(DISK_DISTANCE_SENSOR_PORT);
  PastNLessThanFilter disk_distance_filter(DISK_DISTANCE_SENSOR_THRESHOLD);

  while (true) {
    if (!intake_button.pressedWithinLast(5000_ms) &&
        (disk_distance_filter.get(disk_distance_sensor.get(),
                                  DISK_DISTANCE_SENSOR_LOOPS)))
      intake_button.off();

    int vel = intake_button.intakeDirection();
    Intake.loopController(roller_up.isPressed(), roller_down.isPressed(), vel);

    // DEBUGPRINT(30, "%d %d %d\n", vel, (((int)false) * 2), (((int)true) * 2));

    pros::delay(10);
  }
}
pros::Task intake_task(intake_loop);

inline bool is_expansion_buttons() {
  return pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                         pros::E_CONTROLLER_DIGITAL_R1) &&
         pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                         pros::E_CONTROLLER_DIGITAL_R2) &&
         pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                         pros::E_CONTROLLER_DIGITAL_L1) &&
         pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                         pros::E_CONTROLLER_DIGITAL_L2);
}
void expansion() {
  static pros::ADIDigitalOut expansion(EXPANSION_ADI_PORT);
  okapi::ControllerButton indexerbutton(okapi::ControllerDigital::R2);
  okapi::ControllerButton auto_roller_button(okapi::ControllerDigital::X);

  while (true) {
    if (is_expansion_buttons() || pros::Task::notify_take(1, 0)) {
      flywheel_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      flywheel_motor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      flywheel_task.notify_ext(0 + 1, pros::E_NOTIFY_ACTION_OWRITE, NULL);
      intake_button.off();
      Intake.moveVoltage(0); // set the intake to off
      Intake.lock();         // need to lock to prevent buttons working
      {
        pros::delay(700); // let the flywheel and intake stop

        expansion.set_value(true);

        pros::delay(3000); // wait 5 seconds to make sure it opens

        intake_button.off(); // force intake off, reset
      }
      Intake.unlock(); // let buttons work again
      flywheel_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      flywheel_motor2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 
    } else expansion.set_value(false);

    if (indexerbutton.changedToPressed()) indexer_task.notify();
    if (auto_roller_button.changedToReleased()) auto_roller_task.notify();

    pros::delay(10);
  }
}
pros::Task expansion_task(expansion);

// To set speed flywheel_task.notify_ext(<speed percentage (int)>  + 1,
// pros::E_NOTIFY_ACTION_OWRITE, NULL);
int flywheel_vel = 0; // read only TODO class
void flywheel_spinny() {
  const int MAX_VOLTAGE = 12000;
  const int wheel_speeds[] = {0, 60, 70};
  int vel = 0;

  okapi::ControllerButton upbutton(okapi::ControllerDigital::up);
  okapi::ControllerButton downbutton(okapi::ControllerDigital::down);
  while (true) { // take Task value
    if (upbutton.changedToPressed()) {
      for (int i = 0; i < LEN(wheel_speeds); i++) {
        printf("%d %d %d\n", i, vel, wheel_speeds[i]);
        if (vel <= wheel_speeds[i]) {
          vel = wheel_speeds[std::clamp(i + 1, 0, LEN(wheel_speeds) - 1)];
          break;
        }
      }
    } else if (downbutton.changedToPressed()) {
      for (int i = 0; i < LEN(wheel_speeds); i++) {
        printf("%d %d %d\n", i, vel, wheel_speeds[i]);
        if (vel <= wheel_speeds[i]) {
          vel = wheel_speeds[std::clamp(i - 1, 0, LEN(wheel_speeds) - 1)];
          break;
        }
      }
    } else if (int task_value = pros::Task::notify_take(1, 10)) {
      vel = task_value - 1;
    }
    // DEBUGPRINT(70, "%f\n", vel);
    flywheel_motor.move_voltage(FLYWHEEL_REV * vel / 100. * MAX_VOLTAGE);
    flywheel_motor2.move_voltage(FLYWHEEL_REV * vel / 100. * MAX_VOLTAGE);
    flywheel_vel = vel / 10;
    pros::delay(10);
  }
}
pros::Task flywheel_task(flywheel_spinny);

void opcontrol() {
  lcd_driver();
  auto_done();

  // while (true)
  //{
  //	pros::lcd::print(4, "%.2f %.2f", pros::c::adi_encoder_get(xenc) / 360.0,
  // pros::c::adi_encoder_get(yenc) / 360.0); 	pros::delay(20);
  // }
  okapi::Controller master(okapi::ControllerId::master);
  // okapi::Motor mymtrs[4] = {-11, -12, 19, 20};
  int count_loops = 0;

  double prevtheta = 0;
  double prevx = 0;
  double prevy = 0;
  int intake_vel = 0;
  okapi::AverageFilter<5> powfilter = okapi::AverageFilter<5>();
  okapi::AverageFilter<5> velfilter = okapi::AverageFilter<5>();

  bool fieldoriented = false;
  while (true) {
    count_loops++;

    //		if (!(count_loops % 100)) {
    //			printf("%f %f ; %f %f ; %f %f; %f %f \n",
    // topLeftMotor.getTargetVelocity(), topLeftMotor.getActualVelocity(),
    // topRightMotor.getTargetVelocity(), topRightMotor.getActualVelocity(),
    // bottomRightMotor.getTargetVelocity(),
    // bottomRightMotor.getActualVelocity(),
    // bottomLeftMotor.getTargetVelocity(),
    // bottomLeftMotor.getActualVelocity());
    //		}
    //
    // stuff

    static char a[50];
    sprintf(a, "pow: %.2f temp: %.2f", flywheel_motor.get_power(),
            flywheel_motor.get_temperature() * 1.);
    auto b = std::string(a);

    pros::delay(10);
    pros::lcd::set_text(0, b);

    sprintf(a, "pow: %.2f temp: %.2f", flywheel_motor2.get_power(),
            flywheel_motor2.get_temperature() * 1.);
    b = std::string(a);
    pros::lcd::set_text(1, b);

    sprintf(a, "pow: %.2f temp: %.2f", Intake.intake_motor.get_power(),
            Intake.intake_motor.get_temperature() * 1.);
    b = std::string(a);
    pros::lcd::set_text(2, b);

    sprintf(a, "pow: %.2f temp: %.2f", Intake.intake_motor2.get_power(),
            Intake.intake_motor2.get_temperature() * 1.);
    b = std::string(a);
    pros::lcd::set_text(3, b);

    sprintf(a, "vel: %.2f velset: %.2f",
            -flywheel_motor.get_actual_velocity() * 36., (float)flywheel_vel);
    b = std::string(a);
    pros::lcd::set_text(4, b);

    sprintf(a, "vel: %.2f velset: %.2f",
            -Intake.intake_motor.get_actual_velocity() * 6., (float)intake_vel);
    b = std::string(a);
    pros::lcd::set_text(5, b);

    sprintf(a, "%02dv%02d %02d%02d              ", flywheel_vel,
            std::lrint(-flywheel_motor.get_actual_velocity() * 36. / 100.),
            (int)Intake.intake_motor.get_temperature(),
            (int)Intake.intake_motor2.get_temperature());
    b = std::string(a);
    if (!(count_loops % 30)) {
      master.setText(2, 0, b);
      // printf("actual %f rpm; target %f rpm; %f motor%% %fdeg\n",
      // YAW_CONTROLLER.getProcessValue(), YAW_CONTROLLER.getTarget(),
      // YAW_CONTROLLER.getOutput() * 100, imu.get_rotation());
    }

    pros::delay(10);
  }
}
