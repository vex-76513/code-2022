#include "main.h"
#include "pal/auto.h"

extern pros::Task auto_roller_task;
extern pros::Task expansion_task;
extern pros::Task drive_task;
extern pros::Task intake_task;
extern pros::Task flywheel_task;
extern pros::Task indexer_task;

void spin_roller() {
  Drivetrain.arcade(0, -.1, 0);
  pros::delay(200);
  auto_roller_task.notify();
  pros::delay(1900);
  Drivetrain.arcade(0, 0, 0);
}

void turnToAngle(okapi::QAngle theta) {
  auto mypid = okapi::IterativeControllerFactory::posPID(.00005, 0.0002, 0, 0);
  while (!mypid.isSettled()) {
    mypid.setTarget(theta.convert(.01_deg));
    Drivetrain.arcade(
        0, 0,
        mypid.step(Drivetrain.imu.get_rotation() *
                   100)); // ik this doesn't do any of the loop around stuff but
                          // for now its as good as i can get sorry

    pros::delay(10);
  }
  Drivetrain.arcade(0, 0, 0);
}
void auto_skills_1(auto_color_t color, auto_pos_t pos) {
  printf("I am Auto Skills 1, color is %d, pos is %d\n", color, pos);
  // spin first roller
  spin_roller();

  // get to second roller
  Drivetrain.arcade(0, .1, 0);
  pros::delay(1950);
  Drivetrain.arcade(0, .0, 0);

  turnToAngle(90_deg + .5_deg);

  Intake.loopController(false, false, 1);
  Drivetrain.arcade(0, -.1, 0);
  pros::delay(1600);
  Drivetrain.arcade(0, .0, 0);
  Intake.loopController(false, false, 0);

  spin_roller();

  // turn
  flywheel_task.notify_ext(66 + 1, pros::E_NOTIFY_ACTION_OWRITE, NULL);
  turnToAngle(2.8_deg + .9_deg);
  //  shoot
  pros::delay(3500);
  indexer_task.notify();
  pros::delay(2000);
  indexer_task.notify();
  pros::delay(2000);
  indexer_task.notify();
  pros::delay(1000);

  // drive diagonally to middle
  Drivetrain.arcade(.22, .14, 0);
  pros::delay(2500);
  Drivetrain.arcade(0, 0, 0);
  // turn
  turnToAngle(45_deg);

  // shoot expansion
  expansion_task.notify_ext(1, pros::E_NOTIFY_ACTION_OWRITE, NULL);
  // drive backwards
  Drivetrain.arcade(0, -.05, 0);
  pros::delay(7.5e3);
  Drivetrain.arcade(0, .05, 0);
  pros::delay(1e3);
  Drivetrain.arcade(0, 0, 0);
  //turnToAngle(90_deg);
}
void auto_match_p1_1(auto_color_t color, auto_pos_t pos) {
  spin_roller();
  printf("DONE AUTO p1\n");
}

void auto_match_p2_1(auto_color_t color, auto_pos_t pos) {
  Drivetrain.arcade(.1, 0, 0);
  printf("AUTO p2 set\n");
  pros::delay(1900);
  Drivetrain.arcade(0, 0, 0);
  spin_roller();
  printf("DONE AUTO p2\n");
}

const auto_routine_t auto_list[] = {
    /* Robot skills options */
    {auto_skills_1, AUTO_POS_SKILLS, "Skills Center Goal"},
    /* Match autos */
    {auto_match_p1_1, AUTO_POS_1, "Match P1 Spin & Low"},
    {auto_match_p2_1, AUTO_POS_2, "P2 Move, Spin & Low"},

};

void auto_init() {
  drive_task.suspend();
  intake_task.suspend();
}
void auto_done() {

  drive_task.resume();
  intake_task.resume();
}
void autonomous() {
  auto_init();
  auto_run();
  printf("DONE AUTO\n");
  auto_done();
}
