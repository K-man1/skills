#include "robot.h"

// ── Motors ───────────────────────────────────────────────────────────────────
pros::MotorGroup intake({14, -17});

pros::MotorGroup leftMotors({-13, 12, 11},  pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, -19, -18}, pros::MotorGearset::blue);

// ── Pneumatics ───────────────────────────────────────────────────────────────
pros::adi::Pneumatics will('D', false);
pros::adi::Pneumatics wing('B', false);
pros::adi::Pneumatics up('F',  false);
pros::adi::Pneumatics down('G', false);

// ── Sensors ──────────────────────────────────────────────────────────────────
pros::Imu      imu(10);

// ── Tracking wheels ──────────────────────────────────────────────────────────

// ── Drivetrain ───────────────────────────────────────────────────────────────
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    12,                          // track width (inches)
    lemlib::Omniwheel::NEW_325,  // wheel size
    450,                         // RPM
    8                            // horizontal drift
);

// ── PID controllers ──────────────────────────────────────────────────────────
lemlib::ControllerSettings lateral_controller(
    5.15,  // kP
    0,    // kI
    8.55,  // kD
    0,    // anti-windup
    0.5,    // small error range (in)
    100,    // small error timeout (ms)
    2,    // large error range (in)
    300,    // large error timeout (ms)
    15    // slew ← limits acceleration at the start
);      

lemlib::ControllerSettings angular_controller(
    1.795,  // kP
    0,     // kI
    11.65, // kD
    1.1,   // anti-windup
    1,     // small error range (deg)
    300,   // small error timeout (ms)
    3,     // large error range (deg)
    500,   // large error timeout (ms)
    0      // slew
);

// ── Odometry sensors ─────────────────────────────────────────────────────────
lemlib::OdomSensors sensors(
    nullptr,  // vertical wheel 1
    nullptr,    // vertical wheel 2
    nullptr,    // horizontal wheel 1
    nullptr,    // horizontal wheel 2
    &imu        // IMU
);

// ── Drive curves ─────────────────────────────────────────────────────────────
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// ── Chassis ──────────────────────────────────────────────────────────────────
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors,
    &throttleCurve,
    &steerCurve
);

// ── Controller ───────────────────────────────────────────────────────────────
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// ── Distance sensors ──────────────────────────────────────────────────────────
pros::Distance dist_right(8);
pros::Distance dist_back(2);
pros::Distance dist_left(3);