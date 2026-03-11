#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/image/lv_image.h"

// ── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup intake;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// ── Pneumatics ───────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics will;
extern pros::adi::Pneumatics wing;
extern pros::adi::Pneumatics up;
extern pros::adi::Pneumatics down;

// ── Sensors ──────────────────────────────────────────────────────────────────
extern pros::Imu imu;

// ── LemLib objects ───────────────────────────────────────────────────────────
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;

// ── Controller ───────────────────────────────────────────────────────────────
extern pros::Controller controller;

// ── Display helpers ──────────────────────────────────────────────────────────
void blue_background();
void display_img_from_c_array();
void display_img_from_file(const void* src);

// ── Mechanism helpers ────────────────────────────────────────────────────────
void load();
void score();
void middle(int speed);
void outtake();

#define DIST_RIGHT_LATERAL_OFFSET  5.375
#define DIST_RIGHT_FB_OFFSET       4.0
#define DIST_LEFT_LATERAL_OFFSET  5.375
#define DIST_LEFT_FB_OFFSET        5.375
#define DIST_BACK_LATERAL_OFFSET  3.5
#define DIST_BACK_FB_OFFSET       3.5
#define FIELD_SIZE 144.0

enum class ResetWalls {
    LEFT,        // resets X using left wall
    RIGHT,       // resets X using right wall
    TOP,         // resets Y using top wall
    BOTTOM,      // resets Y using bottom wall
    LEFT_TOP,    // resets both X and Y (left + top walls)
    LEFT_BOTTOM,
    RIGHT_TOP,
    RIGHT_BOTTOM
};
extern pros::Distance dist_right;
extern pros::Distance dist_back;
extern pros::Distance dist_left;

void resetWithDistance(ResetWalls walls);