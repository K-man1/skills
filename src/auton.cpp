#include "robot.h"

// ── Distance resets ──────────────────────────────────────────────────────────────
    void resetWithDistance(ResetWalls walls) {
        double heading = chassis.getPose().theta;

        // Determine which sensor faces which wall based on current heading
        // Normalize heading to 0-360
        double h = fmod(heading, 360.0);
        if (h < 0) h += 360.0;

        // Each sensor's effective facing = its physical offset + robot heading
        // At 0°:   left→left wall, back→bottom wall
        // At 90°:  left→top wall,  back→left wall
        // At 180°: left→right wall, back→top wall
        // At 270°: left→bottom wall, back→right wall

        auto getXSensor = [&]() -> double {
            if (h >= 315 || h < 45)   return dist_left.get() / 25.4;  // facing up, left→left wall
            if (h >= 45  && h < 135)  return dist_back.get() / 25.4;  // facing right, back→left wall
            if (h >= 135 && h < 225)  return dist_right.get() / 25.4; // facing down, right→left wall
            return dist_left.get() / 25.4;                             // facing left
        };

        auto getYSensor = [&]() -> double {
            if (h >= 315 || h < 45)   return dist_back.get() / 25.4;  // facing up, back→bottom wall
            if (h >= 45  && h < 135)  return dist_left.get() / 25.4;  // facing right, left→top wall
            if (h >= 135 && h < 225)  return dist_back.get() / 25.4;  // facing down, back→top wall
            return dist_left.get() / 25.4;                             // facing left
        };

        if (walls == ResetWalls::LEFT || walls == ResetWalls::LEFT_TOP || walls == ResetWalls::LEFT_BOTTOM) {
            double newX = -(FIELD_SIZE / 2.0) + getXSensor() + DIST_LEFT_LATERAL_OFFSET;
            chassis.setPose(newX, chassis.getPose().y, heading);
        }

        if (walls == ResetWalls::RIGHT || walls == ResetWalls::RIGHT_TOP || walls == ResetWalls::RIGHT_BOTTOM) {
            double newX = (FIELD_SIZE / 2.0) - getXSensor() - DIST_RIGHT_LATERAL_OFFSET;
            chassis.setPose(newX, chassis.getPose().y, heading);
        }

        if (walls == ResetWalls::TOP || walls == ResetWalls::LEFT_TOP || walls == ResetWalls::RIGHT_TOP) {
            double newY = (FIELD_SIZE / 2.0) - getYSensor() - DIST_LEFT_LATERAL_OFFSET;
            chassis.setPose(chassis.getPose().x, newY, heading);
        }

        if (walls == ResetWalls::BOTTOM || walls == ResetWalls::LEFT_BOTTOM || walls == ResetWalls::RIGHT_BOTTOM) {
            double newY = -(FIELD_SIZE / 2.0) + getYSensor() + DIST_BACK_FB_OFFSET;
            chassis.setPose(chassis.getPose().x, newY, heading);
        }
    }
// ── Mechanism helpers ─────────────────────────────────────────────────────────
void load() {
    intake.move(127);
    up.set_value(false);
    down.set_value(true);
}

void score() {
    up.set_value(true);
    down.set_value(true);
    intake.move(-127);
    pros::delay(250);
    intake.move(127);
}

void middle(int speed) {
    intake.move(speed);
    up.set_value(false);
    down.set_value(false);
}

void outtake() {
    intake.move(-100);
}

// ── Autonomous ────────────────────────────────────────────────────────────────
void autonomous() {
    //set start position
    chassis.setPose(-0, 0, 90);
    resetWithDistance(ResetWalls::LEFT_TOP);
    load();
    pros::lcd::print(3, "left: %f", dist_left.get() / 25.4);
    pros::lcd::print(4, "back: %f", dist_back.get() / 25.4);
    //Intake 4 blocks 
    chassis.moveToPose(-23, 27, 315, 2000, {.maxSpeed = 75});
    //go into middle goal
    chassis.turnToHeading(315,1000);
    intake.move(0);
    chassis.moveToPose(-9,9,315,3000,{.forwards=false});
    pros::delay(2000);
    middle(50);
    pros::delay(3000);

    //go to loader & goal axis
    chassis.moveToPoint(-46, 48.5, 3000);
    //turn to align to loader
    chassis.turnToHeading(-90, 3000);
    //GO INTO LOADER
    will.extend();
    chassis.moveToPoint(-65, 48.5, 3000, {.maxSpeed=75});
    load();
    pros::delay(2000);
    

    //back away from loader
    chassis.moveToPoint(-46, 47, 3000, {.forwards=false});
    intake.move(0);
    will.retract();
    //go into alley
    chassis.turnToHeading(235, 3000);
    chassis.moveToPoint(-40, 62, 3000, {.forwards=false});
    //go through alley
    chassis.turnToHeading(-90,3000);
    chassis.moveToPoint(30, chassis.getPose().y, 5000, {.forwards=false, .maxSpeed=80});
    //align to long goal
    chassis.moveToPoint(47, 46, 3000, {.forwards=false});
    chassis.turnToHeading(90, 3000);
    //go into long goal
    chassis.moveToPoint(23, 46, 3000, {.forwards=false});
    //score in long goal
    score();
    pros::delay(2000);

    //go into loader
    chassis.moveToPose(69, 46, 90, 3000, {.maxSpeed=75});
    will.toggle();
    load();
    pros::delay(2000);

    chassis.moveToPoint(23, 46, 3000, {.forwards=false});
    will.retract();
    score();
    pros::delay(2000);

    //algin with parking barrier
    chassis.moveToPose(60, 10, 180, 3000);

    //park
    load();
    chassis.moveToPoint(60, -55, 3000);
    chassis.turnToHeading(-90, 3000);
    chassis.moveToPoint(chassis.getPose().x+10, chassis.getPose().y, 3000, {.forwards=false});
    resetWithDistance(ResetWalls::RIGHT_BOTTOM);
    intake.move(0);
}
