#include "robot.h"

void opcontrol() {

    while (true) {
        // ── Drive ──────────────────────────────────────────────────────────
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // ── Intake / mechanism controls ────────────────────────────────────
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Load mode
            intake.move(127);
            up.set_value(false);
            down.set_value(true);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            // Quick reverse pulse, then score
            intake.move(-127);
            pros::delay(200);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // Score mode
            intake.move(127);
            up.set_value(true);
            down.set_value(true);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127);
            pros::delay(50);
        }

        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            middle(75);
        }

        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            middle(30);
        }

        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // Outtake
            intake.move(-100);
        }
        else {
            // Default / idle
            up.set_value(false);
            down.set_value(true);
            intake.move(0);
        }

        // ── Pneumatic toggles ──────────────────────────────────────────────
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            will.toggle();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            wing.toggle();
        }
        pros::delay(25);
    }
}