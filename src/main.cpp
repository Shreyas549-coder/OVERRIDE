#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-9, -8, -1},
                            pros::MotorGearset::green);
pros::MotorGroup rightMotors({19, 11, 17}, pros::MotorGearset::green);

pros::Motor liftLeft(-2);
pros::Motor liftRight(12);
pros::Motor claw(-3);
pros::Motor arm(16);
pros::Imu imu(21);

pros::Rotation horizontalEnc(20);
pros::Rotation verticalEnc(-18);

// single lift rotation sensor (DR4B sides are mechanically linked, so one sensor tracks both)
pros::Rotation liftRot(13);

lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 14,
                              lemlib::Omniwheel::OLD_325, 333, 8);

lemlib::ControllerSettings linearController(7, 0, 18, 3, 1, 100, 3, 500, 10);
lemlib::ControllerSettings angularController(2.1, 0, 11, 3, 1, 100, 3, 500, 0);

lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// ===== LIFT PID SETUP =====
// TUNE THESE VALUES:
lemlib::PID liftPID(1.375, 0., 0.1, 0, true); // kP, kI, kD  <-- TUNE kP and kD here
double liftKG = 6.0;      // gravity feedforward            <-- TUNE THIS
double liftSlewRate = 8;  // smoothness (lower = smoother)  <-- TUNE THIS (last)

double liftTarget = 0;    // current target angle, degrees
double liftLastOutput = 0;
const double maxVoltage = 127;

bool liftPIDEnabled = false; // false = manual driver control, true = PID active (autonomous)

// Ratio between the rotation sensor and the actual lift arm.
// Example: if the sensor spins 2 full rotations for every 1 rotation of the arm, this is 2.0
// If the sensor spins slower than the arm (e.g. sensor rotates 0.5x for every 1x arm), this is 0.5
const double LIFT_GEAR_RATIO = 0.71428571428; // <-- SET THIS to your actual ratio

double getLiftAngle() {
    double sensorDeg = liftRot.get_position() / 100.0; // raw sensor reading, 0-360
    if (sensorDeg > 180.0) sensorDeg -= 360.0; // unwrap at the RAW sensor level first
    double armDeg = sensorDeg / LIFT_GEAR_RATIO; // THEN scale to true arm angle
    return armDeg;
}

void setLiftTarget(double angleDeg) {
    liftTarget = angleDeg;
}

// ===== CALIBRATION MODEL =====
// This quartic was fit from real test data: for a given COMMANDED target,
// it predicts the ACTUAL angle the lift settles at (due to PID steady-state error).
// actualAngle = f(commandedAngle)
double liftCalibrationModel(double commandedDeg) {
    double x = commandedDeg;
    return (-1.602e-7) * pow(x, 4)
         + (9.12058e-5) * pow(x, 3)
         - (1.72986e-2) * pow(x, 2)
         + 2.1156 * x
         - 20.78015;
}

// Inverts the calibration model via binary search: given the REAL angle you want,
// finds the commanded target that actually produces it.
double solveForCommandedTarget(double desiredActualAngle) {
    double lo = 0.0;
    double hi = 244.37; // measured max height in angle

    for (int i = 0; i < 50; i++) {
        double mid = (lo + hi) / 2.0;
        double predictedActual = liftCalibrationModel(mid);
        if (predictedActual < desiredActualAngle) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return (lo + hi) / 2.0;
}

// Use this instead of setLiftTarget() when you want the REAL angle to be accurate,
// e.g. setLiftTargetCalibrated(90) internally commands ~96.02 to actually reach 90.
void setLiftTargetCalibrated(double desiredActualAngle) {
    double correctedCommand = solveForCommandedTarget(desiredActualAngle);
    setLiftTarget(correctedCommand);
}

void waitUntilLiftAt(double targetAngle, double tolerance = 3.0) {
    while (std::fabs(getLiftAngle() - targetAngle) > tolerance) {
        pros::delay(10);
    }
}

// call this right before enabling PID so it doesn't slam to an old target
void enableLiftPID() {
    liftTarget = getLiftAngle();
    liftLastOutput = 0;
    liftPIDEnabled = true;
}

void liftTask() {
    while (true) {
        if (liftPIDEnabled) {
            double currentAngle = getLiftAngle();

            double error = liftTarget - currentAngle;
            double pidOut = liftPID.update(error);

            // 0deg = horizontal/stowed = max gravity torque
            double ffOut = liftKG * cos(currentAngle * M_PI / 180.0);

            double output = pidOut + ffOut;
            if (output > maxVoltage) output = maxVoltage;
            if (output < -maxVoltage) output = -maxVoltage;

            if (output > liftLastOutput + liftSlewRate) output = liftLastOutput + liftSlewRate;
            if (output < liftLastOutput - liftSlewRate) output = liftLastOutput - liftSlewRate;
            liftLastOutput = output;

            liftLeft.move(output);
            liftRight.move(output);
        }
        pros::delay(10);
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);   // keep HOLD — fine alongside kG
    liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftRot.reset_position(); 
    pros::Task liftTaskHandle(liftTask);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.tare_position();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Lift: %f", getLiftAngle());
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

ASSET(example_txt);

void lift_UP(float time) {
    liftLeft.move_voltage(8000);
    liftRight.move_voltage(8000);
    pros::delay(time);
    liftRight.brake();
    liftLeft.brake();
}

void lift_DOWN(float time) {
    liftLeft.move_voltage(-3000);
    liftRight.move_voltage(-3000);
    pros::delay(time);
    liftRight.brake();
    liftLeft.brake();
}

const double SLOW_ZONE_DELTA = 20.0;
const double HOLD_ZONE_DELTA = 5.0;
const int32_t MAX_SPEED = 9000;
const int32_t APPROACH_SPEED = 2500;
const int32_t GENTLE_HOLD = 3000;

void moveArmAuton(double targetAngle) {
    while (true) {
        double current_pos = arm.get_position();
        double error = targetAngle - current_pos;

        if (targetAngle > 0) {
            if (current_pos >= (targetAngle - HOLD_ZONE_DELTA)) {
                arm.move_voltage(GENTLE_HOLD);
                break;
            } else if (current_pos >= (targetAngle - SLOW_ZONE_DELTA)) {
                arm.move_voltage(APPROACH_SPEED);
            } else {
                arm.move_voltage(MAX_SPEED);
            }
        } else if (targetAngle < 0) {
            if (current_pos <= (targetAngle + HOLD_ZONE_DELTA)) {
                arm.move_voltage(-GENTLE_HOLD);
                break;
            } else if (current_pos <= (targetAngle + SLOW_ZONE_DELTA)) {
                arm.move_voltage(-APPROACH_SPEED);
            } else {
                arm.move_voltage(-MAX_SPEED);
            }
        } else {
            if (std::abs(error) < 3.0) {
                arm.move_voltage(0);
                break;
            }
            int32_t return_speed = (error > 0) ? APPROACH_SPEED : -APPROACH_SPEED;
            arm.move_voltage(return_speed);
        }
        pros::delay(20);
    }
}

void autonomous() {
    //244.37 is max height in angleacc
    // ===== TUNING TEST BLOCK — delete once kP/kD/slew are dialed in =====
    /**/
    
    enableLiftPID();
    /*
    setLiftTargetCalibrated(122); // change this to whatever REAL angle you're testing
    while (true) { pros::delay(20); }
    */
    // ===== END TEST BLOCK =====

    chassis.setPose(0,0,0);
    claw.move_voltage(12000);
    //setLiftTargetCalibrated(80);
    //claw.move_voltage(12000);
    //moveArmAuton(-90);

    // changing roller
    chassis.moveToPoint(0, 3, 500, {.minSpeed=33});
    chassis.moveToPoint(0, 0, 500, {.forwards=false, .minSpeed=33});
    pros::delay(500);
    chassis.moveToPoint(0, 3, 500, {.minSpeed=33});
    chassis.moveToPoint(0, 0, 500, {.forwards=false, .minSpeed=33});
    pros::delay(500);

    //score preload
    moveArmAuton(-90);
    setLiftTargetCalibrated(100);
    chassis.moveToPose(-24, 13.5, 90, 3000, {.lead=0.7 ,.maxSpeed=80, .minSpeed=20});
    chassis.waitUntilDone();
    setLiftTargetCalibrated(0);
    pros::delay(500);
    claw.move_voltage(-12000);


/*
    // first pin
    chassis.moveToPose(23, 39, 42, 5000, {.forwards=false, .maxSpeed=80}); 
    chassis.turnToHeading(90, 500);
    chassis.moveToPose(36, 40, -90, 5000, {.maxSpeed=80});

    // second toggle 
    chassis.moveToPose(65.7, 64.5, 90, 5000, {.forwards=false, .lead=0.8, .maxSpeed=90});

    //second pin
    chassis.moveToPoint(62, 36.5, 15000, { .maxSpeed=80});
    chassis.moveToPose(62, 36.5, 0, 15000, {.forwards=false, .lead=0.2, .maxSpeed=80});
    chassis.moveToPose(45, 8, 45, 15000, {.forwards=false, .lead=0.8, .maxSpeed=80});
    chassis.turnToHeading(0, 5000, {.maxSpeed=80});
    chassis.moveToPose(47.5, 22.8, 0, 15000, {.maxSpeed=80});

    //loader pin #3
    chassis.moveToPose(58.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    chassis.moveToPose(47.5, 22.8, 0, 15000, {.maxSpeed=80});

    //loader pin #4
    chassis.moveToPose(58.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    chassis.moveToPose(47.5, 22.8, 0, 15000, {.maxSpeed=80});

    //loader pin #5
    chassis.moveToPose(58.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    chassis.moveToPose(36, 12.1, -90, 15000, {.maxSpeed=80});

    //loader pin #6
    chassis.moveToPose(56.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    chassis.moveToPose(36, 9.1, -90, 15000, {.maxSpeed=80});

    //loader pin #7
    chassis.moveToPose(54.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    chassis.moveToPose(36, 6.1, -90, 15000, {.maxSpeed=80});

    //loader pin #8
    chassis.moveToPose(52.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    chassis.moveToPose(36, 3.1, -90, 15000, {.maxSpeed=80});
    
    //loader pin #9
    chassis.moveToPose(50.5, -2.65, 0, 15000, {.forwards=false, .maxSpeed=80});
    //chassis.moveToPose(165.44, -4.533, -45, 3000);
    */
    

    /*
    chassis.setPose(0,0,0);
    moveArmAuton(-90);
    lift_DOWN(1);
    pros::delay(1500);
    claw.move_voltage(12000);
    chassis.moveToPose(0.07, 8.36, 1, 5000, {.maxSpeed=15});
    chassis.waitUntilDone();
    pros::delay(500);
    lift_UP(500);
    pros::delay(500);
    chassis.turnToHeading(180, 5000, {.maxSpeed=60});
    pros::delay(500);
    moveArmAuton(90);
    pros::delay(500);
    chassis.moveToPoint(-1.5, 32, 7000, {.forwards = false, .maxSpeed=20});
    chassis.waitUntilDone();
    pros::delay(500);
    lift_DOWN(750);
    pros::delay(1000);
    claw.move_voltage(-12000);
    pros::delay(1000);
    chassis.moveToPoint(0, 20.55, 5000, {.maxSpeed=30});
    */
}

void opcontrol() {
    liftPIDEnabled = false;
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.tare_position();

    const double SLOW_ZONE_ANGLE = 65.0;
    const double HOLD_ZONE_ANGLE = 85.0;
    const int32_t MAX_SPEED = 9000;
    const int32_t APPROACH_SPEED = 2500;
    const int32_t GENTLE_HOLD = 3000;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        double current_pos = arm.get_position();
        chassis.arcade(leftY, rightY);
        pros::delay(10);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftLeft.move_voltage(9000);
            liftRight.move_voltage(9000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftLeft.move_voltage(-9000);
            liftRight.move_voltage(-9000);
        } else {
            liftLeft.brake();
            liftRight.brake();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.move_voltage(12000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            claw.move_voltage(-12000);
        } else {
            claw.brake();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (current_pos >= HOLD_ZONE_ANGLE) {
                arm.move_voltage(GENTLE_HOLD);
            } else if (current_pos >= SLOW_ZONE_ANGLE) {
                arm.move_voltage(APPROACH_SPEED);
            } else {
                arm.move_voltage(MAX_SPEED);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (current_pos <= -HOLD_ZONE_ANGLE) {
                arm.move_voltage(-GENTLE_HOLD);
            } else if (current_pos <= -SLOW_ZONE_ANGLE) {
                arm.move_voltage(-APPROACH_SPEED);
            } else {
                arm.move_voltage(-MAX_SPEED);
            }
        } else {
            if (current_pos >= HOLD_ZONE_ANGLE) {
                arm.move_voltage(GENTLE_HOLD);
            } else if (current_pos <= -HOLD_ZONE_ANGLE) {
                arm.move_voltage(-GENTLE_HOLD);
            } else {
                arm.move_voltage(0);
            }
        }
    }
}
