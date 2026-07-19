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
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

// =============================================================================
//  1. DEVICE CONFIGURATIONS & INITIALIZATIONS
// =============================================================================

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup leftMotors({-9, -8, -1}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({19, 11, 17}, pros::MotorGearset::green);

pros::Motor liftLeft(12);
pros::Motor motorLiftRight(-2); 
pros::Motor claw(-3);
pros::Motor arm(16);
pros::Imu imu(21);

pros::Rotation horizontalEnc(20);

// Tracking Sensors
pros::Rotation verticalEnc(-18);
lemlib::TrackingWheel vertical(&verticalEnc, 2.0, 0); // Calibrated rolling diameter: 2.0 inches

// Lift Sensor
pros::Rotation liftRot(-13);

// Drivetrain & Chassis Setup
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 14,
                              lemlib::Omniwheel::OLD_325, 333, 8);

// =============================================================================
//  PID CONSTANTS (Linear is locked to your perfect 18.0 setup!)
// =============================================================================
const double KP_VAL = 7.0;
const double KI_VAL = 0.0;
const double KD_VAL = 18.0; 

// Configure Linear Conditions
lemlib::ControllerSettings linearController(
    KP_VAL, KI_VAL, KD_VAL, 
    0,     // Anti-windup range
    0.1,   // Tightened exit window for millimeter accuracy
    100,   // Small error timeout
    1.0,   // Large error range 
    250,   // Large error timeout
    0      // Slew rate (0 to disable)
);

// Configure Angular Conditions (Starting baseline setup for tuning)
const double ANG_KP = 2.1;
const double ANG_KI = 0.0;
const double ANG_KD = 11.0;

lemlib::ControllerSettings angularController(
    ANG_KP, ANG_KI, ANG_KD, 
    3,    // Anti-windup range
    1,    // Small error range (degrees)
    100,  // Small error timeout
    3,    // Large error range
    500,  // Large error timeout
    0     // Slew rate
);

lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// =============================================================================
//  2. PARAMETRIC SD CARD PID LOGGER (Configured for Headings)
// =============================================================================

class PIDLogger {
private:
    std::string filename;
    bool is_initialized;
    uint32_t start_time;

    std::string formatFloat(double val) {
        std::ostringstream ss;
        ss << std::noshowpoint << val;
        return ss.str();
    }

public:
    PIDLogger() {
        is_initialized = false;
        start_time = 0;
    }

    void startNewRun(double kp, double ki, double kd) {
        // Generates file names like "ANG_2_0_10.csv" to keep them separated from linear logs
        filename = "/usd/ANG_" + formatFloat(kp) + "_" + formatFloat(ki) + "_" + formatFloat(kd) + ".csv";
        
        std::ofstream file(filename, std::ios::out); 
        if (file.is_open()) {
            file << "Time(ms),Target,ActualPosition,Error\n";
            file.close();
            is_initialized = true;
            start_time = pros::millis();
        } else {
            is_initialized = false;
            pros::lcd::print(6, "ERR: SD Card NOT Mounted!");
        }
    }

    void logPoint(double target, double actual) {
        if (!is_initialized) return;

        std::ofstream file(filename, std::ios::app); 
        if (file.is_open()) {
            uint32_t relative_time = pros::millis() - start_time;
            double error = target - actual;
            file << relative_time << "," << target << "," << actual << "," << error << "\n";
            file.close();
        }
    }
};

// Global Logger Setup
PIDLogger chassisLogger;
bool is_logging_active = false;
double logTargetValue = 90.0; // Heading target in degrees

// Dedicated thread to log telemetry asynchronously at ~33Hz (30ms) for high-density curves
void loggerTask() {
    while (true) {
        if (is_logging_active) {
            // Track theta (heading) instead of linear positions
            chassisLogger.logPoint(logTargetValue, chassis.getPose().theta);
        }
        pros::delay(30); 
    }
}

// =============================================================================
//  3. LIFT CONTROL & PID SETUP
// =============================================================================

lemlib::PID liftPID(1.8, 0, 0.08, 0, true); 
double liftKG = 6.0;      
double liftSlewRate = 8;  

double liftTarget = 0;    
double liftLastOutput = 0;
const double maxVoltage = 127;

bool liftPIDEnabled = false; 

const double LIFT_GEAR_RATIO = 0.8; 

double getLiftAngle() {
    double sensorDeg = liftRot.get_position() / 100.0; 
    double armDeg = sensorDeg / LIFT_GEAR_RATIO;         
    if (armDeg > 180.0) armDeg -= 360.0; 
    return armDeg;
}

void liftTask() {
    while (true) {
        if (liftPIDEnabled) {
            double currentAngle = getLiftAngle();

            double error = liftTarget - currentAngle;
            double pidOut = liftPID.update(error);

            double ffOut = liftKG * cos(currentAngle * M_PI / 180.0);

            double output = pidOut + ffOut;
            if (output > maxVoltage) output = maxVoltage;
            if (output < -maxVoltage) output = -maxVoltage;

            if (output > liftLastOutput + liftSlewRate) output = liftLastOutput + liftSlewRate;
            if (output < liftLastOutput - liftSlewRate) output = liftLastOutput - liftSlewRate;
            liftLastOutput = output;

            liftLeft.move(output);
            motorLiftRight.move(output);
        }
        pros::delay(10);
    }
}

// =============================================================================
//  4. INITIALIZE, DISABLED, & COMPETITION INITIALIZE
// =============================================================================

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    
    // Configure drivetrain motors to COAST
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);   
    motorLiftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    // Start background tasks
    pros::Task liftTaskHandle(liftTask);
    pros::Task loggingTaskHandle(loggerTask);
    
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.tare_position();

    // Prints formatted system data directly to the V5 Master Controller Screen
    pros::Task screenTask([&]() {
        while (true) {
            double x = chassis.getPose().x;
            double y = chassis.getPose().y;
            double theta = chassis.getPose().theta;
            double lift = getLiftAngle();

            controller.print(0, 0, "X:%.1f Y:%.1f       ", x, y);
            pros::delay(50); 

            controller.print(1, 0, "T:%.1f L:%.1f       ", theta, lift);
            pros::delay(50); 

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(150); 
        }
    });
}

void disabled() {}
void competition_initialize() {}

ASSET(example_txt);

// =============================================================================
//  5. AUTONOMOUS MODE (ANGULAR PID TUNING RUN)
// =============================================================================

void autonomous() {
    // 1. Reset physical coordinate origin and heading
    chassis.setPose(0, 0, 0); 

    // 2. Clear target file and configure logging.
    // Remember to update these three numbers to match whatever values you are testing in angularController!
    chassisLogger.startNewRun(ANG_KP, ANG_KI, ANG_KD);
    is_logging_active = true; 

    // 3. Turn to face exactly 90 degrees
    chassis.turnToHeading(90, 2000); 
    chassis.waitUntilDone();          

    // 4. Hold logging open for 500ms to capture high-density settled resting state
    pros::delay(500);
    is_logging_active = false;
}

// =============================================================================
//  6. OPCONTROL MODE (DRIVER CONTROL)
// =============================================================================

void opcontrol() {
    liftPIDEnabled = false; 
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.tare_position();

    const double SLOW_ZONE_ANGLE = 65.0;
    const double HOLD_ZONE_ANGLE = 85.0;
    const int32_t OP_MAX_SPEED = 9000;
    const int32_t OP_APPROACH_SPEED = 2500;
    const int32_t OP_GENTLE_HOLD = 1000;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        double current_pos = arm.get_position();
        chassis.arcade(leftY, rightY);
        pros::delay(10);

        // Manual Lift Control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftLeft.move_voltage(9000);
            motorLiftRight.move_voltage(9000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftLeft.move_voltage(-9000);
            motorLiftRight.move_voltage(-9000);
        } else {
            liftLeft.brake();
            motorLiftRight.brake();
        }

        // Claw Control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.move_voltage(12000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            claw.move_voltage(-12000);
        } else {
            claw.brake();
        }

        // Arm Control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (current_pos >= HOLD_ZONE_ANGLE) {
                arm.move_voltage(OP_GENTLE_HOLD);
            } else if (current_pos >= SLOW_ZONE_ANGLE) {
                arm.move_voltage(OP_APPROACH_SPEED);
            } else {
                arm.move_voltage(OP_MAX_SPEED);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (current_pos <= -HOLD_ZONE_ANGLE) {
                arm.move_voltage(-OP_GENTLE_HOLD);
            } else if (current_pos <= -SLOW_ZONE_ANGLE) {
                arm.move_voltage(-OP_APPROACH_SPEED);
            } else {
                arm.move_voltage(-OP_MAX_SPEED);
            }
        } else {
            if (current_pos >= HOLD_ZONE_ANGLE) {
                arm.move_voltage(OP_GENTLE_HOLD);
            } else if (current_pos <= -HOLD_ZONE_ANGLE) {
                arm.move_voltage(-OP_GENTLE_HOLD);
            } else {
                arm.move_voltage(0);
            }
        }
    }
}