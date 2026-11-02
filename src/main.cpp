#include "main.h"
#include "fmt/core.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"
#include <cmath>
#include <cstdint>


// =====================================================================
// LIFT TUNING MODE
//   0 = normal driver control
//   1 = gravity measurement, fills in kGHold[]        (do this first)
//   2 = PID step test, jump between preset angles      (do this second)
// Set back to 0 when you are done.
// =====================================================================
#define LIFT_TUNE_MODE 0


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// motor groups
pros::MotorGroup leftMotors({-9, -8, -7}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({19, 11, 17}, pros::MotorGearset::green);


pros::Motor liftLeft(-2);
pros::Motor liftRight(12);
pros::Motor claw(3);
pros::Motor arm(16);
pros::Imu imu(21);


pros::Rotation verticalEnc(-18);


// single lift rotation sensor (DR4B sides are mechanically linked, so one sensor tracks both)
pros::Rotation liftRot(13);


lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);


lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 14,
                             lemlib::Omniwheel::OLD_325, 333, 8);


lemlib::ControllerSettings linearController(7, 0, 18, 3, 1, 100, 3, 500, 10);
lemlib::ControllerSettings angularController(1, 0, 0.05, 3, 1, 100, 3, 500, 0);


lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);


lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);


lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors,
                       &throttleCurve, &steerCurve);


// =====================================================================
// LIFT
// =====================================================================


// Hand rolled instead of lemlib::PID for one reason: the derivative is taken off
// the MEASUREMENT, not off the error. lemlib::PID::reset() zeroes prevError, so
// the first loop after every setLiftTarget() sees a derivative of (error - 0)
// and fires a kD * error spike the instant you press a button, before the lift
// has moved at all. On a 180 degree jump with kD at 0.1 that is an 18 unit kick.
// Off the measurement there is no spike, because the D term only ever reacts to
// the lift physically moving. You would have hit this at step 3 below.
//
// Units are unchanged: called once per 10 ms loop, integral accumulates per loop
// and derivative is the per loop change, so the procedure still reads the same:
//   1. kGHold[] measured, kP 0, kD 0, kI 0   -> lift should roughly hold anywhere
//   2. raise kP until it oscillates, then cut 40 percent
//   3. add kD, start at kP/10, raise until the overshoot is gone
//   4. add kI only if it still stops short, start at 0.0005
struct LiftPID {
   double kP, kI, kD;
   double windupRange;   // only integrate once the error is inside this
   double iLimit;        // hard cap on how much the I term may contribute

   double integral = 0;
   double lastAngle = 0;
   bool primed = false;

   LiftPID(double p, double i, double d, double windup, double iCap)
       : kP(p), kI(i), kD(d), windupRange(windup), iLimit(iCap) {}

   void reset(double angle) {
       integral = 0;
       lastAngle = angle;
       primed = true;
   }

   void clearIntegral() { integral = 0; }

   double update(double target, double angle) {
       if (!primed) reset(angle);

       double error = target - angle;

       if (std::fabs(error) < windupRange) integral += error;
       else integral = 0;

       if (kI > 0) {
           double cap = iLimit / kI;
           if (integral >  cap) integral =  cap;
           if (integral < -cap) integral = -cap;
       }

       double velocity = angle - lastAngle;   // lift degrees per 10 ms loop
       lastAngle = angle;

       return kP * error + kI * integral - kD * velocity;
   }
};

// kP, kI, kD, windupRange, integral cap.
// Starting gains for the on-robot test; adjust from measured response.
// kD acts on degrees moved per loop, so keep the 10 ms loop while tuning.
LiftPID liftPID(1.6, 0.0, 0.16, 10, 20);


// How much of the downward command to actually use. Descending is gravity
// assisted, so the full negative command is never needed and only makes the
// arrival harder. 1.0 is the old behaviour, 0.7 is roughly 70 percent.
//
// Worth knowing what this can and cannot do: it caps how hard the motors pull
// the lift down, but it cannot slow a lift that is already falling under its own
// weight. Braking a fall needs a POSITIVE command, which is kD's job, not this.
const double LIFT_DOWN_SCALE = 0.7;

// Autonomous descent protection. Gravity can keep accelerating the lift after
// a lower target creates a large negative PID command.
const double LIFT_AUTON_DOWN_MAX = 85.0;
const double LIFT_AUTON_DOWN_SLEW = 35.0;
const double LIFT_AUTON_BOTTOM_HOLD = 20.0; // move() units, only at a 0-degree target


double liftSlewRate = 60;   // output change allowed per 10 ms loop, tune this LAST
double liftTarget = 0;     // current target, in lift degrees
double liftLastOutput = 0;
const double maxVoltage = 127;


bool liftPIDEnabled = false;   // false = manual driver control, true = PID active
bool liftAutonomousMode = false;


// Tuning scratch value. Global so the screen task can print it, because only
// ONE task in this program is allowed to touch the brain screen.
double liftTestHold = 0;


// How far initialize() has got. Shown on brain screen line 7 so a startup that
// hangs is visible, instead of looking like the whole program is dead.
const char* initStage = "boot";


// Telemetry, written by liftTask and read by the screen task. PID and FF are
// kept apart on purpose: when the lift sits at the wrong height, seeing which
// of the two is winning tells you immediately whether the table is wrong or
// the gains are.
double liftAngleNow = 0;    // last trusted angle, in lift degrees
double liftPIDNow = 0;      // PID part of the last output
double liftFFNow = 0;       // feedforward part of the last output
bool   liftSensorOK = true;


// Travel limits, in lift degrees. Measured max height was 244.37, keep margin.
const double LIFT_MIN = 0.0;
const double LIFT_MAX = 220.0;


// Ratio between the rotation sensor and the lift arm.
const double LIFT_GEAR_RATIO = 0.71428571428;


// ---------------------------------------------------------------------
// GRAVITY FEEDFORWARD TABLE
//
// Run with LIFT_TUNE_MODE 1. At each angle, let go of the jog buttons and tap
// X or Y until the lift sits perfectly still, then write that hold number here.
// These are motor commands in the -127 to 127 range, same units as move().
//
// This replaces the old liftKG * cos(angle). On this lift gravity is hardest
// near the TOP, and cos() peaks at the BOTTOM and goes negative past 90, so the
// old feedforward was pushing down exactly where the lift needed the most help.
// ---------------------------------------------------------------------
// DO NOT GUESS THESE. {0, 20, 60, 110, 150, 170} is what made the step test
// send the lift to the ceiling on every button. Two problems with it:
//   1. 150 and 170 are outside the -127..127 range move() accepts, so near the
//      top the output pinned at 127 and the PID had no authority at all.
//   2. It rose 1.25 command units per degree between 114 and 154 while kP was
//      0.5. You need kP > d(feedforward)/d(angle) or there is no angle where
//      PID + FF balances, and it runs away upward with no equilibrium. At
//      target 40 sitting at 220 it was still commanding about +73, which is
//      why Y never brought it back down.
// Measured in MODE 1. These are the angles the lift was actually parked at, not
// round numbers, because interpolating between real points beats rounding them.
// Angles must stay strictly increasing.
//
// The shape is what you want to see: nearly flat through the middle, then a
// step up near the top as the arm reaches out. Worst slope is 0.09 against a
// kP of 0.5, comfortably stable.
const int kGN = 7;
const double kGAngle[kGN] = {0, 49, 80, 109, 125, 177, 220};
double       kGHold[kGN]  = {4,  5,  6,   6,   7,   8,  12};

// Holding a DR4B still should never need half power. Hard cap so a bad table
// cannot saturate the output and lock the PID out.
const double LIFT_FF_MAX = 70;


double liftFF(double angle) {
   double ff = kGHold[kGN - 1];
   if (angle <= kGAngle[0]) {
       ff = kGHold[0];
   } else if (angle < kGAngle[kGN - 1]) {
       for (int i = 0; i < kGN - 1; i++) {
           if (angle <= kGAngle[i + 1]) {
               double t = (angle - kGAngle[i]) / (kGAngle[i + 1] - kGAngle[i]);
               ff = kGHold[i] + t * (kGHold[i + 1] - kGHold[i]);
               break;
           }
       }
   }
   if (ff >  LIFT_FF_MAX) ff =  LIFT_FF_MAX;
   if (ff < -LIFT_FF_MAX) ff = -LIFT_FF_MAX;
   return ff;
}


// Worst upward slope in the feedforward table, in command units per lift degree.
// This has to stay below kP or there is no angle where PID + FF balances and the
// lift runs away. Shown on brain screen line 7 next to kP.
double liftMaxFFSlope() {
   double worst = 0;
   for (int i = 0; i < kGN - 1; i++) {
       double s = (kGHold[i + 1] - kGHold[i]) / (kGAngle[i + 1] - kGAngle[i]);
       if (s > worst) worst = s;
   }
   return worst;
}


// ---------------------------------------------------------------------
// Reading the lift angle.
//
// The old code unwrapped at raw 180, which is 252 lift degrees after the gear
// ratio. Max travel is 244.37, so there were only 8 degrees before the reading
// flipped negative and the PID slammed. Unwrapping at 300 instead means the
// only thing that wraps is dropping below zero, which is what you want.
// ---------------------------------------------------------------------
// A wrong port or a loose cable makes get_position() return PROS_ERR, which is
// INT32_MAX. Divided down that is a 30 million degree reading, the error goes
// hugely negative, and the lift slams into the bottom stop and sits there. That
// looks exactly like a tuning problem and is not one, so it gets caught here
// and reported on brain screen line 6 instead.
double getLiftAngle() {
   std::int32_t raw = liftRot.get_position();

   if (raw == PROS_ERR || raw > 100000 || raw < -100000) {
       liftSensorOK = false;
       return liftAngleNow;      // freeze on the last good reading, do not slam
   }
   liftSensorOK = true;

   double sensorDeg = raw / 100.0;
   if (sensorDeg > 300.0) sensorDeg -= 360.0;
   liftAngleNow = sensorDeg / LIFT_GEAR_RATIO;
   return liftAngleNow;
}


void setLiftTarget(double angleDeg) {
   if (angleDeg < LIFT_MIN) angleDeg = LIFT_MIN;
   if (angleDeg > LIFT_MAX) angleDeg = LIFT_MAX;
   // Clear the integral only. lastAngle has to survive, or the derivative reads
   // a huge fake velocity on the next loop.
   if (std::fabs(angleDeg - liftTarget) > 0.5) liftPID.clearIntegral();
   liftTarget = angleDeg;
}


// call this right before enabling PID so it does not slam to an old target
void enableLiftPID() {
   double angle = getLiftAngle();
   liftTarget = angle;
   liftPID.reset(angle);
   liftLastOutput = liftFF(angle);   // seed the slew so it does not sag
   liftPIDEnabled = true;
}


void liftTask() {
   while (true) {
       if (liftPIDEnabled) {
           double currentAngle = getLiftAngle();

           if (!liftSensorOK) {
               // No trustworthy position. Hold with the last feedforward rather
               // than running the loop on a garbage error.
               liftLeft.move(liftFFNow);
               liftRight.move(liftFFNow);
               pros::delay(10);
               continue;
           }

           const double previousAngle = liftPID.lastAngle;
           liftPIDNow = liftPID.update(liftTarget, currentAngle);
           liftFFNow = liftFF(currentAngle);

           double output = liftPIDNow + liftFFNow;

           // Soft travel limits. Past the top the loop may hold but never climb,
           // so a bad feedforward table cannot walk the lift into the stop.
           if (currentAngle >= LIFT_MAX && output > liftFFNow) output = liftFFNow;
           if (currentAngle <= LIFT_MIN && output < 0) output = 0;

           // Coming down, gravity is already doing most of the work, so a full
           // negative command just drops the lift onto its own stop. Scale the
           // downward half only. The upward half and the feedforward that holds
           // it in place are untouched.
           if (output < 0) output *= LIFT_DOWN_SCALE;

           if (liftAutonomousMode && output < 0) {
               // Cap the downward command in autonomous. Positive output is
               // left available so the D term can brake a falling lift.
               if (output < -LIFT_AUTON_DOWN_MAX) output = -LIFT_AUTON_DOWN_MAX;

               // Ramp into a stronger downward command gradually.
               if (output < liftLastOutput - LIFT_AUTON_DOWN_SLEW) {
                   output = liftLastOutput - LIFT_AUTON_DOWN_SLEW;
               }
           }

           // Once the lift has settled near zero, press gently against the
           // rubber bands. Never apply this while descending quickly or while
           // autonomous is targeting a raised position.
           if (liftAutonomousMode && liftTarget <= LIFT_MIN &&
               currentAngle <= 5.0 && currentAngle - previousAngle >= -0.5 &&
               output > -LIFT_AUTON_BOTTOM_HOLD) {
               output = -LIFT_AUTON_BOTTOM_HOLD;
           }

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


// =====================================================================


void initialize() {
   // ORDER MATTERS HERE, and the old order is why the controller text stopped.
   //
   // chassis.calibrate() blocks while the IMU on port 21 settles. If that sensor
   // is unplugged or unhappy, LemLib retries and initialize() can sit there for
   // many seconds, or never return at all. opcontrol() does not start until
   // initialize() returns, so a stuck calibrate looks exactly like "my buttons
   // and my controller screen do nothing" even though the code is fine.
   //
   // The screen now comes up FIRST, so line 7 tells you where startup actually
   // got to. Calibrate happens last.
   pros::lcd::initialize();

   // ONE task owns the brain screen, including all position and lift telemetry.
   pros::Task screenTask([]() {
   while (true) {
       // One snapshot of the same estimated pose used by autonomous navigation.
       // X/Y are inches relative to the origin set by chassis.setPose().
       const lemlib::Pose pose = chassis.getPose();
       // Keep these readings visible in every tuning mode.
       pros::lcd::set_text(0, fmt::format("X: {:.2f} in", pose.x));
       pros::lcd::set_text(1, fmt::format("Y: {:.2f} in", pose.y));
       pros::lcd::set_text(2, fmt::format("Target Angle: {:.1f} deg", liftTarget));
       pros::lcd::set_text(3, fmt::format("Motor Command: {:.1f}", liftLastOutput));
       pros::lcd::set_text(4, fmt::format("PIDon: {}", liftPIDEnabled ? 1 : 0));
       pros::lcd::set_text(5, fmt::format("Sen: {}", liftSensorOK ? "OK" : "BAD"));
       pros::lcd::set_text(6, fmt::format("Lift: {:.1f} deg  Hold: {:.0f}",
                                       liftAngleNow, liftTestHold));
       // FFslope must stay below kP or the loop has no stable equilibrium and
       // the lift runs to whichever end it is pointed at. Check this before
       // every step test.
       pros::lcd::print(7, "%s  FFsl %.2f/kP %.2f",
                        initStage, liftMaxFFSlope(), liftPID.kP);
       pros::delay(50);
   }
}, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT * 2, "screen");

   initStage = "motors";
   liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   liftRot.reset_position();
   pros::Task liftTaskHandle(liftTask);

   arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   arm.tare_position();

   initStage = "imu calib";
   chassis.calibrate();

   initStage = "done";
}


void disabled() {}
void competition_initialize() {}



const double ARM_TARGET_TOLERANCE = 4.0;

const int32_t ARM_MAX_VOLTAGE = 12000;
const int32_t ARM_MEDIUM_VOLTAGE = 12000;
const int32_t ARM_APPROACH_VOLTAGE = 12000;
const int32_t ARM_FINAL_VOLTAGE = 2500;
const int32_t ARM_HOLD_VOLTAGE = 3000;

 void moveArmAutonWithLiftTrigger(double targetAngle, double liftPeakAngle, int timeoutMs = 3000) {
    // The arm has only two valid targets.
    targetAngle = (targetAngle >= 0) ? 90.0 : -90.0;

    const double TOLERANCE = 4.0;
    bool liftTriggered = false;
    int elapsed = 0;

    while (elapsed < timeoutMs) {
        double currentPos = arm.get_position();
        double error = targetAngle - currentPos;

        // Trigger the lift when the arm crosses 30 degrees.
        if (!liftTriggered &&
            ((targetAngle > 0 && currentPos >= 30.0) ||
             (targetAngle < 0 && currentPos <= -30.0))) {
            setLiftTarget(liftPeakAngle);
            liftTriggered = true;
        }

        // Stop the movement loop when the arm reaches its target.
        if (std::fabs(error) <= TOLERANCE) {
            arm.move_voltage(targetAngle > 0 ? 3000 : -3000);
            return;
        }

        // Full power throughout the entire arm movement.
        arm.move_voltage(error > 0 ? 12000 : -12000);

        pros::delay(10);
        elapsed += 10;
    }

    // Maintain torque if the movement reaches its timeout.
    arm.move_voltage(targetAngle > 0 ? 3000 : -3000);
}


void moveArmAuton(double targetAngle, int timeoutMs = 3000) {
    targetAngle = (targetAngle >= 0) ? 90.0 : -90.0;

    int elapsed = 0;

    while (elapsed < timeoutMs) {
        double currentPos = arm.get_position();
        double error = targetAngle - currentPos;

        if (std::fabs(error) <= ARM_TARGET_TOLERANCE) {
            arm.move_voltage(targetAngle > 0
                ? ARM_HOLD_VOLTAGE
                : -ARM_HOLD_VOLTAGE);
            return;
        }

        // Full voltage for the entire flip.
        arm.move_voltage(error > 0
            ? ARM_MAX_VOLTAGE
            : -ARM_MAX_VOLTAGE);

        pros::delay(10);
        elapsed += 10;
    }

    arm.move_voltage(targetAngle > 0
        ? ARM_HOLD_VOLTAGE
        : -ARM_HOLD_VOLTAGE);
}

//reset lift and claw
void runR2Sequence() {
    // Disable lift PID so it does not fight the direct motor commands
    liftPIDEnabled = false;

    liftLeft.move_voltage(0);
    liftRight.move_voltage(0);

    // Run the R2 movement
    liftLeft.set_current_limit(2500);
    liftRight.set_current_limit(2500);

    liftLeft.move_voltage(-8000);
    liftRight.move_voltage(-8000);
    arm.move_voltage(-6000);

    pros::delay(2000);

    // Stop the motors
    liftLeft.move_voltage(0);
    liftRight.move_voltage(0);
    arm.move_voltage(0);

    // Re-enable PID and hold the lift at its current position
    enableLiftPID();
}

void autonomous() {
   liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

   liftAutonomousMode = true;
   enableLiftPID();

   moveArmAuton(-90);
   pros::delay(2000);
   arm.move_relative(540, 12000);

    /*
   chassis.setPose(0, 0, 180);
   
   
   claw.move_voltage(12000);

   // Toggle roller while the claw runs.

   setLiftTarget(145);
   chassis.moveToPoint(0, 8, 750, {.forwards=false, .minSpeed = 33});
   moveArmAuton(90);
   chassis.waitUntilDone();
   setLiftTarget(0);
   chassis.turnToHeading(0, 750, {.maxSpeed=60});
   chassis.moveToPoint(0, -3, 1000, {.forwards = false, .minSpeed = 33});
   pros::delay(500);
   moveArmAuton(-90);


   chassis.moveToPoint(5, 18, 1000, {.maxSpeed = 60});
   
   chassis.turnToHeading(225, 1000);
    chassis.moveToPoint(31, 41, 3000, {.forwards=false, .maxSpeed = 50});
    chassis.turnToHeading(180, 500);
    moveArmAuton(90);
    setLiftTarget(60);
    chassis.moveToPoint(26, 24, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    setLiftTarget(0);
    pros::delay(750);
    claw.move_voltage(-12000);
    pros::delay(500);
    chassis.moveToPoint(24, 35, 3000, {.forwards=false, .maxSpeed = 50});
    chassis.waitUntil(3);
    claw.move_voltage(12000);
    moveArmAuton(-90);
    chassis.turnToHeading(315, 500);
    chassis.moveToPoint(57, 9, 5000, {.forwards=false, .maxSpeed = 50});
    chassis.turnToHeading(-90, 500);
    moveArmAuton(90);
    setLiftTarget(100);
    chassis.moveToPoint(24, 16, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    setLiftTarget(60);
    pros::delay(500);
    claw.move_voltage(-12000);
    pros::delay(1000);
*/
   //standalone skills

   //return to loader
   /*
chassis.setPose(-47.223, -34.389, 0);
setLiftTarget(0);
moveArmAuton(-90);
chassis.moveToPoint(-47.223, -34.389-24, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(85, 1000);
chassis.moveToPoint(-46.223-13,-34.389-25, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(6);
    claw.move_voltage(12000);
pros::delay(1000);
//third pin
chassis.setPose(-62.253, -58.81, 90);

chassis.moveToPoint(-62.253+12, -58.81, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(0, 1000);
moveArmAuton(90);
setLiftTarget(150);
chassis.moveToPoint(-62.253+15.5, -58.81+27, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.waitUntilDone();
setLiftTarget(120);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);

//return to loader
chassis.setPose(-47.223, -34.389, 0);
chassis.moveToPoint(-47.223, -34.389-28, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(4);
pros::Task r2Task([] {
    runR2Sequence();
});

pros::delay(2000);
moveArmAuton(-90);
chassis.turnToHeading(85, 1000);
chassis.moveToPoint(-46.223-13,-34.389-30.5, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(6);
    claw.move_voltage(12000);
pros::delay(1000);

//pin 4
chassis.setPose(-62.253, -58.81, 90);
chassis.moveToPoint(-62.253+12, -58.81, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(-5, 1000);
moveArmAuton(90);
setLiftTarget(180);
chassis.moveToPoint(-62.253+14, -58.81+27, 2000, {.maxSpeed=50, .minSpeed=30});

chassis.waitUntilDone();
setLiftTarget(150);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);

//back to loader
chassis.setPose(-47.223, -34.389, 0);
setLiftTarget(0);
moveArmAuton(-90);
chassis.moveToPoint(-47.223, -34.389-27, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(85, 1000);
chassis.moveToPoint(-46.223-14,-34.389-28, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(6);
    claw.move_voltage(12000);
pros::delay(1000);

chassis.setPose(0,0,0);
chassis.moveToPoint(0, 16, 2000, {.maxSpeed=50});
chassis.turnToHeading(-90, 1000);
chassis.moveToPoint(-10, 16, 2000, {.maxSpeed=50});
chassis.turnToHeading(-25, 1000);
chassis.moveToPoint(-55, 62, 7000, {.maxSpeed=50});
moveArmAuton(90);
setLiftTarget(150);
chassis.waitUntilDone();
setLiftTarget(90);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);
   */
}


// Turns the LIFT_TUNE_MODE number into text so it can be shown on the screen.
// Proves which mode actually compiled in, rather than which one you edited.
#define LIFT_STR2(x) #x
#define LIFT_STR(x) LIFT_STR2(x)
void opcontrol() {
    liftAutonomousMode = false;
    liftPIDEnabled = false;

    initStage = "opc mode " LIFT_STR(LIFT_TUNE_MODE);

#if LIFT_TUNE_MODE == 1

    // MODE 1: Gravity measurement

    liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    liftTestHold = 0;
    long ticks = 0;

    controller.rumble(".");
    pros::delay(60);
    controller.print(0, 0, "M1 ready       ");
    pros::delay(60);

    while (true) {
        ticks++;

        getLiftAngle();

        chassis.arcade(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)
        );

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftLeft.move_voltage(9000);
            liftRight.move_voltage(9000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftLeft.move_voltage(-9000);
            liftRight.move_voltage(-9000);
        }
        else {
            liftLeft.move(liftTestHold);
            liftRight.move(liftTestHold);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.move_voltage(12000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            claw.move_voltage(-12000);
        }
        else {
            claw.brake();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            arm.move_voltage(9000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            arm.move_voltage(-9000);
        }
        else {
            arm.move_voltage(0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            liftTestHold += 1;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            liftTestHold -= 1;
        }

        if (liftTestHold > LIFT_FF_MAX) {
            liftTestHold = LIFT_FF_MAX;
        }

        if (liftTestHold < -LIFT_FF_MAX) {
            liftTestHold = -LIFT_FF_MAX;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            liftRot.reset_position();
            liftAngleNow = 0;
        }

        if (ticks % 10 == 1) {
            controller.print(
                0, 0, "M1 a%d h%d  ",
                (int)liftAngleNow,
                (int)liftTestHold
            );
        }

        pros::delay(10);
    }

#elif LIFT_TUNE_MODE == 2

    // MODE 2: PID step test

    liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    enableLiftPID();

    long ticks = 0;

    controller.rumble(".");
    pros::delay(60);
    controller.print(0, 0, "M2 ready       ");
    pros::delay(60);

    while (true) {
        ticks++;

        chassis.arcade(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)
        );

        bool jogUp =
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        bool jogDown =
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (jogUp || jogDown) {
            liftPIDEnabled = false;

            double cmd = jogUp ? 90 : -50;

            liftLeft.move(cmd);
            liftRight.move(cmd);

            liftLastOutput = cmd;
            getLiftAngle();
        }
        else if (!liftPIDEnabled) {
            enableLiftPID();
        }
        else {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                setLiftTarget(30);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                setLiftTarget(100);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                setLiftTarget(180);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                setLiftTarget(0);
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            liftPIDEnabled = false;

            liftRot.reset_position();
            liftAngleNow = 0;

            pros::delay(50);
            enableLiftPID();
        }

        if (ticks % 10 == 1) {
            controller.print(
                0, 0, "M2 a%d t%d  ",
                (int)liftAngleNow,
                (int)liftTarget
            );
        }

        pros::delay(10);
    }

#else

    // MODE 0: Normal driver control

    liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Prevent the arm from coasting after R2.
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    liftLeft.move_voltage(0);
    liftRight.move_voltage(0);

    const int32_t LIFT_DRIVER_VOLTAGE = 12000;

    int lastArmDirection = 0;

    bool r2TaskRunning = false;
    uint32_t r2TaskStart = 0;

    bool arm540Running = false;
    double arm540Target = 0;
    uint32_t arm540StartTime = 0;

    while (true) {

        // ==========================================
        // DRIVE
        // ==========================================

        chassis.tank(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
        );

        // ==========================================
        // R2 RESET
        // ==========================================

        // R2 is allowed to start regardless of arm540Running.
        if (controller.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_R2)
            && !r2TaskRunning) {

            r2TaskRunning = true;
            r2TaskStart = pros::millis();

            // Cancel any active arm position movement.
            arm540Running = false;

            lastArmDirection = 0;

            // Immediately start moving downward.
            arm.move_voltage(-6000);
        }

        // ==========================================
        // CLAW
        // ==========================================

        int clawCommand = 0;

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            clawCommand = 12000;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            clawCommand = -12000;
        }

        if (clawCommand != 0) {
            claw.move_voltage(clawCommand);
        }
        else {
            claw.move_voltage(3000);
        }

        // ==========================================
        // ARM AND RESET SEQUENCING
        // ==========================================

        if (r2TaskRunning) {

            // Continue resetting downward.
            liftLeft.set_current_limit(2500);
            liftRight.set_current_limit(2500);

            liftLeft.move_voltage(-8000);
            liftRight.move_voltage(-8000);

            arm.move_voltage(-6000);

            if (pros::millis() - r2TaskStart >= 2000) {

                // Finish reset and actively brake.
                liftLeft.brake();
                liftRight.brake();
                arm.brake();

                r2TaskRunning = false;

                // Reset the arm cycle.
                arm540Running = false;
                lastArmDirection = 0;
            }
        }

        else {

            // --------------------------------------
            // INITIAL R1 MOVEMENT
            // --------------------------------------

            if (controller.get_digital_new_press(
                    pros::E_CONTROLLER_DIGITAL_R1)
                && !arm540Running) {

                arm540Target = arm.get_position() + 400.0;
                arm540StartTime = pros::millis();

                lastArmDirection = 0;

                int result = arm.move_absolute(
                    arm540Target,
                    200
                );

                arm540Running = (result == 1);
            }

            // --------------------------------------
            // MONITOR INITIAL MOVEMENT
            // --------------------------------------

            if (arm540Running) {

                double error =
                    arm540Target - arm.get_position();

                if (std::fabs(error) <= 5.0) {

                    arm540Running = false;

                    lastArmDirection = 1;

                    arm.move_voltage(2500);
                }
                else if (pros::millis() - arm540StartTime >= 4000) {

                    arm540Running = false;

                    lastArmDirection = 0;

                    arm.brake();
                }
            }

            // --------------------------------------
            // NORMAL ARM CONTROL
            // --------------------------------------

            if (!arm540Running) {

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {

                    arm.move_voltage(12000);
                    lastArmDirection = 1;
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {

                    arm.move_voltage(-12000);
                    lastArmDirection = -1;
                }
                else {

                    if (lastArmDirection == 1) {
                        arm.move_voltage(2500);
                    }
                    else if (lastArmDirection == -1) {
                        arm.move_voltage(-3000);
                    }
                    else {
                        arm.brake();
                    }
                }
            }
        }

        // ==========================================
        // MANUAL LIFT
        // ==========================================

        if (r2TaskRunning) {
            // R2 already controls the lift.
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {

            liftLeft.set_current_limit(2500);
            liftRight.set_current_limit(2500);

            liftLeft.move_voltage(LIFT_DRIVER_VOLTAGE);
            liftRight.move_voltage(LIFT_DRIVER_VOLTAGE);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

            liftLeft.set_current_limit(2500);
            liftRight.set_current_limit(2500);

            liftLeft.move_voltage(-8000);
            liftRight.move_voltage(-8000);
        }
        else {

            liftLeft.set_current_limit(2500);
            liftRight.set_current_limit(2500);

            liftLeft.brake();
            liftRight.brake();
        }

        pros::delay(10);
    }

#endif
}
