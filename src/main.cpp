#include "main.h"
#include "fmt/core.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"
#include "pros/distance.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>




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
pros::MotorGroup leftMotors({-9, -8, -1}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({19, 11, 17}, pros::MotorGearset::green);

pros::Distance rightDistance(15);
// Left distance sensor smart port.
pros::Distance leftDistance(10);


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
const double LIFT_AUTON_DOWN_MAX = 35.0;
const double LIFT_AUTON_DOWN_SLEW = 8.0;




double liftSlewRate = 25;   // output change allowed per 10 ms loop, tune this LAST
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
// HEIGHT TABLE
//
// Put the lift at each angle and measure the same point on the claw above the
// tile with a tape, keeping the separate arm at the same angle each time.
// Heights must come out strictly increasing; include the actual bottom height.
// Add more rows if you want it tighter, just bump kHN to match.
// ---------------------------------------------------------------------
const int kHN = 6;
const double kHAngle[kHN]  = {0, 50, 100, 150, 200, 220};
double       kHInches[kHN] = {0, 0,  0,   0,   0,   0};   // <-- FILL THESE IN




bool liftHeightCalibrationValid() {
   for (int i = 0; i < kHN; i++) {
       if (!std::isfinite(kHAngle[i]) || !std::isfinite(kHInches[i])) return false;
       if (kHAngle[i] < LIFT_MIN || kHAngle[i] > LIFT_MAX) return false;
       if (i > 0 && (kHAngle[i] <= kHAngle[i - 1] ||
                     kHInches[i] <= kHInches[i - 1])) return false;
   }
   return true;
}

double inchesToLiftAngle(double inches) {
   // Reject unmeasured heights and unreachable requests instead of driving to
   // the top with the placeholder table or silently using a different height.
   if (!liftHeightCalibrationValid() || !std::isfinite(inches) ||
       inches < kHInches[0] || inches > kHInches[kHN - 1]) {
       return std::numeric_limits<double>::quiet_NaN();
   }
   if (inches <= kHInches[0]) return kHAngle[0];
   if (inches >= kHInches[kHN - 1]) return kHAngle[kHN - 1];
   for (int i = 0; i < kHN - 1; i++) {
       if (inches <= kHInches[i + 1]) {
           double t = (inches - kHInches[i]) / (kHInches[i + 1] - kHInches[i]);
           return kHAngle[i] + t * (kHAngle[i + 1] - kHAngle[i]);
       }
   }
   return kHAngle[kHN - 1];
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




// Ask for a height above the tile. False leaves the existing target unchanged.
// Enable PID first, just as for setLiftTarget().
bool setLiftHeight(double inches) {
   const double angle = inchesToLiftAngle(inches);
   if (!std::isfinite(angle)) return false;
   setLiftTarget(angle);
   return true;
}




bool liftAtTarget(double tolerance = 2.0) {
   return std::fabs(getLiftAngle() - liftTarget) <= tolerance;
}




void waitUntilLiftAt(double tolerance = 2.0, int timeoutMs = 2000) {
   int elapsed = 0;
   while (!liftAtTarget(tolerance) && elapsed < timeoutMs) {
       pros::delay(10);
       elapsed += 10;
   }
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
   liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   liftRot.reset_position();
   pros::Task liftTaskHandle(liftTask);

   arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   arm.tare_position();

   initStage = "imu calib";
   chassis.calibrate();

   initStage = "done";
}




void disabled() {}
void competition_initialize() {}




ASSET(example_txt);




// Old open loop lift helpers. These fight the PID task, so only use them with
// liftPIDEnabled false.
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




const double SLOW_ZONE_DELTA = 40.0;
const double HOLD_ZONE_DELTA = 5.0;
const int32_t MAX_SPEED = 12000;
const int32_t APPROACH_SPEED = 5000;
const int32_t GENTLE_HOLD = 3000;




void moveArmAuton(double targetAngle, int timeoutMs = 2000) {
   int elapsed = 0;
   while (elapsed < timeoutMs) {
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
           arm.move_voltage((error > 0) ? APPROACH_SPEED : -APPROACH_SPEED);
       }
       pros::delay(20);
       elapsed += 20;
   }
}


// Flip to the other side while the lift rises, then settle at the requested
// scoring angle when the arm passes upright (0 degrees).
void scorePin(double finalLiftAngle, int timeoutMs = 2500) {
   if (!std::isfinite(finalLiftAngle)) return;
   finalLiftAngle = std::clamp(finalLiftAngle, LIFT_MIN, LIFT_MAX);

   const double startAngle = arm.get_position();
   if (!std::isfinite(startAngle)) return;
   const int direction = startAngle < 0 ? 1 : -1;
   const double armTarget = direction * 90.0;
   const double raisedLiftAngle = std::min(finalLiftAngle + 30.0, LIFT_MAX);
   bool liftRaised = false;
   bool passedUpright = false;
   bool armAtTarget = false;

   int elapsed = 0;
   for (; elapsed < timeoutMs; elapsed += 20) {
       const double armAngle = arm.get_position();
       if (!std::isfinite(armAngle)) break;

       // Give the arm a 15-degree head start, then let the lift rise in parallel.
       if (!liftRaised && direction * (armAngle - startAngle) >= 15.0 &&
           direction * armAngle < 0) {
           setLiftTarget(raisedLiftAngle);
           liftRaised = true;
       }
       if (!passedUpright && direction * armAngle >= 0) {
           setLiftTarget(finalLiftAngle);
           passedUpright = true;
       }

       const double remaining = direction * (armTarget - armAngle);
       if (remaining <= HOLD_ZONE_DELTA) {
           arm.move_voltage(direction * GENTLE_HOLD);
           armAtTarget = true;
           break;
       }
       const int32_t voltage = remaining <= SLOW_ZONE_DELTA
                                   ? APPROACH_SPEED : MAX_SPEED;
       arm.move_voltage(direction * voltage);
       pros::delay(20);
   }

   // Keep the requested angle as the final PID target, even after a timeout.
   setLiftTarget(finalLiftAngle);
   if (!armAtTarget) arm.move_voltage(0);
   if (armAtTarget) {
       waitUntilLiftAt(3.0, timeoutMs - elapsed);
   }
}




// =====================================================================
// DISTANCE SENSOR ODOMETRY RESET
// =====================================================================
//
// Corrects ONE axis of the odometry pose by measuring a wall whose field
// coordinate you already know, using the right hand sensor on port 15.
//
// Three things have to be accounted for or the "correction" leaves odom worse
// than not touching it:
//
//   1. Robot heading. The sensor turns with the robot, so the beam direction in
//      FIELD coordinates is the robot heading plus the mounting angle.
//   2. Mounting angle. Right facing is 90 degrees clockwise of robot forward.
//   3. Mounting offset. The sensor does not sit at the tracking center, so its
//      field position is the robot pose plus that offset rotated into the field
//      frame. Skipping this bakes an error into every reset, and because the
//      offset rotates with the robot the error changes sign as it turns, which
//      is nastier than a fixed bias.
//
// It also handles the beam striking the wall at an angle, which is the part that
// usually gets missed. A sensor 10 inches from a wall reads 10 only when it is
// square to it; at 30 degrees off it reads 11.5. Projecting the beam vector onto
// the wall normal takes care of that on its own.
//
// ASSUMPTION worth being explicit about: this trusts the heading and corrects
// only position. One distance sensor cannot tell a heading error apart from a
// position error, so if the IMU has drifted this will confidently write in a
// wrong answer. Guard rails below reject the readings most likely to be junk,
// but they cannot catch a bad heading.
//
// Usage, e.g. squaring up on the right wall at x = 72 before a scoring run:
//     DistanceFix fix = resetOdomFromWall(Wall::PlusX, 72.0);
//     if (!fix.ok) { /* fix.reason says why, odom left untouched */ }


// ---- MEASURE THESE ON THE ROBOT, they are guesses ----
// Offsets from the TRACKING CENTER, the point odom actually reports, in inches.
const double DIST_OFFSET_FORWARD = 0.0;   // + toward the front of the robot
const double DIST_OFFSET_RIGHT   = 6.5;   // + toward the right of the robot
// Mounting angle relative to robot forward, degrees clockwise.
// 90 means pointing straight out the right side.
const double DIST_SENSOR_ANGLE   = 90.0;

// TODO: Measure the left sensor offsets from the tracking center, in inches.
const double LEFT_DIST_OFFSET_FORWARD = 0.0;
const double LEFT_DIST_OFFSET_RIGHT = -6.5; // negative means left; provisional
const double LEFT_DIST_SENSOR_ANGLE = -90.0;
// TODO: Set the negative-Y wall coordinate in YOUR odometry frame.
// NAN disables this reset until configured; -72 only fits a field-centered origin.
const double LOADER2_WALL_Y = NAN;

// Sensor mounting geometry relative to the tracking center.
struct DistanceMount {
    double forward;
    double right;
    double angle;
};
const DistanceMount RIGHT_DISTANCE_MOUNT{
    DIST_OFFSET_FORWARD, DIST_OFFSET_RIGHT, DIST_SENSOR_ANGLE};
const DistanceMount LEFT_DISTANCE_MOUNT{
    LEFT_DIST_OFFSET_FORWARD, LEFT_DIST_OFFSET_RIGHT, LEFT_DIST_SENSOR_ANGLE};

// Which wall face the beam is pointed at. Names are the direction of the wall's
// outward axis, so PlusX is the wall you reach by driving toward +x.
enum class Wall { PlusX, MinusX, PlusY, MinusY };

struct DistanceFix {
   bool ok;
   double measured;      // where the sensor put the wall, in field coordinates
   double correction;    // how far the pose was moved, 0 when rejected
   double spread;        // inches between the closest and furthest sample
   const char* reason;   // why it was rejected
};


// Anything slower than this counts as stopped, in RPM.
const double DIST_STILL_RPM = 5.0;

// This reset is only meant to run with the robot stationary. If the drivetrain
// is still turning, the samples are smeared across a moving vantage point and
// the pose is shifting out from under the correction as it is applied.
// Checks the first motor of each side, which is enough to catch a chassis that
// is still coasting or a motion that was never waited on.
static bool driveIsStopped() {
   return std::fabs(leftMotors.get_actual_velocity()) < DIST_STILL_RPM &&
          std::fabs(rightMotors.get_actual_velocity()) < DIST_STILL_RPM;
}


// Median plus spread. Because the robot is stationary the samples SHOULD agree
// closely, so how much they disagree is real information: a wide spread means
// the beam is catching an edge, a gap, or something reflective. A median on its
// own would quietly hide exactly that, which is why the spread comes back too.
static std::int32_t distanceMedianMM(pros::Distance& sensor, int samples, std::int32_t& spreadMM) {
   std::int32_t v[15];
   if (samples > 15) samples = 15;
   if (samples < 1) samples = 1;
   for (int i = 0; i < samples; i++) {
       v[i] = sensor.get_distance();
       if (v[i] == PROS_ERR || v[i] <= 0 || v[i] >= 9999) {
           spreadMM = 0;
           return v[i];
       }
       if (i + 1 < samples) pros::delay(10);
   }
   std::sort(v, v + samples);
   spreadMM = v[samples - 1] - v[0];
   return v[samples / 2];
}


// Samples default to 9 rather than a handful, because standing still costs
// nothing but 80 ms and more samples make both the median and the spread mean
// something.
static DistanceFix resetOdomFromSensor(pros::Distance& sensor,
                              double offsetForward, double offsetRight,
                              double sensorAngle, Wall wall, double wallCoord,
                              double maxCorrection = 6.0,
                              int minConfidence = 40,
                              int samples = 9,
                              double maxSpreadIn = 0.6) {
   DistanceFix r{false, 0.0, 0.0, 0.0, "ok"};

   if (!driveIsStopped()) {
       r.reason = "robot still moving, call waitUntilDone first";
       return r;
   }

   std::int32_t spreadMM = 0;
   std::int32_t mm = distanceMedianMM(sensor, samples, spreadMM);
   r.spread = spreadMM / 25.4;

   if (mm == PROS_ERR || mm <= 0) { r.reason = "sensor error, check distance sensor port"; return r; }
   if (mm >= 9999) { r.reason = "nothing in range"; return r; }
   // Confidence is only meaningful past 200 mm, per the PROS docs.
   const auto confidence = sensor.get_confidence();
   if (mm > 200 && (confidence == PROS_ERR || confidence < minConfidence)) {
       r.reason = "low confidence"; return r;
   }
   // Standing still, a flat wall reads within a few mm. Disagreement past this
   // means the beam is not on a flat wall, so the median is not worth trusting.
   if (r.spread > maxSpreadIn) {
       r.reason = "samples disagree, beam not on a flat wall";
       return r;
   }

   const double DEG = 3.14159265358979323846 / 180.0;
   double d = mm / 25.4;   // the sensor reports millimetres, odom is in inches

   lemlib::Pose p = chassis.getPose();

   // LemLib heading is a compass bearing: 0 is +y, clockwise positive.
   // So forward is (sin, cos) and right is that turned 90 clockwise.
   double th = p.theta * DEG;
   double fx = std::sin(th),  fy = std::cos(th);
   double rx = std::cos(th),  ry = -std::sin(th);

   // (1) and (3): where the sensor actually is, in field coordinates
   double sx = p.x + fx * offsetForward + rx * offsetRight;
   double sy = p.y + fy * offsetForward + ry * offsetRight;

   // (2): where the beam is pointed, in field coordinates
   double bth = (p.theta + sensorAngle) * DEG;
   double bx = std::sin(bth), by = std::cos(bth);

   // where the beam says it struck
   double hx = sx + bx * d;
   double hy = sy + by * d;

   bool xAxis = (wall == Wall::PlusX || wall == Wall::MinusX);
   double along = xAxis ? bx : by;   // beam component along the wall normal
   double hit = xAxis ? hx : hy;

   // A beam nearly parallel to the wall carries almost no information about that
   // axis, and a small heading error there turns into a huge position error. It
   // also has to be pointed AT the named wall, not away from it. 0.5 is 60
   // degrees off perpendicular, which is about as oblique as is still useful.
   double towardWall = (wall == Wall::PlusX || wall == Wall::PlusY) ? along : -along;
   if (towardWall < 0.5) { r.reason = "beam too oblique or facing away"; return r; }

   double correction = wallCoord - hit;

   // A big correction almost always means the beam found a robot or a game
   // element rather than the wall. Refuse it rather than teleport the pose.
   if (std::fabs(correction) > maxCorrection) {
       r.measured = hit;
       r.reason = "correction too large, probably not the wall";
       return r;
   }

   if (xAxis) chassis.setPose(p.x + correction, p.y, p.theta);
   else chassis.setPose(p.x, p.y + correction, p.theta);

   r.ok = true;
   r.measured = hit;
   r.correction = correction;
   return r;
}
// Explicit sensor and mounting geometry for resets using either sensor.
DistanceFix resetOdomFromWall(pros::Distance& sensor, const DistanceMount& mount,
                              Wall wall, double wallCoord,
                              double maxCorrection = 6.0,
                              int minConfidence = 40,
                              int samples = 9,
                              double maxSpreadIn = 0.6) {
    return resetOdomFromSensor(sensor, mount.forward, mount.right, mount.angle,
                              wall, wallCoord, maxCorrection, minConfidence,
                              samples, maxSpreadIn);
}

// Existing right-side API and defaults are preserved.
DistanceFix resetOdomFromWall(Wall wall, double wallCoord,
                              double maxCorrection = 6.0,
                              int minConfidence = 40,
                              int samples = 9,
                              double maxSpreadIn = 0.6) {
    return resetOdomFromSensor(rightDistance, DIST_OFFSET_FORWARD,
                              DIST_OFFSET_RIGHT, DIST_SENSOR_ANGLE, wall,
                              wallCoord, maxCorrection, minConfidence,
                              samples, maxSpreadIn);
}

// Call while stopped, with the selected sensor facing the +X wall.
// Always pass the matching mount so the correct offset and beam angle are used.
DistanceFix resetAtGoal1(pros::Distance& sensor, const DistanceMount& mount) {
    return resetOdomFromWall(sensor, mount, Wall::PlusX, 18, 4.0);
}

// Same wall, baseline, and correction limit as goal 1.
DistanceFix resetAtGoal2(pros::Distance& sensor, const DistanceMount& mount) {
    return resetOdomFromWall(sensor, mount, Wall::PlusX, 18, 4.0);
}

// Start the arm at 2 inches without blocking the 5-inch lift/intake trigger.
void followPathWithMechanisms(const asset& path, float lookahead, int timeoutMs,
                              bool forwards, double armAngle, bool moveLift,
                              double liftAngle, bool startIntakeAtFive) {
   chassis.follow(path, lookahead, timeoutMs, forwards);
   chassis.waitUntil(2);
   pros::Task armTask([armAngle]() { moveArmAuton(armAngle); });

   if (moveLift || startIntakeAtFive) {
      chassis.waitUntil(5);
      if (moveLift) setLiftTarget(liftAngle);
      if (startIntakeAtFive) claw.move_voltage(12000);
   }

   chassis.waitUntilDone();
   armTask.join();
}

ASSET(loader_txt);
ASSET(goal_txt);
ASSET(loader2_txt);
ASSET(goal2_txt);
ASSET(uturn_txt);
ASSET(yellow_txt);
ASSET(green_txt);

void autonomous() {
   liftAutonomousMode = true;
   enableLiftPID();

   chassis.setPose(0, 0, 180);
   claw.move_voltage(12000);

   // Toggle roller while the claw runs.
   chassis.moveToPoint(0, 8, 500, {.forwards=false, .minSpeed = 33});
   moveArmAuton(90);
   chassis.turnToHeading(0, 750, {.maxSpeed=60});
   chassis.moveToPoint(0, -1, 500, {.forwards = false, .minSpeed = 33});
   pros::delay(500);
   chassis.moveToPoint(0, 6, 500, {.minSpeed = 33});
   pros::delay(500);
   chassis.moveToPoint(0, -1, 500, {.forwards = false, .minSpeed = 33});
   pros::delay(500);

   // Score the preload, then lower the lift before releasing it.
   setLiftTarget(100);
   chassis.moveToPose(-24, 16, -90, 6000, {.lead = 0.4, .maxSpeed = 70, .minSpeed = 30});
   moveArmAuton(90);
   chassis.waitUntilDone();

   setLiftTarget(0);
   claw.move_voltage(-12000);
   pros::delay(300);
   claw.brake();

   // Collect the first pin, then score it.
   
//    chassis.moveToPoint(-3, 17, 3000, {.forwards=false, .minSpeed=20});
//    chassis.turnToPoint(22, 36.95, 6000, {.forwards=false, .minSpeed=60, .earlyExitRange=6});
//    chassis.moveToPoint(22, 36.95, 6000, {.forwards=false, .minSpeed=60});
      chassis.moveToPose(25, 38.95, -135, 6000, {.forwards=false, .lead=0});
      moveArmAuton(-90);
    claw.move_voltage(12000);
    chassis.waitUntilDone();
    chassis.moveToPose(38, 41, 90, 2000, {.maxSpeed=55});
   moveArmAuton(90);
   pros::delay(100);
   setLiftTarget(60);
   chassis.waitUntilDone();
   setLiftTarget(0);
   pros::delay(200);
   claw.move_voltage(-12000);
   // Re-anchor at the first-goal scoring pose used to tune the U-turn.
   chassis.setPose(38, 41, 90);
   // Sweep around to the second toggle in one backward motion.
   chassis.follow(uturn_txt, 9, 6000, false);
   chassis.waitUntilDone();

   // Contact with the bottom wall fixes Y, but does not tell us X. Keep the
   // measured X; at this handoff the robot's front points right.
   const lemlib::Pose wallPose = chassis.getPose();
   chassis.setPose(wallPose.x, 61, 90);
   chassis.follow(yellow_txt, 4.5, 3000, true);
   chassis.waitUntilDone();
   claw.move_voltage(12000);
   // Green heads west. Follow it backwards to keep the front pointing right
   // at the handoff instead of turning beside the bottom wall.
   followPathWithMechanisms(green_txt, 5, 6500, false, -90, false, 0, false);
   claw.brake();
   return; // Test these paths before restoring the scoring sequence below.

   // Previous scoring sequence, to be adapted after the new pickup path is tested.
   chassis.turnToHeading(0, 500, {.maxSpeed=80});
   chassis.moveToPose(52, 33, 0, 1500, {.lead=0, .maxSpeed=80});
   moveArmAuton(90);
   pros::delay(100);
   setLiftTarget(90);
   chassis.moveToPose(52, 33, 0, 1500, {.maxSpeed=80});
   chassis.waitUntilDone();
   setLiftTarget(60);
   claw.move_voltage(-12000);

   // Loader pin #3: use the scoring pose as a local origin for the pickup path.
   chassis.setPose(0,0,0);
    followPathWithMechanisms(loader_txt, 8, 3000, false, -90, true, 0, true);
   pros::delay(1000);
   followPathWithMechanisms(goal_txt, 8, 3000, true, 90, true, 120, false);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);

   // Loader pin #4: repeat the lane entry and raise to 150 on the way out.
   followPathWithMechanisms(loader_txt, 8, 3000, false, -90, true, 0, true);
   pros::delay(1000);
   followPathWithMechanisms(goal_txt, 8, 3000, true, 90, true, 120, false);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);

   // Loader pin #5: the pictured goal is the origin of this new path pair.
   followPathWithMechanisms(loader_txt, 8, 6000, false, -90, true, 0, true);
   chassis.setPose(-13.5, -37.5, 90);
   pros::delay(1000);
   followPathWithMechanisms(goal2_txt, 8, 6000, true, 90, true, 60, false);
   setLiftTarget(0);
   claw.move_voltage(-12000);
   pros::delay(500);
   claw.brake();


   //Loader 6
   chassis.turnToHeading(0, 1000, {.maxSpeed=45});
   chassis.waitUntilDone();
   chassis.setPose(0,0,0);
   followPathWithMechanisms(loader2_txt, 8, 6000, false, -90, true, 0, true);
   pros::delay(1000);
   // The robot is stationary at the loader; anchor the return to goal2's start.
   chassis.setPose(-14.5, -37, 90);
   followPathWithMechanisms(goal2_txt, 8, 6000, true, 90, true, 60, false);
   setLiftTarget(0);
   claw.move_voltage(-12000);
   pros::delay(500);
   claw.brake();

   //Loader 7
   chassis.turnToHeading(0, 1000, {.maxSpeed=45});
   chassis.waitUntilDone();
   chassis.setPose(0,0,0);
   followPathWithMechanisms(loader2_txt, 8, 6000, false, -90, true, 0, true);
   pros::delay(1000);
   chassis.setPose(-14.5, -37, 90);
   followPathWithMechanisms(goal2_txt, 8, 6000, true, 90, true, 60, false);
   setLiftTarget(0);
   claw.move_voltage(-12000);
   pros::delay(500);
   claw.brake();

   //Loader 8
   chassis.turnToHeading(0, 1000, {.maxSpeed=45});
   chassis.waitUntilDone();
   chassis.setPose(0,0,0);
   followPathWithMechanisms(loader2_txt, 8, 6000, false, -90, true, 0, true);
   pros::delay(1000);
   chassis.setPose(-14.5, -37, 90);
   followPathWithMechanisms(goal2_txt, 8, 6000, true, 90, true, 60, false);
   setLiftTarget(0);
   claw.move_voltage(-12000);
   pros::delay(500);
   claw.brake();

   // Middle Pin
   chassis.turnToHeading(0, 1000, {.maxSpeed=45});
   chassis.waitUntilDone();
   chassis.setPose(0,0,0);
   followPathWithMechanisms(loader2_txt, 8, 6000, false, -90, true, 0, true);
   chassis.setPose(-13.5, -37.5, 90);
   pros::delay(1000);
   chassis.moveToPose(48, 24, 45, 6000);

}





// Turns the LIFT_TUNE_MODE number into text so it can be shown on the screen.
// Proves which mode actually compiled in, rather than which one you edited.
#define LIFT_STR2(x) #x
#define LIFT_STR(x) LIFT_STR2(x)


void opcontrol() {
   liftAutonomousMode = false;
   // THE decisive marker. screenTask paints this on brain screen line 7.
   // If line 7 reads "opc mode 1" then opcontrol is running and the right mode
   // compiled in, which means missing controller text is a controller problem,
   // not a code-never-reached problem. If it still reads "imu calib" or "done",
   // opcontrol is not running and the controller was never going to show text.
   initStage = "opc mode " LIFT_STR(LIFT_TUNE_MODE);

   arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   arm.tare_position();




#if LIFT_TUNE_MODE == 1
   // -----------------------------------------------------------------
   // MODE 1, gravity measurement.
   //   L1 / L2   jog the lift up and down
   //   X / Y     add or subtract 1 from the hold value
   //   B / DOWN  claw, so you can load a Pin for the test
   //   R1 / R2   arm
   // Let go of L1 and L2, then tap X or Y until the lift sits perfectly still.
   // Read the Lift and Hold lines off the brain screen and write the pair into
   // kGHold[]. Do about 0, 50, 100, 150, 200, 240.
   // Coast brake so the motor is not holding it for you.
   // -----------------------------------------------------------------
   liftPIDEnabled = false;
   liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);




   liftTestHold = 0;
   long ticks = 0;

   // Unmistakable proof that opcontrol actually started and that this is the
   // MODE 1 build. One buzz, and text on the controller before the loop even
   // begins, so you are never left guessing whether the upload took.
   // rumble and print share ONE controller write slot, and PROS drops any write
   // inside 50 ms of the last one. Back to back, the rumble wins and the print
   // is silently thrown away. They have to be spaced.
   controller.rumble(".");
   pros::delay(60);
   controller.print(0, 0, "M1 ready       ");
   pros::delay(60);




   while (true) {
       ticks++;

       // liftTask is idle in this mode, so read the sensor here to keep the
       // brain screen alive.
       getLiftAngle();

       chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                      controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));




       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
           liftLeft.move_voltage(9000);
           liftRight.move_voltage(9000);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
           liftLeft.move_voltage(-9000);
           liftRight.move_voltage(-9000);
       } else {
           liftLeft.move(liftTestHold);
           liftRight.move(liftTestHold);
       }




       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
           claw.move_voltage(12000);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
           claw.move_voltage(-12000);
       } else {
           claw.brake();
       }




       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
           arm.move_voltage(9000);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
           arm.move_voltage(-9000);
       } else {
           arm.move_voltage(0);
       }




       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) liftTestHold += 1;
       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) liftTestHold -= 1;

       // Never record a hold the feedforward will not be allowed to use later.
       if (liftTestHold >  LIFT_FF_MAX) liftTestHold =  LIFT_FF_MAX;
       if (liftTestHold < -LIFT_FF_MAX) liftTestHold = -LIFT_FF_MAX;

       // Lift on the bottom stop, then LEFT to re-zero without a reboot.
       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
           liftRot.reset_position();
           liftAngleNow = 0;
       }




       // Integers only, and no faster than the controller's 50 ms write limit.
       // The M1 prefix is there so you can always tell which build is running
       // on the brain. Trailing spaces wipe the previous, longer line.
       // The controller screen holds 15 characters per line. Anything longer is
       // rejected outright, so keep it short and pad to wipe the old text.
       if (ticks % 10 == 1) {
           controller.print(0, 0, "M1 a%d h%d  ",
                            (int)liftAngleNow, (int)liftTestHold);
       }




       pros::delay(10);
   }




#elif LIFT_TUNE_MODE == 2
   // -----------------------------------------------------------------
   // MODE 2, PID step test. Only run this once kGHold[] holds real measured
   // numbers and brain screen line 7 shows FFsl comfortably below kP.
   //
   //   A / B / X / Y   go to 30 / 100 / 180 / 0 lift degrees
   //   L1 / L2         manual override jog. PID suspends while held and picks
   //                   the lift up again from wherever you let go, so you can
   //                   park above a target and check it lands the same coming
   //                   down as it does going up.
   //   LEFT            re-zero the sensor, lift sitting on the bottom stop
   //
   // The brain shows X/Y, target angle, motor command, PID/sensor status,
   // and the measured lift angle.
   // Use the response to adjust the LiftPID gains near the top of the file.
   // Persistent error can also indicate an inaccurate gravity table or friction.
   // -----------------------------------------------------------------
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

       chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                      controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

       bool jogUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
       bool jogDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

       if (jogUp || jogDown) {
           liftPIDEnabled = false;
           double cmd = jogUp ? 90 : -50;
           liftLeft.move(cmd);
           liftRight.move(cmd);
           liftLastOutput = cmd;
           getLiftAngle();          // liftTask is idle, keep the screen alive
       } else if (!liftPIDEnabled) {
           enableLiftPID();         // re-arm at wherever the jog left it
       } else {
           if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) setLiftTarget(30);
           if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) setLiftTarget(100);
           if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) setLiftTarget(180);
           if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) setLiftTarget(0);
       }

       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
           liftPIDEnabled = false;
           liftRot.reset_position();
           liftAngleNow = 0;
           pros::delay(50);
           enableLiftPID();
       }

       if (ticks % 10 == 1) {
           controller.print(0, 0, "M2 a%d t%d  ",
                            (int)liftAngleNow, (int)liftTarget);
       }

       pros::delay(10);
   }




#else
   // -----------------------------------------------------------------
   // MODE 0, normal driver control.
   // -----------------------------------------------------------------
   liftPIDEnabled = false;




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

       // liftTask is idle in this mode, so read the sensor here to keep the
       // brain screen alive.
       getLiftAngle();

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
            claw.set_current_limit(2500);
           claw.move_voltage(12000);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            claw.set_current_limit(1000);
           claw.move_voltage(-6000);
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
#endif
}
