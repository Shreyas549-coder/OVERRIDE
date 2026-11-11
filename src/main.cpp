#include "main.h"
#include "fmt/core.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/ai_vision.hpp"
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


// =====================================================================
// AUTONOMOUS MODE
//   0 = normal autonomous
//   1 = Object ID / yellow finder. autonomous() does NOTHING except read the AI
//       Vision sensor and show the blobs it sees (brain screen lines 0-3,
//       controller line 0, and the PROS terminal). Point the rear sensor at a
//       pin, run autonomous, and check that it is boxed steadily. If it is not,
//       tune the AIV_YELLOW_* numbers below. Read AIV_PIN_CLASS_ID off here.
//   2 = Pin tracking demo. The robot rotates IN PLACE to keep the pin on the
//       drivetrain centreline. It never drives forward or backward.
// Set back to 0 when you are done.
//
// This macro is read in three places, not just autonomous(): aivInit() picks the
// detection type from it, and the screen task switches its layout on it. All
// three have to agree.
// =====================================================================
#define AUTON_MODE 0


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// motor groups
pros::MotorGroup leftMotors({-9, -8, -7}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({19, 11, 17}, pros::MotorGearset::green);


pros::Motor liftLeft(-1);
pros::Motor liftRight(12);
pros::Motor claw(3);
pros::Motor arm(15);
pros::Imu imu(21);

// rear facing AI Vision sensor
pros::AIVision aivision(14);


pros::Rotation verticalEnc(-18);


// single lift rotation sensor (DR4B sides are mechanically linked, so one sensor tracks both)
pros::Rotation liftRot(13);


// Third argument is the SIDEWAYS distance from the tracking centre to the
// wheel, positive to the right. Measured with a ruler: the wheel sits about
// half an inch LEFT of the centre line, so -0.5. It was 0, which told odometry
// the wheel was dead centre; a wheel that is off centre sweeps an arc during
// every turn and reports it as forward travel the robot never made. At 0.5 in
// that is about 1.6 in of phantom travel per 180 deg turn.
//
// The spin-in-place test does NOT measure this: over a full 360 the false
// travel points in every direction in turn and very nearly cancels, which is
// why 3 rotations only drifted 2.7 in. Partial turns are where it shows up.
lemlib::TrackingWheel vertical(&verticalEnc, 2, -0.5);


lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 14,
                             lemlib::Omniwheel::OLD_325, 333, 8);


// ControllerSettings(kP, kI, kD, windupRange, smallError, smallErrorTimeout,
//                    largeError, largeErrorTimeout, slew)
//
// Linear is unchanged and measured good: 6 reps of a 24 in drive gave
// worst 0.62 in, avg 0.38 in. Nothing to gain by touching it.
lemlib::ControllerSettings linearController(6.5, 0, 28, 3, 1, 100, 3, 500, 10);

// Angular: kP was 1 and kD was 0.05, which is effectively no damping and far
// too little authority. At kP 1 a 3 deg error commands an output of 3 out of
// 127, below what it takes to break stiction, so the turn stalls a few degrees
// short and exits on the 3 deg / 500 ms large-error window instead of finishing.
// Three degrees of heading is about 1.3 in of lateral miss over 24 in of travel,
// which is what the pin pickups were losing.
lemlib::ControllerSettings angularController(3.7, 0, 30, 3, 1, 100, 1.5, 500, 0);
//                                                            largeError ^^^  was 3
//                                           ^^^     ^^  were kP 1, kD 0.05
// kP history: 1 (original) -> 2.5 -> 4.0. At 2.5 it finished a consistent
// 2.2 deg short every rep (worst 2.54, avg 2.19): inside the 3 deg exit window
// but with only ~5/127 of output left, too little to break stiction and close it.
// kD history: 0.05 (original) -> 12 -> 18 -> 24. At kP 4.0 / kD 12 the turn
// carried past the target on momentum and pulled back (worst 1.88, avg 1.63);
// kD is the term that starts braking before the target instead of at it.
// Measured, 6 reps of a 90 deg turn each time, same start:
//   kP 2.5 / kD 12  worst 2.54  avg 2.19   stalls short, no authority left
//   kP 4.0 / kD 12  worst 1.88  avg 1.63   overshoots on momentum, pulls back
//   kP 4.0 / kD 18  worst 1.35  avg 0.91   less overshoot
//   kP 4.0 / kD 24  worst 1.25  avg 0.72   but shook on ~1 run in 5
//   kP 4.0 / kD 21  worst 0.66  avg 0.34   FINAL, and better than 24 on both
// 21 beat 24 outright: past the noise boundary the extra gain was fighting IMU
// noise, not momentum. Raising kD further trades overshoot for buzz.
// If turns ever feel too violent, limit acceleration with the slew field (last
// argument, currently 0) rather than adding more kD.


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

// Driver-control copy of the lift PID. Tune this one freely, autonomous keeps liftPID.
LiftPID driverLiftPID(1.6, 0.0, 0.16, 10, 20);

// Which PID the lift task is running. Autonomous = liftPID, driver = driverLiftPID.
LiftPID* activeLiftPID = &liftPID;


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
const double LIFT_MAX = 240.0;


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
   activeLiftPID = &liftPID;
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

           const double previousAngle = activeLiftPID->lastAngle;
           liftPIDNow = activeLiftPID->update(liftTarget, currentAngle);
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
// AI VISION: DRIVE BACKWARD ONTO A PIN (YELLOW COLOR TRACKING)
// =====================================================================
//
// The pin is found by color, not by the AI game model. The sensor boxes every
// blob that matches the yellow signature defined below, and the biggest blob
// near where the pin was last seen is treated as the pin.
//
// Mounting convention (all "left/right" below are as seen by someone standing
// BEHIND the robot, looking at its back):
//   * The sensor faces backward, so image RIGHT = right from behind.
//   * The sensor sits AIV_CAMERA_OFFSET_LEFT_IN to the left of the drivetrain
//     centerline, so the drivetrain centerline is that far to the RIGHT of the
//     image center. The pin therefore has to sit at image center PLUS the offset,
//     not at the exact image center.
//   * The offset is converted to pixels with the pin's apparent width, so it
//     stays correct as the pin gets closer and bigger:
//         pixelsPerInch = blobWidthPx / AIV_PIN_WIDTH_IN
//         targetX       = imageCenter + offsetIn * pixelsPerInch
//
// Steering: error = pinCenterX - targetX. Pin right of target (error > 0) means
// the back of the robot has to swing to the right-from-behind, which is the
// robot turning clockwise from above, which is a positive "turn" below.
// If it steers AWAY from the pin on blocks, flip AIV_STEER_SIGN to -1.

// ---- hardware / identification ----
const int    AIV_YELLOW_SIG_ID   = 1;      // color signature ID for yellow (1-7)
const bool   AIV_SET_COLOR_IN_CODE = true; // true = this program defines the yellow signature itself
                                           // false = use whatever the AI Vision utility saved
const bool   AIV_USE_GAME_MODEL  = true;   // true = game element AI model, false = color signature
const int    AIV_PIN_CLASS_ID    = 4;      // class ID of the pin in the game model (found with AUTON_MODE 1)
                                           // used when AIV_USE_GAME_MODEL is true

// ---- geometry ----
const double AIV_CAMERA_OFFSET_LEFT_IN = -6.9;  // camera is this far LEFT of drivetrain center, seen from behind
const double AIV_PIN_WIDTH_IN    = 3.0;    // TODO measure: real width of the yellow blob the sensor boxes
const double AIV_IMAGE_WIDTH_PX  = 320.0;  // AI Vision image is 320 x 240
const double AIV_IMAGE_CENTER_PX = AIV_IMAGE_WIDTH_PX / 2.0;

// ---- target selection ----
const double AIV_FIRST_WINDOW_PX = 120.0;  // first look: accept blobs this far from image center
const double AIV_TRACK_WINDOW_PX = 60.0;   // after that: stay within this of where it was last seen
const double AIV_MIN_WIDTH_PX    = 6.0;    // ignore specks narrower than this

// ---- driving ----
const double AIV_DRIVE_SPEED     = 60;     // move() units, always applied backward
const double AIV_KP              = 0.25;   // move() units per pixel of error
const double AIV_STEER_MAX       = 35;     // steering clamp
const double AIV_DEADBAND_PX     = 3.0;    // no steering inside this
const int    AIV_STEER_SIGN      = 1;      // flip to -1 if it steers away from the pin
const double AIV_KH              = 2.0;    // heading hold gain, move() units per degree

// ---- lost handling ----
const uint32_t AIV_LOST_CONFIRM_MS = 80;   // pin must be missing this long before it counts as out of view

enum class AivEnd { LOST_LIMIT, MAX_DISTANCE, TIMEOUT };

// ---- speed control ----------------------------------------------------
// aivDriveSpeed is the default backward speed (move() units, 0-127, always
// applied backward). aivSetDriveSpeed() changes it for every later call.
// driveBackToPin() also takes a speed for one call only.
double aivDriveSpeed = AIV_DRIVE_SPEED;
double aivSteerKp    = AIV_KP;
double aivSteerMax   = AIV_STEER_MAX;
const double AIV_DEFAULT_SPEED = -1;   // pass this (or leave the speed out) to use aivDriveSpeed

void aivSetDriveSpeed(double speed) {
    speed = std::fabs(speed);
    if (speed > 127) speed = 127;
    aivDriveSpeed = speed;
}
double aivGetDriveSpeed() { return aivDriveSpeed; }

// Steering strength. kp = move() units per pixel of error, maxSteer = clamp.
void aivSetSteering(double kp, double maxSteer) {
    aivSteerKp = std::fabs(kp);
    aivSteerMax = std::fabs(maxSteer);
}

// Call once from initialize().
void aivInit() {
    aivision.reset();
    // The class id finder needs the AI model objects on, whatever AIV_USE_GAME_MODEL says.
    aivision.enable_detection_types((AIV_USE_GAME_MODEL || AUTON_MODE == 1)
                                        ? pros::AivisionModeType::objects
                                        : pros::AivisionModeType::colors);
    if (!AIV_USE_GAME_MODEL && AIV_SET_COLOR_IN_CODE) {
        // Approximate yellow. Tune hue/saturation range, or set AIV_SET_COLOR_IN_CODE false
        // and use a signature saved from the AI Vision utility instead.
        pros::AIVision::Color yellow = {.id = (uint8_t)AIV_YELLOW_SIG_ID,
                                        .red = 240, .green = 200, .blue = 20,
                                        .hue_range = 20, .saturation_range = 0.5};
        aivision.set_color(yellow);
    }
}

// ---- class id finder -------------------------------------------------
// aivScanClasses() fills aivSeen with the (up to) 3 biggest AI model objects the
// sensor sees right now, biggest first. aivSeen[0].id is the one to use for
// AIV_PIN_CLASS_ID when a pin is the biggest thing in view.
const int AIV_SEEN_MAX = 3;
struct AivSeen { int id; int x; int w; int h; int score; };
AivSeen aivSeen[AIV_SEEN_MAX];
int aivSeenCount = 0;

void aivScanClasses() {
    auto objects = aivision.get_all_objects();
    AivSeen found[AIV_SEEN_MAX];
    int n = 0;

    for (auto& o : objects) {
        if (!pros::AIVision::is_type(o, pros::AivisionDetectType::object)) continue;

        AivSeen s;
        s.id    = (int)o.id;
        s.x     = (int)o.object.element.xoffset;
        s.w     = (int)o.object.element.width;
        s.h     = (int)o.object.element.height;
        s.score = (int)o.object.element.score;

        // insert sorted by box area, biggest first, keep only AIV_SEEN_MAX
        const int area = s.w * s.h;
        int pos = n;
        while (pos > 0 && found[pos - 1].w * found[pos - 1].h < area) pos--;
        if (pos >= AIV_SEEN_MAX) continue;

        const int last = (n < AIV_SEEN_MAX) ? n : AIV_SEEN_MAX - 1;
        for (int i = last; i > pos; i--) found[i] = found[i - 1];
        found[pos] = s;
        if (n < AIV_SEEN_MAX) n++;
    }

    for (int i = 0; i < n; i++) aivSeen[i] = found[i];
    aivSeenCount = n;
}

// Prints every detected object's class ID over the terminal. Not called automatically.
void aivPrintClasses() {
    auto objects = aivision.get_all_objects();
    for (auto& o : objects) {
        if (pros::AIVision::is_type(o, pros::AivisionDetectType::object)) {
            printf("class id %d  x %d w %d h %d\n", (int)o.id,
                   (int)o.object.element.xoffset, (int)o.object.element.width,
                   (int)o.object.element.height);
        }
    }
}

// Finds the biggest yellow pin blob within windowPx of lastX. Returns its center x
// and width in pixels.
static bool aivFindPin(double lastX, double windowPx, double& centerX, double& widthPx) {
    auto objects = aivision.get_all_objects();
    double bestArea = 0;
    bool found = false;

    for (auto& o : objects) {
        double x, w, h;

        if (AIV_USE_GAME_MODEL) {
            if (!pros::AIVision::is_type(o, pros::AivisionDetectType::object)) continue;
            if ((int)o.id != AIV_PIN_CLASS_ID) continue;
            x = o.object.element.xoffset;
            w = o.object.element.width;
            h = o.object.element.height;
        } else {
            if (!pros::AIVision::is_type(o, pros::AivisionDetectType::color)) continue;
            if ((int)o.id != AIV_YELLOW_SIG_ID) continue;
            x = o.object.color.xoffset;
            w = o.object.color.width;
            h = o.object.color.height;
        }

        if (w < AIV_MIN_WIDTH_PX) continue;

        const double cx = x + w / 2.0;
        if (std::fabs(cx - lastX) > windowPx) continue;

        const double area = w * h;
        if (area > bestArea) {
            bestArea = area;
            centerX = cx;
            widthPx = w;
            found = true;
        }
    }
    return found;
}

static double aivClamp(double v, double lim) {
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

// =====================================================================
// AI VISION: ROTATE UNTIL A PIN IS FOUND
// =====================================================================

// Returns true if a pin was found.
// Returns false if the search timed out or completed a full rotation.
//
// turnSpeed:      Rotation speed (1-60)
// maxRotationDeg: Maximum amount to rotate
// timeoutMs:      Maximum search time
// clockwise:      true = clockwise, false = counterclockwise
//
// The robot stops as soon as the pin is detected consistently.

bool rotateUntilPinFound(double turnSpeed = 25.0,
                         double maxRotationDeg = 360.0,
                         int timeoutMs = 12000,
                         bool clockwise = true) {

    // Limit rotation speed
    turnSpeed = std::fabs(turnSpeed);

    if (turnSpeed < 1.0) turnSpeed = 1.0;
    if (turnSpeed > 60.0) turnSpeed = 60.0;

    if (maxRotationDeg <= 0) maxRotationDeg = 360.0;
    if (timeoutMs <= 0) timeoutMs = 12000;

    // Stop any existing LemLib movement
    chassis.cancelAllMotions();

    // Record starting rotation and time
    const double startRotation = imu.get_rotation();
    const uint32_t startTime = pros::millis();

    // Require multiple consecutive detections
    int consecutiveFrames = 0;
    const int REQUIRED_FRAMES = 3;

    // Rotation direction
    const double direction = clockwise ? 1.0 : -1.0;

    while (true) {

        // -------------------------------------------------------------
        // 1. SEARCH FOR PIN
        // -------------------------------------------------------------

        double centerX = 0;
        double widthPx = 0;

        bool pinDetected = aivFindPin(
            AIV_IMAGE_CENTER_PX,
            AIV_IMAGE_WIDTH_PX,
            centerX,
            widthPx
        );

        if (pinDetected) {
            consecutiveFrames++;
        } else {
            consecutiveFrames = 0;
        }

        // -------------------------------------------------------------
        // 2. CONFIRM DETECTION
        // -------------------------------------------------------------

        if (consecutiveFrames >= REQUIRED_FRAMES) {

            leftMotors.brake();
            rightMotors.brake();

            return true;
        }

        // -------------------------------------------------------------
        // 3. CHECK TIMEOUT
        // -------------------------------------------------------------

        uint32_t now = pros::millis();

        if (now - startTime >= (uint32_t)timeoutMs) {
            break;
        }

        // -------------------------------------------------------------
        // 4. CHECK ROTATION LIMIT
        // -------------------------------------------------------------

        double rotated = std::fabs(
            imu.get_rotation() - startRotation
        );

        if (rotated >= maxRotationDeg) {
            break;
        }

        // -------------------------------------------------------------
        // 5. ROTATE ROBOT
        // -------------------------------------------------------------

        // Positive direction = clockwise with your current drivetrain setup.

        leftMotors.move(direction * turnSpeed);
        rightMotors.move(-direction * turnSpeed);

        pros::delay(20);
    }

    // -------------------------------------------------------------
    // 6. PIN NOT FOUND
    // -------------------------------------------------------------

    leftMotors.brake();
    rightMotors.brake();

    return false;
}

// speed: move() units 0-127, applied backward. Leave it out (or pass
// AIV_DEFAULT_SPEED) to use the default set by aivSetDriveSpeed().
//
// Drives BACKWARD only, steering with the camera so the pin lines up with the
// drivetrain centerline. Ends when whichever comes first:
//   * the pin has been out of view for lostDistanceIn inches or lostTimeoutMs
//     (the heading is locked on the IMU the moment it leaves view)
//   * maxDistanceIn inches have been driven
//   * timeoutMs has passed
// Only the drive motors are stopped. The claw, arm and lift keep doing whatever
// they were already doing.
//
// If the pin is never found, it counts as out of view from the start: heading is
// locked immediately and it stops after lostDistanceIn / lostTimeoutMs.
AivEnd driveBackToPin(double maxDistanceIn, int timeoutMs,
                      double speed = AIV_DEFAULT_SPEED,
                      double lostDistanceIn = 6.0, int lostTimeoutMs = 400) {
    chassis.cancelAllMotions();

    double driveSpeed = (speed < 0) ? aivDriveSpeed : std::fabs(speed);
    if (driveSpeed > 127) driveSpeed = 127;

    const uint32_t start = pros::millis();
    const lemlib::Pose startPose = chassis.getPose();

    double lastX = AIV_IMAGE_CENTER_PX;
    double window = AIV_FIRST_WINDOW_PX;
    uint32_t lastSeen = start;

    bool locked = false;
    double lockHeading = 0;
    uint32_t lockStart = 0;
    lemlib::Pose lockPose = startPose;

    double turn = 0;
    AivEnd result = AivEnd::TIMEOUT;

    while (true) {
        const uint32_t now = pros::millis();

        if ((int)(now - start) >= timeoutMs) { result = AivEnd::TIMEOUT; break; }

        const lemlib::Pose pose = chassis.getPose();
        const double traveled = std::hypot(pose.x - startPose.x, pose.y - startPose.y);
        if (traveled >= maxDistanceIn) { result = AivEnd::MAX_DISTANCE; break; }

        if (!locked) {
            double cx = 0, wpx = 0;
            if (aivFindPin(lastX, window, cx, wpx)) {
                lastSeen = now;
                lastX = cx;
                window = AIV_TRACK_WINDOW_PX;

                const double pxPerIn = wpx / AIV_PIN_WIDTH_IN;
                const double targetX = AIV_IMAGE_CENTER_PX + AIV_CAMERA_OFFSET_LEFT_IN * pxPerIn;
                const double error = cx - targetX;

                if (std::fabs(error) < AIV_DEADBAND_PX) turn = 0;
                else turn = aivClamp(AIV_STEER_SIGN * aivSteerKp * error, aivSteerMax);
            }
            else if (now - lastSeen >= AIV_LOST_CONFIRM_MS) {
                // Out of view. Lock the heading here and ride it out.
                locked = true;
                lockHeading = imu.get_rotation();
                lockStart = now;
                lockPose = pose;
            }
            // else: a frame or two missing, keep the last steering
        }

        if (locked) {
            const double sinceLock = std::hypot(pose.x - lockPose.x, pose.y - lockPose.y);
            if (sinceLock >= lostDistanceIn || (int)(now - lockStart) >= lostTimeoutMs) {
                result = AivEnd::LOST_LIMIT;
                break;
            }
            // IMU rotation increases clockwise, and positive turn is clockwise.
            turn = aivClamp(AIV_KH * (lockHeading - imu.get_rotation()), aivSteerMax);
        }

        // Backward only. Steering can slow one side but never drives it forward.
        double l = -driveSpeed + turn;
        double r = -driveSpeed - turn;
        if (l > 0) l = 0;
        if (r > 0) r = 0;
        leftMotors.move(l);
        rightMotors.move(r);

        pros::delay(10);
    }

    leftMotors.brake();
    rightMotors.brake();
    return result;
}

// =====================================================================
// AI VISION: TRACK A PIN IN PLACE (PURE ROTATION, BEZIER TURN SPEED)
// =====================================================================
//
// One update step, non-blocking. Call it repeatedly from a loop. It never drives
// forward or backward: both sides always get equal and opposite commands, so the
// only motion possible is rotation about the robot's own centre.
//
// Only used by AUTON_MODE 2. Nothing in normal autonomous or driver control
// calls it, and it touches only the drive motors.
//
// THE MATHS (aim for the middle of the ROBOT, not the middle of the camera)
//
//   The camera sits AIV_CAMERA_OFFSET_LEFT_IN to one side of the turning centre
//   and AIV_CAMERA_BACK_IN behind it. From one frame:
//       pxPerIn = blobWidthPx / AIV_PIN_WIDTH_IN        (inches to pixels at the pin)
//       lens-to-pin distance   = AIV_FOCAL_PX / pxPerIn
//       x = (blobCentreX - imageCentre) / pxPerIn - offset   sideways from the robot's centreline, in
//       z = lens-to-pin distance + AIV_CAMERA_BACK_IN         straight back from the turning centre, in
//       angle to turn = atan2(x, z)
//   That is the turn that puts the pin on the robot's centreline. The sideways
//   shift is only ignored when the pin is very far away.
//
// LAST SEEN
//
//   Every good frame is turned into an ABSOLUTE heading (IMU) that the robot has to
//   face for the pin to be on its centreline. The robot steers to that heading
//   whether or not the camera still sees the pin. So when the pin drops out, or
//   swings out of view while the robot turns, the robot carries on turning toward
//   where it was last seen, instead of stopping, and the camera picks it up again.
//   Memory is forgotten after TRACK_MEMORY_MS, and runs slower than live tracking.
//
// PIN CUT OFF AT THE EDGE OF THE IMAGE
//
//   When a pin comes in from the side the box is cut off by the image edge, so its
//   width is too small. A tiny width reads as "very far away", which makes the
//   sideways offset vanish and the robot aims the CAMERA at it, which can send it
//   the wrong way. A cut-off box now borrows the last full-width reading, and its
//   centre is rebuilt from the edge that is still in view. With no full-width
//   reading yet it just turns toward the pin to bring it fully into view.
//
// SMOOTHNESS
//
//   * The camera frame is ~100 ms old when read. The robot's IMU heading is
//     logged, so the frame's angle is taken against the heading at the moment it
//     was captured (TRACK_VISION_LATENCY_MS).
//   * Turn speed is a Bezier curve on the angle error: eases in away from the
//     target, eases out as it closes in. Inside TRACK_DEADBAND_DEG it stops.
//   * Speeding up is gentle (TRACK_ACCEL), slowing down is fast (TRACK_DECEL).
//   * Counter-clockwise turns are faster (TRACK_CCW_SCALE) because the camera is
//     on the left and its view swings away from the pin turning that way.
//
// Returns true while the camera can see a pin, false when it cannot (the robot may
// still be turning toward the last seen position, see aivTrackUsingMemory).

// ---- geometry for the angle calculation ----
// Horizontal field of view of the AI Vision sensor, in degrees. This is an
// ESTIMATE, check it. It sets the pixels-to-degrees scale for pins that are cut
// off at the edge. It does not change where the robot stops.
const double AIV_HFOV_DEG       = 74.0;
const double AIV_FOCAL_PX       = (AIV_IMAGE_WIDTH_PX / 2.0) /
                                  std::tan(AIV_HFOV_DEG * 3.14159265358979 / 360.0);
// TODO measure: how far BEHIND the robot's turning centre the camera lens sits,
// in inches. Only changes how hard it turns for a given pixel error (the stop
// point is where the pin is on the centreline, whatever this is set to).
const double AIV_CAMERA_BACK_IN = 7.0;

// ---- tuning ----
const double TRACK_DEADBAND_DEG  = 2.0;    // inside this the robot stops turning (bigger = looser, smoother)
const double TRACK_FULL_ERR_DEG  = 35.0;   // error at which the turn reaches TRACK_MAX_TURN
const double TRACK_MIN_TURN      = 10.0;   // move() units just outside the deadband, enough to actually move
const double TRACK_MAX_TURN      = 45.0;   // move() units at full error
const double TRACK_BEZ_C1        = 0.10;   // Bezier shape: both near 0 / 1 give a gentle S-curve.
const double TRACK_BEZ_C2        = 0.90;   //   Raise C1 or lower C2 for a snappier response.
const double TRACK_ACCEL         = 300.0;  // max SPEED-UP of the turn command per second (3 per 10 ms loop)
const double TRACK_DECEL         = 1500.0; // max SLOW-DOWN per second (15 per 10 ms loop), keep this high
// The camera is on the LEFT of the drivetrain facing backward, so turning
// COUNTER-clockwise swings its view away and it loses the pin unless the robot
// moves faster. Every counter-clockwise turn (speed and speed-up rate) is
// multiplied by this. 1.0 = same as clockwise. Raise it if it still loses the pin
// turning that way, lower it if it overshoots that way.
const double TRACK_CCW_SCALE     = 2.5;
// How old a camera frame is by the time it is read. If it still overshoots,
// raise this. If it stops short or feels sluggish, lower it.
const double TRACK_VISION_LATENCY_MS = 100.0;
// Last seen: how long to keep turning toward the remembered spot, and the speed
// cap while doing it (it is less certain than a live reading).
const double   TRACK_MEMORY_MS       = 2500.0;
const double   TRACK_MEMORY_MAX_TURN = 30.0;
// A box edge this close to the image edge counts as cut off.
const double   TRACK_EDGE_PX         = 3.0;

// telemetry, so a caller can report what the tracker is doing
double aivTrackError   = 0;                    // degrees, angle from the centreline to the pin, right of it is positive
double aivTrackTargetX = AIV_IMAGE_CENTER_PX;  // image column the centreline currently sits at (display only)
double aivTrackTurn    = 0;                    // move() units currently applied (after smoothing)
bool   aivTrackHasPin  = false;                // the camera sees a pin right now
bool   aivTrackUsingMemory = false;            // pin not visible, turning toward where it was last seen

static double   aivTrackLastX    = AIV_IMAGE_CENTER_PX;
static double   aivTrackDesired  = 0;     // turn the Bezier asks for, before the slew limit
static uint32_t aivTrackLastSeen = 0;
static uint32_t aivTrackLastCall = 0;
static bool     aivTrackPrimed   = false;

// last raw camera frame, so a repeated frame is not mistaken for a new one
static bool     aivFrameValid = false;
static double   aivFrameCx    = 0;
static double   aivFrameW     = 0;
static double   aivFrameErr   = 0;   // angle from that frame, degrees (used only if the IMU is unavailable)

// last seen: the absolute IMU heading at which the pin is on the centreline
static bool     aivTargetValid   = false;
static double   aivTargetHeading = 0;

// last full-width (not cut off) blob width, smoothed
static bool     aivGoodWValid = false;
static double   aivGoodW      = 0;

// short log of IMU heading, so the heading at the moment a frame was captured
// can be looked up
static const int TRACK_HIST_N = 64;   // 64 calls is about 640 ms at a 10 ms loop
static double aivHistHeading[TRACK_HIST_N];
static double aivHistTime[TRACK_HIST_N];
static int    aivHistNext  = 0;
static int    aivHistCount = 0;

static void aivHistPush(uint32_t t, double heading) {
    aivHistTime[aivHistNext]    = (double)t;
    aivHistHeading[aivHistNext] = heading;
    aivHistNext = (aivHistNext + 1) % TRACK_HIST_N;
    if (aivHistCount < TRACK_HIST_N) aivHistCount++;
}

// Heading logged closest to time t (ms). Falls back to `fallback` if nothing is logged.
static double aivHeadingAt(double t, double fallback) {
    double best = fallback;
    double bestDiff = 1e18;
    for (int i = 0; i < aivHistCount; i++) {
        const double d = std::fabs(aivHistTime[i] - t);
        if (d < bestDiff) { bestDiff = d; best = aivHistHeading[i]; }
    }
    return best;
}

// Cubic Bezier from 0 to 1 with control values c1 and c2. t in 0..1.
static double aivBezierEase(double t) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    const double u = 1.0 - t;
    return 3.0 * u * u * t * TRACK_BEZ_C1 + 3.0 * u * t * t * TRACK_BEZ_C2 + t * t * t;
}

// Call once before a tracking loop so it does not start on stale state.
void trackPinInPlaceReset() {
    aivTrackLastX    = AIV_IMAGE_CENTER_PX;
    aivTrackDesired  = 0;
    aivTrackLastSeen = pros::millis();
    aivTrackLastCall = pros::millis();
    aivTrackTurn     = 0;
    aivTrackError    = 0;
    aivTrackTargetX  = AIV_IMAGE_CENTER_PX;
    aivTrackHasPin   = false;
    aivTrackUsingMemory = false;
    aivFrameValid    = false;
    aivTargetValid   = false;
    aivGoodWValid    = false;
    aivHistNext      = 0;
    aivHistCount     = 0;
    aivTrackPrimed   = true;
}

bool trackPinInPlace() {
    if (!aivTrackPrimed) trackPinInPlaceReset();

    const uint32_t now = pros::millis();
    double dtMs = (double)(now - aivTrackLastCall);
    aivTrackLastCall = now;
    if (dtMs < 1.0) dtMs = 1.0;
    if (dtMs > 50.0) dtMs = 50.0;   // a long gap must not allow a big jump

    // IMU heading, clockwise positive. Logged every call for the latency correction.
    const double heading = imu.get_rotation();
    const bool imuOK = std::isfinite(heading);
    if (imuOK) aivHistPush(now, heading);

    double cx = 0, wpx = 0;
    // Whole image is the search window: a pin anywhere on screen is accepted.
    if (aivFindPin(aivTrackLastX, AIV_IMAGE_WIDTH_PX, cx, wpx)) {
        aivTrackLastSeen = now;
        aivTrackLastX    = cx;
        aivTrackHasPin   = true;

        // Only recompute on a NEW frame. A repeated frame changes nothing.
        if (!aivFrameValid || cx != aivFrameCx || wpx != aivFrameW) {
            aivFrameCx    = cx;
            aivFrameW     = wpx;
            aivFrameValid = true;

            // Is the box cut off by the edge of the image?
            const double left  = cx - wpx / 2.0;
            const double right = cx + wpx / 2.0;
            const bool clipL = left  <= TRACK_EDGE_PX;
            const bool clipR = right >= AIV_IMAGE_WIDTH_PX - TRACK_EDGE_PX;

            double cxUse = cx;
            double wUse  = wpx;
            bool   trust = true;

            if (clipL != clipR) {
                // Exactly one side cut off: the width is too small and the centre
                // is pulled inward. Rebuild both from the side still in view.
                if (aivGoodWValid) {
                    wUse  = aivGoodW;
                    cxUse = clipL ? right - wUse / 2.0 : left + wUse / 2.0;
                } else {
                    trust = false;   // nothing to borrow from yet
                }
            }
            else if (!clipL && !clipR) {
                // Full box: remember its width (smoothed) for later cut-off frames.
                aivGoodW = aivGoodWValid ? 0.7 * aivGoodW + 0.3 * wpx : wpx;
                aivGoodWValid = true;
            }
            // (both sides cut off: pin wider than the view, use it as it is)

            double angleDeg;
            if (trust) {
                const double pxPerIn = wUse / AIV_PIN_WIDTH_IN;
                aivTrackTargetX = AIV_IMAGE_CENTER_PX + AIV_CAMERA_OFFSET_LEFT_IN * pxPerIn;

                // Sideways from the robot's centreline (inches) and straight back
                // from its turning centre (inches), then the angle between them.
                const double xRelIn = (cxUse - AIV_IMAGE_CENTER_PX) / pxPerIn
                                      - AIV_CAMERA_OFFSET_LEFT_IN;
                const double zRelIn = AIV_FOCAL_PX / pxPerIn + AIV_CAMERA_BACK_IN;
                angleDeg = std::atan2(xRelIn, zRelIn) * 180.0 / 3.14159265358979;
            } else {
                // Cut off and no good width yet. Turn toward it to bring it into
                // view, a later full frame does the precise aim.
                angleDeg = std::atan2(cx - AIV_IMAGE_CENTER_PX, AIV_FOCAL_PX)
                           * 180.0 / 3.14159265358979;
            }
            aivFrameErr = angleDeg;

            // Turn the angle into an absolute heading, using the heading the robot
            // had when the frame was captured, so the age of the frame does not matter.
            if (imuOK) {
                const double headingThen =
                    aivHeadingAt((double)now - TRACK_VISION_LATENCY_MS, heading);
                aivTargetHeading = headingThen + AIV_STEER_SIGN * angleDeg;
                aivTargetValid   = true;
            }
        }
    }
    else {
        if (now - aivTrackLastSeen >= AIV_LOST_CONFIRM_MS) {
            // The camera has really lost it. The remembered heading is kept.
            aivTrackHasPin = false;
            aivTrackLastX  = AIV_IMAGE_CENTER_PX;
        }
        if ((double)(now - aivTrackLastSeen) >= TRACK_MEMORY_MS) {
            // Too long ago to trust. Forget it all.
            aivTargetValid = false;
            aivGoodWValid  = false;
            aivFrameValid  = false;
        }
    }

    // Error from the remembered heading, which also covers dropouts.
    aivTrackUsingMemory = false;
    bool haveErr = false;
    double err = 0;
    if (imuOK && aivTargetValid) {
        err = AIV_STEER_SIGN * (aivTargetHeading - heading);
        haveErr = true;
        aivTrackUsingMemory = !aivTrackHasPin;
    }
    else if (!imuOK && aivTrackHasPin && aivFrameValid) {
        err = aivFrameErr;   // no IMU: use the raw frame angle
        haveErr = true;
    }

    if (haveErr) {
        const double absErr = std::fabs(err);
        aivTrackError = err;

        if (absErr <= TRACK_DEADBAND_DEG) {
            aivTrackDesired = 0;
        }
        else {
            // 0 at the edge of the deadband, 1 at TRACK_FULL_ERR_DEG.
            const double t = (absErr - TRACK_DEADBAND_DEG) /
                             (TRACK_FULL_ERR_DEG - TRACK_DEADBAND_DEG);
            double mag = TRACK_MIN_TURN +
                         (TRACK_MAX_TURN - TRACK_MIN_TURN) * aivBezierEase(t);
            if (aivTrackUsingMemory && mag > TRACK_MEMORY_MAX_TURN) mag = TRACK_MEMORY_MAX_TURN;

            // Sign of the motor command: positive is clockwise, negative is
            // counter-clockwise. Counter-clockwise gets the extra speed.
            const double dir = AIV_STEER_SIGN * (err > 0 ? 1.0 : -1.0);
            if (dir < 0) mag *= TRACK_CCW_SCALE;
            if (mag > 127.0) mag = 127.0;
            aivTrackDesired = dir * mag;
        }
    }
    else {
        aivTrackDesired = 0;
    }

    // Slew limit: gentle speeding up, fast slowing down. Slowing down means the
    // command is getting smaller in size, or is flipping sign.
    const bool slowing = (std::fabs(aivTrackDesired) < std::fabs(aivTrackTurn)) ||
                         (aivTrackDesired * aivTrackTurn < 0);
    const double accel = (aivTrackDesired < 0) ? TRACK_ACCEL * TRACK_CCW_SCALE : TRACK_ACCEL;
    const double maxStep = (slowing ? TRACK_DECEL : accel) * dtMs / 1000.0;
    double diff = aivTrackDesired - aivTrackTurn;
    if (diff >  maxStep) diff =  maxStep;
    if (diff < -maxStep) diff = -maxStep;
    aivTrackTurn += diff;

    // Pure rotation: equal and opposite, so there is no forward component.
    leftMotors.move(aivTrackTurn);
    rightMotors.move(-aivTrackTurn);

    return aivTrackHasPin;
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
// Must stay == 1, not a bare truthiness test: AUTON_MODE 2 is the tracking demo
// and wants the normal pose/lift layout, not the finder's.
#if AUTON_MODE == 1
       // Class id finder: lines 0-3 show what the AI Vision sensor sees.
       pros::lcd::set_text(0, fmt::format("AIV objects: {}  (finder mode)", aivSeenCount));
       for (int i = 0; i < AIV_SEEN_MAX; i++) {
           if (i < aivSeenCount) {
               const AivSeen s = aivSeen[i];
               pros::lcd::set_text(1 + i, fmt::format("class id {}  x {} w {} score {}",
                                                      s.id, s.x, s.w, s.score));
           } else {
               pros::lcd::set_text(1 + i, "");
           }
       }
#else
       pros::lcd::set_text(0, fmt::format("X: {:.2f} in", pose.x));
       pros::lcd::set_text(1, fmt::format("Y: {:.2f} in", pose.y));
       pros::lcd::set_text(2, fmt::format("Target Angle: {:.1f} deg", liftTarget));
       pros::lcd::set_text(3, fmt::format("Motor Command: {:.1f}", liftLastOutput));
#endif
       pros::lcd::set_text(4, fmt::format("PIDon: {}  Arm: {:.0f}",
                                       liftPIDEnabled ? 1 : 0, arm.get_position()));
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

   initStage = "aivision";
   aivInit();

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
const int32_t ARM_HOLD_VOLTAGE = 3500;

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
    liftLeft.set_current_limit(750); //300
    liftRight.set_current_limit(750); //300

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
#if AUTON_MODE == 1

   // YELLOW FINDER. Nothing on the robot moves. Point the rear AI Vision sensor
   // at a pin and watch it from any of:
   //   * brain screen lines 0-4 (biggest yellow blob first, line 4 is what
   //     driveBackToPin() would lock onto)
   //   * controller line 0: "Yel n<count> w<width>"
   //   * the PROS terminal (pros terminal)
   // The pin should be boxed steadily. If it flickers or is missed, tune the
   // AIV_YELLOW_* numbers. If other yellow things get boxed, narrow them.
   // Then set AUTON_MODE back to 0.
   uint32_t lastReport = 0;
   while (true) {
       aivScanClasses();

       // Same lookup driveBackToPin() uses, wide open, so screen line 4 shows
       // whether the drive code would find the pin and where.
       double pcx = 0, pw = 0;
       aivFindPin(AIV_IMAGE_CENTER_PX, 1000.0, pcx, pw);

       const uint32_t now = pros::millis();
       if (now - lastReport >= 300) {
           lastReport = now;

           if (aivSeenCount == 0) {
               printf("AIV: no yellow detected\n");
               controller.print(0, 0, "Yel: none      ");
           } else {
               for (int i = 0; i < aivSeenCount; i++) {
                   printf("AIV yellow  x %d w %d h %d\n",
                          aivSeen[i].x, aivSeen[i].w, aivSeen[i].h);
               }
               controller.print(0, 0, "Yel n%d w%d    ", aivSeenCount, aivSeen[0].w);
           }
       }

       pros::delay(20);
   }

#elif AUTON_MODE == 2

   // =================================================================
   // PIN TRACKING DEMO v2 - YELLOW COLOUR DETECTION, SYMMETRIC TURN
   // =================================================================
   //
   // Pure rotation: both sides always get equal and opposite commands, so the
   // only motion possible is turning about the robot's own centre.
   //
   // WHY v1 WORKED ONE WAY ONLY (v1 is kept, commented out, at the bottom of
   // this branch):
   //   v1 worked out the real-world angle to the pin from its apparent width,
   //   including a sideways correction for the camera being off-centre:
   //       xRelIn = (blobCentre - imageCentre) / pxPerIn - AIV_CAMERA_OFFSET_LEFT_IN
   //   AIV_CAMERA_OFFSET_LEFT_IN is -6.75, so that subtraction ADDS a constant
   //   +6.75 inches no matter where the pin is. That is a fixed turn bias always
   //   in the SAME rotational direction. On one side it adds to the real error
   //   (it turned, just not well); on the other side it cancels the real error,
   //   so anything within roughly 10-20 degrees that way produced about zero
   //   command and the robot sat still. Exactly the behaviour seen on the field.
   //   The size of that bias is set by AIV_PIN_WIDTH_IN, which is still an
   //   unmeasured guess, and TRACK_CCW_SCALE = 2.5 then made one direction
   //   2.5x faster on top of it.
   //
   // WHAT v2 DOES INSTEAD
   //   No distance maths, no field-of-view constant, no IMU heading memory, no
   //   measured pin width. Just: steer so the yellow blob sits at the middle of
   //   the image. One gain, one clamp, one slew rate, used identically for both
   //   directions, so it cannot be lopsided.
   //
   //   Trade-off, on purpose: this aims the CAMERA at the pin, not the middle of
   //   the drivetrain. Because the camera is mounted off to one side, the robot
   //   ends up pointing a few degrees off the pin. For "it follows the pin" in
   //   front of judges that is invisible. If you want it to aim the chassis
   //   centreline instead, put a fixed pixel offset in TRK_AIM_OFFSET_PX and
   //   tune it by eye - a constant cannot introduce a direction-dependent bias
   //   the way the width-based maths did.
   //
   // DETECTION
   //   Colour signature, not the game model, with a deliberately loose hue and
   //   saturation window so the pin is picked up easily under whatever lighting
   //   the venue has. Glare and shading often break one pin into several boxes,
   //   so the biggest box is found first and any yellow box near it is merged
   //   into the same bounding box. Without that the aim jumps between pieces.
   //
   // CONTROLLER LINE 0
   //   "Pin x<col> w<width>"  tracking; x is the blob centre column, 160 = centred
   //   "Pin: sweep <-1|1>"    lost, creeping the way it last went to find it again
   //   "Pin: none"            nothing yellow in view, stopped
   //
   // IF IT TURNS THE WRONG WAY: set TRK_SIGN to -1. That is the only sign in
   // here, and it flips both directions together.

   // ---- tuning (all of it, mode 2 only, nothing outside this branch) ----
   const int    TRK_SIG_ID        = AIV_YELLOW_SIG_ID;  // colour slot this demo programs
   const double TRK_MIN_W_PX      = 5.0;    // ignore yellow specks narrower than this
   const double TRK_MERGE_FACTOR  = 1.5;    // merge yellow boxes within this x the winner's width
   const double TRK_AIM_OFFSET_PX = 0.0;    // + moves the aim point right of image centre
   const int    TRK_SIGN          = 1;      // -1 if it turns away from the pin

   // Smoothing. The blob centre is low-pass filtered, and only when the camera
   // actually delivers a new frame (it runs ~30 fps while this loop runs at 100 Hz,
   // so the same frame is read about three times and must not count as three).
   const double TRK_FILTER        = 0.50;   // 0..1 per new frame. Lower = smoother but laggier.
   const int    TRK_ACQUIRE_FRAMES = 2;     // good frames needed before it will move at all

   // Hysteresis instead of one deadband: it starts turning at ENGAGE and does not
   // stop until the error falls to RELEASE. With a single threshold the command
   // chatters 0 / MIN_TURN / 0 while sitting on it.
   const double TRK_ENGAGE_PX     = 25.0;   // start turning when error exceeds this
   const double TRK_RELEASE_PX    = 12.0;   // keep turning until error drops below this
   // After it releases, stay stopped for this long and refuse to re-engage. This is
   // what stops it hunting: the robot needs time to actually come to rest, and the
   // camera needs time to deliver frames taken from a STOPPED robot. Without it the
   // loop re-engages on error measured while still coasting, overshoots the other
   // way, and never settles on a stationary pin.
   const uint32_t TRK_SETTLE_MS   = 400;

   // SPEED. History, so none of this gets re-broken:
   //   * kP 0.55 / max 80  -> far too fast, oscillated hard.
   //   * kP 0.07 / max 20 / MIN_TURN 7 / RELEASE 10 -> barely twitched and stopped.
   //     That was NOT lack of torque: the wheels did turn at 7, so stiction is
   //     below that. It stopped because RELEASE 10 px is only ~2.7 degrees, so the
   //     latch let go almost immediately.
   //   * kP 0.30 / max 50 / MIN_TURN 18 -> moved well but hunted around a
   //     stationary pin, because MIN_TURN 18 is a big enough command to coast past
   //     the pin during the camera's ~100 ms delay, then correct back for ever.
   // Hence: a LOW minimum command (little overshoot), a modest ceiling, and the
   // settle dwell above to break the hunt. Smoothness comes from the dwell and the
   // blink handling, not from starving the gains.
   //   Hunting on a still pin  -> raise TRK_SETTLE_MS, then TRK_RELEASE_PX.
   //   Barely moves / twitches -> raise TRK_MIN_TURN a little, and check that
   //                              TRK_RELEASE_PX is not so wide it quits early.
   //   Too slow to catch up    -> raise TRK_MAX_TURN first, TRK_KP only a little.
   // With ~100 ms of camera delay, too much TRK_KP always ends in a fast limit
   // cycle however smooth the slew limit is: that is the trap, not low speed.
   const double TRK_KP            = 0.16;   // move() units per pixel of error
   const double TRK_MIN_TURN      = 9.0;    // floor once engaged. Must beat stiction, but every unit
                                            // above it is overshoot on a stationary pin. The wheels
                                            // were seen moving at 7, so stiction is below that.
   const double TRK_MAX_TURN      = 25.0;   // clamp, same both directions
   const double TRK_SLEW_UP       = 2.5;    // max SPEED-UP per 10 ms loop (gentle starts)
   const double TRK_SLEW_DOWN     = 5.0;    // max SLOW-DOWN per 10 ms loop (quick to back off)

   // ---- what to do when the pin is not detected ----
   // A loose colour signature flickers, so a dropout is usually the detector
   // blinking, not the pin leaving. Reacting to a blink is what made it jitter:
   // every flicker past the old 150 ms threshold fired a sweep pulse.
   // So: coast down to a stop first, and only sweep after the pin has been gone a
   // long time AND was last seen near the edge of the image, i.e. it really did
   // leave the view. A pin that vanishes mid-image now produces no motion at all.
   const uint32_t TRK_HOLD_MS     = 500;    // blind period before it considers the pin gone
   const double TRK_HOLD_DECAY    = 0.98;   // command multiplier per loop while blind
   const double TRK_EDGE_PX       = 45.0;   // last seen this close to an edge = it left the view
   const uint32_t TRK_SWEEP_MS    = 1500;   // how long to creep after a pin that really left
   const double TRK_SWEEP_TURN    = 18.0;   // creep speed while searching

   chassis.cancelAllMotions();

   // Nothing here drives the lift. Leave it braked where it is rather than limp.
   liftAutonomousMode = false;
   liftPIDEnabled = false;
   liftLeft.brake();
   liftRight.brake();

   // aivInit() turned ON the AI game model, because AIV_USE_GAME_MODEL is true.
   // This demo wants colour blobs, so it switches the sensor over here. Only
   // AUTON_MODE 2 ever runs this, so the game model is left alone everywhere else.
   aivision.enable_detection_types(pros::AivisionModeType::colors);
   {
       // Loose on purpose. hue_range and saturation_range are the "margin of
       // error": widen them if the pin is missed, narrow them if the robot
       // chases something else yellow in the room (wood floor, a sign, a shirt).
       pros::AIVision::Color yellow = {.id = (uint8_t)TRK_SIG_ID,
                                       .red = 235, .green = 200, .blue = 40,
                                       .hue_range = 25, .saturation_range = 0.75};
       aivision.set_color(yellow);
   }
   pros::delay(100);   // let the signature take effect before the first read

   // Biggest yellow blob, with nearby yellow blobs merged into its bounding box.
   auto findYellow = [&](double& cxOut, double& wOut) -> bool {
       auto objects = aivision.get_all_objects();

       // pass 1: the biggest acceptable yellow box
       double bestL = 0, bestR = 0, bestArea = 0;
       bool found = false;
       for (auto& o : objects) {
           if (!pros::AIVision::is_type(o, pros::AivisionDetectType::color)) continue;
           if ((int)o.id != TRK_SIG_ID) continue;
           const double w = o.object.color.width;
           const double h = o.object.color.height;
           if (w < TRK_MIN_W_PX) continue;
           const double area = w * h;
           if (area > bestArea) {
               bestArea = area;
               bestL = o.object.color.xoffset;
               bestR = bestL + w;
               found = true;
           }
       }
       if (!found) return false;

       // pass 2: swallow any yellow box close to the winner, so one pin split
       // into several boxes is treated as one pin
       const double reach = (bestR - bestL) * TRK_MERGE_FACTOR;
       double l = bestL, r = bestR;
       for (auto& o : objects) {
           if (!pros::AIVision::is_type(o, pros::AivisionDetectType::color)) continue;
           if ((int)o.id != TRK_SIG_ID) continue;
           const double w = o.object.color.width;
           if (w < TRK_MIN_W_PX) continue;
           const double ol = o.object.color.xoffset;
           const double orr = ol + w;
           if (orr < bestL - reach || ol > bestR + reach) continue;
           if (ol  < l) l = ol;
           if (orr > r) r = orr;
       }

       cxOut = (l + r) / 2.0;
       wOut  = r - l;
       return true;
   };

   double   turn       = 0;   // command actually being applied, after the slew limit
   double   errF       = 0;   // filtered error, pixels, + = pin right of the aim point
   bool     haveErrF   = false;
   bool     turning    = false;   // hysteresis latch
   uint32_t releasedAt = 0;       // when the latch last released, for the settle dwell
   bool     everReleased = false;
   int      lastDir    = 1;   // which way the pin was last seen, for the search creep
   int      goodFrames = 0;
   double   lastRawCx  = -1, lastRawW = -1;   // to spot a genuinely new camera frame
   double   lastGoodCx = AIV_IMAGE_CENTER_PX; // where it was when last detected
   bool     everSeen   = false;
   uint32_t lastSeen   = pros::millis();
   uint32_t lastReport = 0;
   double   shownW     = 0;
   int      state      = 0;   // 0 none, 1 tracking, 2 coasting blind, 3 searching

   while (true) {
       const uint32_t now = pros::millis();
       double cx = 0, wpx = 0;
       double want = 0;
       const bool detected = findYellow(cx, wpx);

       if (detected) {
           lastSeen   = now;
           everSeen   = true;
           lastGoodCx = cx;
           shownW     = wpx;
           if (goodFrames < TRK_ACQUIRE_FRAMES) goodFrames++;

           // Filter only on a NEW frame, so reading the same frame three times
           // does not triple the filter's effect.
           const bool newFrame = (cx != lastRawCx) || (wpx != lastRawW);
           if (newFrame) {
               lastRawCx = cx;
               lastRawW  = wpx;
               const double errRaw = cx - (AIV_IMAGE_CENTER_PX + TRK_AIM_OFFSET_PX);
               if (!haveErrF) { errF = errRaw; haveErrF = true; }
               else           { errF += TRK_FILTER * (errRaw - errF); }
           }
       }

       if (detected && goodFrames >= TRK_ACQUIRE_FRAMES && haveErrF) {
           state = 1;
           const double absErr = std::fabs(errF);

           // Hysteresis latch: engage wide, release narrow, and after releasing
           // stay put for TRK_SETTLE_MS before any new engage is allowed.
           const bool settled = everReleased && (now - releasedAt < TRK_SETTLE_MS);
           if (!turning && absErr > TRK_ENGAGE_PX && !settled) turning = true;
           if ( turning && absErr < TRK_RELEASE_PX) {
               turning      = false;
               releasedAt   = now;
               everReleased = true;
           }

           if (!turning) {
               want = 0;
           } else {
               lastDir = (errF > 0) ? 1 : -1;
               double mag = TRK_MIN_TURN + TRK_KP * (absErr - TRK_RELEASE_PX);
               if (mag < TRK_MIN_TURN) mag = TRK_MIN_TURN;
               if (mag > TRK_MAX_TURN) mag = TRK_MAX_TURN;
               want = TRK_SIGN * lastDir * mag;
           }
       }
       else if (everSeen && now - lastSeen < TRK_HOLD_MS) {
           // Blind. Almost certainly the detector blinking. Coast smoothly down
           // toward a stop instead of holding a big command through blind time.
           state   = 2;
           turning = false;
           want    = turn * TRK_HOLD_DECAY;
           if (std::fabs(want) < 1.0) want = 0;
       }
       else if (everSeen && now - lastSeen < TRK_HOLD_MS + TRK_SWEEP_MS &&
                (lastGoodCx <= TRK_EDGE_PX ||
                 lastGoodCx >= AIV_IMAGE_WIDTH_PX - TRK_EDGE_PX)) {
           // Gone a long time AND last seen at the edge, so it really did leave
           // the view. Creep the way it went to bring it back.
           state      = 3;
           turning    = false;
           goodFrames = 0;
           haveErrF   = false;
           want       = TRK_SIGN * lastDir * TRK_SWEEP_TURN;
       }
       else {
           // Gone, but it vanished mid-image: a detection failure, not a
           // departure. Stay still and wait for it to come back.
           state      = 0;
           turning    = false;
           goodFrames = 0;
           haveErrF   = false;
           want       = 0;
       }

       // Slew limit on the SIZE of the command, so it is identical for clockwise
       // and counter-clockwise. Gentle to speed up, quicker to back off.
       const bool reducing = (std::fabs(want) < std::fabs(turn)) || (want * turn < 0);
       const double step = reducing ? TRK_SLEW_DOWN : TRK_SLEW_UP;
       double diff = want - turn;
       if (diff >  step) diff =  step;
       if (diff < -step) diff = -step;
       turn += diff;

       // Pure rotation: equal and opposite, no forward component ever.
       leftMotors.move(turn);
       rightMotors.move(-turn);

       if (now - lastReport >= 250) {
           lastReport = now;
           if (state == 1) {
               controller.print(0, 0, "Pin e%d w%d    ", (int)errF, (int)shownW);
           } else if (state == 2) {
               controller.print(0, 0, "Pin: blink     ");
           } else if (state == 3) {
               controller.print(0, 0, "Pin: sweep %d  ", lastDir);
           } else {
               controller.print(0, 0, "Pin: none      ");
           }
       }

       pros::delay(10);
   }

// ---------------------------------------------------------------------
// OLD v1 TRACKER, COMMENTED OUT. Kept for reference only. It used the AI game
// model plus the width-based angle maths described above, which carried the
// one-directional bias. The trackPinInPlace() function it calls is still
// defined further up the file and is simply no longer used.
// ---------------------------------------------------------------------
#if 0


   // PIN TRACKING DEMO. Pure rotation, the robot never translates. Put a pin
   // behind the robot, move it around, and the robot should keep turning so the
   // pin stays on the drivetrain centreline.
   //
   // Controller line 0 shows "Trk e<error> t<targetX>". e is the angle from the
   // drivetrain centreline to the pin in DEGREES (right of it is positive) and
   // should settle inside TRACK_DEADBAND_DEG. t is the image column the centreline
   // currently sits at, for reference only.
   //
   // "Trk: mem e<error>" means the camera has lost the pin and the robot is turning
   // toward where it was last seen (see TRACK_MEMORY_MS, TRACK_MEMORY_MAX_TURN).
   //
   // Still overshooting: raise TRACK_VISION_LATENCY_MS, lower TRACK_MAX_TURN.
   // Stops short or sluggish: lower TRACK_VISION_LATENCY_MS, raise TRACK_MIN_TURN.
   // Check AIV_PIN_WIDTH_IN, AIV_HFOV_DEG and AIV_CAMERA_BACK_IN, they set the
   // angle maths.
   //
   chassis.cancelAllMotions();

   // Nothing here drives the lift. Leave it braked where it is rather than limp.
   liftAutonomousMode = false;
   liftPIDEnabled = false;
   liftLeft.brake();
   liftRight.brake();

   trackPinInPlaceReset();

   uint32_t lastReport = 0;
   while (true) {
       trackPinInPlace();

       const uint32_t now = pros::millis();
       if (now - lastReport >= 250) {
           lastReport = now;
           if (aivTrackHasPin) {
               controller.print(0, 0, "Trk e%d t%d    ",
                                (int)aivTrackError, (int)aivTrackTargetX);
           } else {
               if (aivTrackUsingMemory) {
                   controller.print(0, 0, "Trk: mem e%d    ", (int)aivTrackError);
               } else {
                   controller.print(0, 0, "Trk: no pin    ");
               }
           }
       }

       pros::delay(10);
   }

#endif   // old v1 pin tracker

#else
/*
   liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

   liftAutonomousMode = true;
   enableLiftPID();

   // Arm down so the claw is at pickup height, claw spinning to take the pin in.
   
   chassis.setPose(0, 0, 180);
   claw.move_voltage(12000);

   // Toggle roller while the claw runs.

   setLiftTarget(145);
   chassis.moveToPoint(0, 8, 1000, {.forwards=false, .minSpeed = 33});   // timeout was 750
   chassis.waitUntilDone();
   
   setLiftTarget(0);
   chassis.turnToHeading(0, 4500, {.maxSpeed=60});   // timeout was 1000
   
   chassis.moveToPoint(0, -5, 1250, {.forwards = false, .minSpeed = 33});   // timeout was 1000
  


   chassis.moveToPoint(5, 18, 3000, {.maxSpeed = 60});   // timeout was 1000
   chassis.waitUntil(4);
    moveArmAuton(-90);
   
   chassis.turnToHeading(235, 2000);   // timeout was 1000
   chassis.waitUntilDone();
   aivSetDriveSpeed(50);
AivEnd pinEnd = driveBackToPin(48, 10000, 50, 6, 750);
pros::delay(500);

   chassis.turnToHeading(180, 1250);   // timeout was 500
    moveArmAuton(90);
    setLiftTarget(60);
    chassis.moveToPoint(26, 24, 2500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    setLiftTarget(0);
    pros::delay(750);
    claw.move_voltage(-12000);
    pros::delay(500);
    chassis.moveToPoint(24, 35, 3000, {.forwards=false, .maxSpeed = 50});
    chassis.waitUntil(3);
    claw.move_voltage(12000);
    moveArmAuton(-90);
    chassis.turnToHeading(305, 2000);   // timeout was 1000
    chassis.waitUntilDone();
   aivSetDriveSpeed(50);
    AivEnd pinEnd2 = driveBackToPin(30, 10000, 50, 6, 1000);
    pros::delay(500);
    chassis.turnToHeading(0, 1250);   // timeout was 500
    chassis.moveToPoint(50, 13, 4000, {.maxSpeed = 60});   // timeout was 1000
    chassis.turnToHeading(-90, 1750);   // timeout was 500
    moveArmAuton(90);
    setLiftTarget(100);
    chassis.moveToPoint(24, 13, 3250, {.maxSpeed = 60});   // timeout was 1000
    chassis.waitUntilDone();
    
    setLiftTarget(60);
    
    chassis.setPose(24,12,-90);
    pros::delay(500);
    claw.move_voltage(-12000);
    chassis.moveToPoint(27, 12, 1000, {.forwards=false, .maxSpeed = 50});
    pros::delay(1200);
*/

liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

   liftAutonomousMode = true;
   enableLiftPID();

   
   chassis.setPose(0, 0, 180);
   
   
   claw.move_voltage(12000);

   // Toggle roller while the claw runs.

   setLiftTarget(145);
   chassis.moveToPoint(0, 8, 750, {.forwards=false, .minSpeed = 33});
   moveArmAuton(90);
   chassis.waitUntilDone();
   setLiftTarget(0);
   chassis.turnToHeading(0, 750, {.maxSpeed=60});
   chassis.moveToPoint(0, -4, 1500, {.forwards = false, .minSpeed = 33});
   pros::delay(500);
   moveArmAuton(-90);


   chassis.moveToPoint(5, 18, 1000, {.maxSpeed = 60});
   
   chassis.turnToHeading(225, 1000);
    chassis.moveToPoint(31, 44, 3000, {.forwards=false, .maxSpeed = 50});
    chassis.turnToHeading(180, 500);
    moveArmAuton(90);
    setLiftTarget(60);
    chassis.moveToPoint(26, 20, 1000, {.maxSpeed = 60});
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
    chassis.moveToPoint(52, 8, 3000, {.forwards=false, .maxSpeed = 50});
    pros::delay(500);
    chassis.moveToPoint(47, 14, 5000, {.maxSpeed = 50});
    chassis.turnToHeading(-90, 500);
    moveArmAuton(90);
    setLiftTarget(100);
    chassis.moveToPoint(24, 14, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    setLiftTarget(60);
    pros::delay(500);
    claw.move_voltage(-12000);
    pros::delay(2000);
    
   //standalone skills

   //return to loader
chassis.setPose(-47.223, -34.389, 0);
chassis.moveToPoint(-47.223, -34.389-24, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=30});   // timeout was 1500
chassis.waitUntil(4);
setLiftTarget(0);
claw.move_voltage(12000);
moveArmAuton(-90);
chassis.turnToHeading(90, 1750);   // timeout was 1000
chassis.moveToPoint(-46.223-18,-34.389-24, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=30});   // timeout was 1500
pros::delay(1000);

//third pin
chassis.setPose(-62.253, -58.81, 90);

chassis.moveToPoint(-62.253+12, -58.81, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(0, 1750);   // timeout was 1000
moveArmAuton(90);
setLiftTarget(170);
chassis.moveToPoint(-62.253+12, -58.81+27, 3750, {.maxSpeed=50, .minSpeed=30});   // timeout was 2000
chassis.waitUntilDone();
setLiftTarget(110);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);

//return to loader
chassis.setPose(-47.223, -34.389, 0);
chassis.moveToPoint(-47.223, -34.389-24, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=30});   // timeout was 1500
chassis.waitUntil(4);
setLiftTarget(0);
claw.move_voltage(12000);
moveArmAuton(-90);
chassis.turnToHeading(90, 1750);   // timeout was 1000
chassis.moveToPoint(-46.223-18,-34.389-24, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});   // timeout was 1500
pros::delay(1000);

//fourth pin
chassis.setPose(-62.253, -58.81, 90);

chassis.moveToPoint(-62.253+13, -58.81, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(0, 1750);   // timeout was 1000
moveArmAuton(90);
setLiftTarget(190);
chassis.moveToPoint(-62.253+14, -58.81+27, 3750, {.maxSpeed=50, .minSpeed=30});   // timeout was 2000
chassis.waitUntilDone();
setLiftTarget(150);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);

//back to loader
chassis.setPose(-47.223, -34.389, 0);
chassis.moveToPoint(-47.223, -34.389-24, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=30});   // timeout was 1500
chassis.waitUntil(4);
setLiftTarget(0);
claw.move_voltage(12000);
moveArmAuton(-90);
chassis.turnToHeading(90, 1750);   // timeout was 1000
chassis.moveToPoint(-46.223-18,-34.389-24, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=30});   // timeout was 1500
pros::delay(1000);



chassis.setPose(0,0,0);
chassis.moveToPoint(0, 15, 2500, {.maxSpeed=50});   // timeout was 2000
chassis.turnToHeading(-90, 1750);   // timeout was 1000
chassis.moveToPoint(-10, 15, 2000, {.maxSpeed=50});
chassis.turnToHeading(-45, 1250);   // timeout was 1000
chassis.moveToPoint(-59, 59, 8000, {.maxSpeed=50});   // timeout was 4000
moveArmAuton(90);
setLiftTarget(90);
chassis.waitUntilDone();
setLiftTarget(40);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);
   
/*



   // =====================================================================
   // PID TUNING TEST  (AUTON_MODE 0)
   // =====================================================================
   // Repeats one motion several times from the same start and reports how far
   // off it finished each time. Tune against the SPREAD of the numbers, not a
   // single run: one run cannot tell a gain problem from a bad launch.
   //
   //   TUNE_TEST 0 = turn test      (tune angularController with this)
   //   TUNE_TEST 1 = straight drive (tune linearController, only after turns)
   //   TUNE_TEST 2 = spin in place  (measures the tracking wheel offset)
   //
   // TEST 2: a full 360 moves the robot nowhere, so any position the odometry
   // reports afterwards is error from the vertical tracking wheel's offset
   // being wrong. A wheel mounted d inches off the centre line sweeps an arc of
   // 2*pi*d per rotation, so  offset = drift / (2*pi*rotations).  Put that
   // number in the third argument of the TrackingWheel constructor.
   //
   // Controller screen, after each rep:  "n) err <degrees or inches>"
   // At the end:                         "worst <x>  avg <x>"  + a rumble.
   // =====================================================================
   const int      TUNE_TEST      = 0;      // 0 = turns, 1 = drive, 2 = spin
   const int      TUNE_REPS      = 6;      // test 0/1: even number. test 2: full rotations
   const double   TUNE_TURN_DEG  = 90.0;   // turn test: swings between 0 and this
   const double   TUNE_DRIVE_IN  = 24.0;   // drive test: moves between 0 and this
   const int      TUNE_DRIVE_SPD = 90;     // drive test: maxSpeed
   const uint32_t TUNE_PAUSE_MS  = 1200;   // settle time before each reading

   // Nothing but the drivetrain moves during a tuning run.
   liftAutonomousMode = false;
   liftPIDEnabled     = false;
   liftLeft.brake();
   liftRight.brake();
   claw.brake();

   auto tuneWrap = [](double a) {
       while (a >  180.0) a -= 360.0;
       while (a < -180.0) a += 360.0;
       return a;
   };

   chassis.setPose(0, 0, 0);
   pros::delay(500);

   double tuneWorst = 0, tuneSum = 0;
   double tuneSpinDrift = 0;   // TEST 2: inches the pose has wandered from the origin

   for (int i = 0; i < TUNE_REPS; i++) {
       double err = 0;

       if (TUNE_TEST == 0) {
           const double target = (i % 2 == 0) ? TUNE_TURN_DEG : 0.0;
           chassis.turnToHeading(target, 4000);
           chassis.waitUntilDone();
           pros::delay(TUNE_PAUSE_MS);
           err = tuneWrap(target - chassis.getPose().theta);
       }
       else if (TUNE_TEST == 1) {
           const double target = (i % 2 == 0) ? TUNE_DRIVE_IN : 0.0;
           chassis.moveToPoint(0, target, 5000, {.maxSpeed = TUNE_DRIVE_SPD});
           chassis.waitUntilDone();
           pros::delay(TUNE_PAUSE_MS);
           err = target - chassis.getPose().y;
       }
       else {
           // One full rotation, in four quarter turns so turnToHeading always
           // has somewhere to go. Net rotation is 360, net travel is zero.
           for (int q = 1; q <= 4; q++) {
               chassis.turnToHeading(tuneWrap(q * 90.0), 4000);
               chassis.waitUntilDone();
           }
           pros::delay(TUNE_PAUSE_MS);
           const lemlib::Pose sp = chassis.getPose();
           tuneSpinDrift = std::hypot(sp.x, sp.y);   // cumulative, never reset
           err = tuneSpinDrift;
       }

       const double mag = std::fabs(err);
       tuneSum += mag;
       if (mag > tuneWorst) tuneWorst = mag;

       controller.print(0, 0, "%d) err %+.2f    ", i + 1, err);
       pros::delay(600);          // controller.print needs room between writes
   }

   controller.print(1, 0, "worst %.2f avg %.2f ",
                    tuneWorst, tuneSum / (double)TUNE_REPS);
   if (TUNE_TEST == 2) {
       pros::delay(600);
       controller.print(2, 0, "offset %.3f in   ",
                        tuneSpinDrift / (2.0 * 3.14159265358979 * TUNE_REPS));
   }
   controller.rumble(".");
   leftMotors.brake();
   rightMotors.brake();

   */

   #endif

}


// =====================================================================
// DRIVER SCORING
// =====================================================================

// ---- LIFT DEGREES <-> HEIGHT -----------------------------------------
// Measured pairs: lift angle vs height of the BOTTOM OF THE PIN in inches.
// Both columns must be strictly increasing.
const bool HEIGHT_TABLE_CALIBRATED = true;

const int kHN = 6;
const double kHAngle[kHN] = {0,   60,  120,  180,  220,  240};   // lift degrees
// The first five are measured. The 240 point (33.0 in) is an EXTRAPOLATION of the
// 180->220 slope (0.125 in/deg). Park the lift at 240, measure the bottom of the pin, and replace it.
const double kHInch[kHN]  = {1,  7.5,   16, 25.5, 30.5, 33.0};

// Seated pin-bottom height needed to score pin 1..4, in inches.
// Each pin+cup pair adds 10 in, and a pair is scored at a time, so every pin is
// 10 in above the previous one.
//   neutral pin 1 = 8.77 (goal top) - 2.93 (insert depth) = 5.84
//   alliance pin 1 = 3.25 - 2.93 = 0.32, clamped to 1.00 (lift cannot go lower)
// Neutral pin 4 (35.84 in) is above the top of the table; it will go as high as
// the lift can and no further. Change the 33.0 in the table above once measured.
const double kNeutralPinIn[4]  = {1, 15.84, 20.84, 26.84};
const double kAlliancePinIn[4] = {1.00, 9.32, 16.32, 22.32}; // {1.00, 10.32, 17.32, 23.32}

double interpTable(const double* xs, const double* ys, int n, double x) {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (x <= xs[i + 1]) {
            double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }
    return ys[n - 1];
}

double liftHeightFromAngle(double angleDeg) { return interpTable(kHAngle, kHInch, kHN, angleDeg); }
double liftAngleFromHeight(double inches)   { return interpTable(kHInch, kHAngle, kHN, inches); }

// ---- SEQUENCE CONSTANTS (tune these) ----------------------------------
const double SEQ_OVER_IN  = 1.0;    // lift goes this far above the goal height
const double SEQ_DROP_IN  = 5.0;    // then comes down this far so the pin sits in the cup
                                    // (pin ends 1 in below table value; set 2.0 to end exactly at it)

// Alliance goal, first pin only: do NOT move the lift, just flip the arm (raise the
// arm, flip it to the front, release the claw, reset the arm). Set false to put the
// lift back into that sequence.
const bool ALLIANCE_PIN1_ARM_ONLY = true;

// Every scoring sequence (neutral and alliance, all 4 pins): after the claw releases the
// pin, drive straight BACKWARD this many inches, then start the reset. Set SEQ_BACKUP_IN
// to 0 to turn it off.
// The sequence drives the wheels itself during this, the joysticks are ignored until it ends.
const double   SEQ_BACKUP_IN         = 1.0;   // inches to drive backward after the release
const double   SEQ_BACKUP_SPEED      = 20;    // move() units, always applied backward
const double   SEQ_BACKUP_KH         = 2.0;   // heading hold gain, move() units per degree
const uint32_t SEQ_BACKUP_TIMEOUT_MS = 800;   // safety net: stop backing up after this long either way

// Arm rotation quantities, in arm motor degrees, relative to where the arm is when it starts.
const double SEQ_ARM_RAISE_ROT = 500;   // arm up before/while the lift rises
const double SEQ_ARM_FLIP_ROT  = 900;   // total rotation from start to fully flipped to the front
const double ARM_UP_ROT_EMPTY  = 500;   // TODO: R2, arm up about 90 degrees, no pin
const double ARM_UP_ROT_HELD   = 500;   // TODO: L2, arm up about 90 degrees, holding a pin
const double LIFT_SETTLE_TOL   = 3.0;   // lift degrees
const uint32_t SEQ_RELEASE_MS  = 600;   // how long the claw spins in reverse to let go
const uint32_t SEQ_LIFT_DELAY_MS = 500; // 350 lift waits this long after the arm starts moving
const uint32_t SEQ_FLIP_DELAY_MS = 150; // after the lift reaches height, wait this long before the arm flips (was hardcoded)
const double   SEQ_FLIP_LIFT_TOL = 20.0;    // arm starts flipping once the lift is within this many degrees of its height (LIFT_SETTLE_TOL is too strict here)
const uint32_t SEQ_RAISE_TIMEOUT_MS = 1500; // flip anyway if the lift/arm have not got there after this long (was 4000)
const int32_t  SEQ_FLIP_VOLTAGE      = 12000; // arm voltage during the flip (full power, same as the other arm moves)
const int32_t  SEQ_FLIP_HOLD_VOLTAGE = 5500;  //4000 arm keeps pressing down with this once it is flipped (same as auton hold)
const uint32_t SEQ_ARM_SETTLE_MS = 200; // after the arm reaches the flip position, wait this long before the lift lowers
const uint32_t SEQ_PRE_RELEASE_MS = 50; // after the lift lowers, wait at least this long before the claw lets go

// Reset. The arm flips back to straight up (the position it was raised to) while
// the lift starts descending. After SEQ_RESET_ARM_BACK_DELAY_MS the arm continues
// back to its resting position. The reset ends when the lift is down and the arm
// has stopped; the max is only a safety net.
const uint32_t SEQ_RESET_ARM_BACK_DELAY_MS = 850;   //700 arm stops at upright this long, then continues back
const int32_t  SEQ_RESET_ARM_UP_VOLTAGE    = 12000; // arm voltage flipping back to straight up
const int32_t  SEQ_RESET_ARM_BACK_VOLTAGE  = 6000;  // arm voltage for the final move back (was -6000)
const uint32_t SEQ_RESET_BACK_MIN_MS = 200;   // after the arm starts going back, never finish before this
const uint32_t SEQ_RESET_MAX_MS = 1500;       // backup timeout
const uint32_t SEQ_QUICK_RESET_MAX_MS = 2500; // L1 quick reset: safety timeout (longer, the lift may start from the top)
const double   SEQ_RESET_LIFT_DONE_DEG = 5.0; // lift counts as "all the way down" at or below this

// Extra lift height per pin (inches), added on top of the pin height tables.
// Zero now that the tables use the 10 in per pair spacing. Nudge a pin here if it is off.
const double SEQ_PIN_EXTRA_IN[4] = {0.0, 0.0, 0.0, 0.0};

// ---- DRIVER LIFT PID HELPERS ------------------------------------------
void setDriverLiftTarget(double angleDeg) {
    if (angleDeg < LIFT_MIN) angleDeg = LIFT_MIN;
    if (angleDeg > LIFT_MAX) angleDeg = LIFT_MAX;
    if (std::fabs(angleDeg - liftTarget) > 0.5) driverLiftPID.clearIntegral();
    liftTarget = angleDeg;
}

// call right before using the driver PID so it does not slam to an old target
void enableDriverLiftPID() {
    double angle = getLiftAngle();
    liftTarget = angle;
    driverLiftPID.reset(angle);
    activeLiftPID = &driverLiftPID;
    liftLastOutput = liftFF(angle);
    liftPIDEnabled = true;
}

bool liftSettled() {
    return liftSensorOK && std::fabs(liftTarget - liftAngleNow) <= LIFT_SETTLE_TOL;
}

// ---- SHARED ARM STATE (global so the sequence can touch it) ------------
int  lastArmDirection = 0;      // 1 = up push, 2 = up brake-hold, -1 = down hold, 0 = none
bool armMoveRunning = false;    // R2 / L2 position move in progress
double armMoveTarget = 0;
uint32_t armMoveStart = 0;
bool armCoast = false;

void setArmBrake(bool coast) {
    if (coast != armCoast) {
        arm.set_brake_mode(coast ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_HOLD);
        armCoast = coast;
    }
}

// ---- SCORING SEQUENCE ---------------------------------------------------
enum SeqState { SEQ_IDLE, SEQ_ARM_UP, SEQ_RAISE, SEQ_FLIP, SEQ_LOWER, SEQ_RELEASE, SEQ_BACKUP, SEQ_RESET };
SeqState seqState = SEQ_IDLE;
uint32_t seqStateStart = 0;
double seqBaseArm = 0;
double seqHeightIn = 0;
bool seqArmHeld = false;
bool seqNoLift = false;       // true = this run only flips the arm, the lift is never moved
uint32_t seqFlipArrive = 0;   // time the arm reached the flip position, 0 = not yet
bool seqDoBackup = false;     // true = drive backward SEQ_BACKUP_IN inches after the release (every pin)
bool seqResetNow = false;     // true = L1 quick reset: lift down and arm back at once, nothing else
lemlib::Pose seqBackupStart(0, 0, 0);   // pose when the backup began
double seqBackupHeading = 0;            // IMU heading held during the backup

bool seqActive() { return seqState != SEQ_IDLE; }

void seqEnter(SeqState s) {
    seqState = s;
    seqStateStart = pros::millis();
}

void startSequence(bool neutral, int pin) {
    if (!HEIGHT_TABLE_CALIBRATED) {
        controller.rumble("---");
        return;
    }
    seqHeightIn = (neutral ? kNeutralPinIn[pin] : kAlliancePinIn[pin]) + SEQ_PIN_EXTRA_IN[pin];

    // Alliance goal, first pin: arm only, the lift stays where it is.
    seqNoLift = ALLIANCE_PIN1_ARM_ONLY && !neutral && pin == 0;

    // Every pin, neutral and alliance: back up SEQ_BACKUP_IN inches after the release.
    seqDoBackup = SEQ_BACKUP_IN > 0;
    seqResetNow = false;

    armMoveRunning = false;
    setArmBrake(false);
    seqBaseArm = arm.get_position();
    arm.move_absolute(seqBaseArm + SEQ_ARM_RAISE_ROT, 200);
    seqArmHeld = false;

    // The lift does NOT move yet. Hold it still until the arm is up.
    liftPIDEnabled = false;
    liftLeft.brake();
    liftRight.brake();

    seqEnter(SEQ_ARM_UP);
}

void cancelSequence() {
    seqState = SEQ_IDLE;
    seqResetNow = false;
    liftPIDEnabled = false;
    liftLeft.brake();
    liftRight.brake();
    arm.brake();
    lastArmDirection = 0;
}

// Starts the reset: lift down and arm back to upright, driven in SEQ_RESET.
void seqBeginReset() {
    liftPIDEnabled = false;
    liftLeft.set_current_limit(750); //300
    liftRight.set_current_limit(750); //300
    seqEnter(SEQ_RESET);
}

// L1: reset lift and arm to the back at once. No flip, no backup, no waiting for
// the arm to reach upright first.
void startQuickReset() {
    armMoveRunning = false;
    seqNoLift = false;
    seqDoBackup = false;
    seqResetNow = true;
    setArmBrake(false);
    seqBeginReset();
}

void updateSequence() {
    const uint32_t t = pros::millis() - seqStateStart;

    switch (seqState) {
    case SEQ_ARM_UP: {
        // Arm is moving up. Lift stays put for SEQ_LIFT_DELAY_MS so the arm is
        // well clear before the lift starts rising.
        if (!seqArmHeld && arm.get_position() >= seqBaseArm + SEQ_ARM_RAISE_ROT - 5.0) {
            arm.brake();
            seqArmHeld = true;
        }

        if (seqNoLift) {
            // No lift to wait for. Flip as soon as the arm is up (or the same
            // safety timeout as the normal sequence).
            if ((seqArmHeld && t >= SEQ_FLIP_DELAY_MS) || t >= SEQ_RAISE_TIMEOUT_MS) {
                arm.move_voltage(SEQ_FLIP_VOLTAGE);
                seqFlipArrive = 0;
                seqEnter(SEQ_FLIP);
            }
            break;
        }

        if (t >= SEQ_LIFT_DELAY_MS) {
            enableDriverLiftPID();
            setDriverLiftTarget(liftAngleFromHeight(seqHeightIn + SEQ_OVER_IN));
            seqEnter(SEQ_RAISE);
        }
        break;
    }
    case SEQ_RAISE: {
        // Lift is rising. Arm brakes at its raised position if it has not already.
        if (!seqArmHeld && arm.get_position() >= seqBaseArm + SEQ_ARM_RAISE_ROT - 5.0) {
            arm.brake();
            seqArmHeld = true;
        }
        // Flip as soon as the lift is CLOSE to height. Waiting for it to fully
        // settle (3 deg) is what caused the long pause, the last few degrees creep.
        const bool liftClose = liftSensorOK &&
                               std::fabs(liftTarget - liftAngleNow) <= SEQ_FLIP_LIFT_TOL;
        if ((seqArmHeld && t >= SEQ_FLIP_DELAY_MS && liftClose) || t >= SEQ_RAISE_TIMEOUT_MS) {
            arm.move_voltage(SEQ_FLIP_VOLTAGE);
            seqFlipArrive = 0;
            seqEnter(SEQ_FLIP);
        }
        break;
    }
    case SEQ_FLIP: {
        // Full voltage the whole way, then a steady press-down hold. move_absolute
        // slows down and gives up short of the end under load, this does not.
        // Counts as arrived when the arm reaches the target, OR when it stops moving
        // (it has hit its end stop), so a too-big SEQ_ARM_FLIP_ROT cannot stall it.
        const double flipTarget = seqBaseArm + SEQ_ARM_FLIP_ROT;
        arm.move_voltage(seqFlipArrive == 0 ? SEQ_FLIP_VOLTAGE : SEQ_FLIP_HOLD_VOLTAGE);

        const bool atTarget = arm.get_position() >= flipTarget - 5.0;
        const bool stalled  = t >= 300 && std::fabs(arm.get_actual_velocity()) < 5.0;
        if ((atTarget || stalled) && seqFlipArrive == 0) {
            seqFlipArrive = pros::millis();
            arm.move_voltage(SEQ_FLIP_HOLD_VOLTAGE);
        }
        const bool armSettled = seqFlipArrive != 0 &&
                                pros::millis() - seqFlipArrive >= SEQ_ARM_SETTLE_MS;
        if (armSettled || t >= 3000) {
            seqFlipArrive = 0;
            arm.move_voltage(SEQ_FLIP_HOLD_VOLTAGE);   // keep pressing down through lower + release
            if (seqNoLift) {
                // Arm only: nothing to lower, go straight to the release.
                seqEnter(SEQ_RELEASE);
            }
            else {
                // come down 3 inches from the raised height
                setDriverLiftTarget(liftAngleFromHeight(seqHeightIn + SEQ_OVER_IN - SEQ_DROP_IN));
                seqEnter(SEQ_LOWER);
            }
        }
        break;
    }
    case SEQ_LOWER:
        if ((t >= SEQ_PRE_RELEASE_MS && liftSettled()) || t >= 2500) seqEnter(SEQ_RELEASE);
        break;

    case SEQ_RELEASE:
        // claw reverse is applied in the claw section of opcontrol
        if (t >= SEQ_RELEASE_MS) {
            if (seqDoBackup) {
                // Drive back first, the reset starts when that ends.
                seqBackupStart = chassis.getPose();
                seqBackupHeading = imu.get_rotation();
                seqEnter(SEQ_BACKUP);
            }
            else {
                seqBeginReset();
            }
        }
        break;

    case SEQ_BACKUP: {
        // The sequence drives the wheels here, opcontrol skips the joysticks.
        // The claw keeps spinning in reverse (see the claw section) so the pin
        // stays released while the robot pulls away.
        const lemlib::Pose p = chassis.getPose();
        const double moved = std::hypot(p.x - seqBackupStart.x, p.y - seqBackupStart.y);

        if (moved >= SEQ_BACKUP_IN || t >= SEQ_BACKUP_TIMEOUT_MS) {
            leftMotors.brake();
            rightMotors.brake();
            seqBeginReset();
            break;
        }

        // Hold the heading so it backs out straight. IMU rotation increases
        // clockwise and positive turn is clockwise.
        double turn = SEQ_BACKUP_KH * (seqBackupHeading - imu.get_rotation());
        if (turn > 20) turn = 20;
        if (turn < -20) turn = -20;

        // Backward only. Steering can slow one side but never drives it forward.
        double l = -SEQ_BACKUP_SPEED + turn;
        double r = -SEQ_BACKUP_SPEED - turn;
        if (l > 0) l = 0;
        if (r > 0) r = 0;
        leftMotors.move(l);
        rightMotors.move(r);
        break;
    }

    case SEQ_RESET: {
        // Lift starts going down immediately (skipped on an arm-only run, the lift
        // never left the bottom).
        if (!seqNoLift) {
            liftLeft.move_voltage(-8000);
            liftRight.move_voltage(-8000);
        }

        // Arm first flips back to straight up (where it was raised to) while the
        // lift descends. After SEQ_RESET_ARM_BACK_DELAY_MS it continues back.
        // The L1 quick reset skips that and sends the arm straight back.
        const bool armBackPhase = seqResetNow || t >= SEQ_RESET_ARM_BACK_DELAY_MS;
        if (!armBackPhase) {
            const double upright = seqBaseArm + SEQ_ARM_RAISE_ROT;
            if (arm.get_position() > upright + 5.0) arm.move_voltage(-SEQ_RESET_ARM_UP_VOLTAGE);
            else arm.brake();
        }
        else {
            arm.move_voltage(-SEQ_RESET_ARM_BACK_VOLTAGE);
        }

        // PID is off during the reset, so the lift task is not updating the angle.
        getLiftAngle();
        const bool liftDown   = seqNoLift || (liftSensorOK && liftAngleNow <= SEQ_RESET_LIFT_DONE_DEG);
        const bool armStopped = std::fabs(arm.get_actual_velocity()) < 5.0;
        const uint32_t backStart = seqResetNow ? 0 : SEQ_RESET_ARM_BACK_DELAY_MS;
        const bool backLongEnough = t >= backStart + SEQ_RESET_BACK_MIN_MS;
        const uint32_t resetMax = seqResetNow ? SEQ_QUICK_RESET_MAX_MS : SEQ_RESET_MAX_MS;

        if ((backLongEnough && liftDown && armStopped) || t >= resetMax) {
            liftLeft.brake();
            liftRight.brake();
            arm.brake();
            lastArmDirection = -1;   // arm is down, so the claw can pull a pin without drag
            seqResetNow = false;
            seqState = SEQ_IDLE;     // claw resumes idle spin immediately
        }
        break;
    }

    default:
        break;
    }
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
    //
    // Shifters:   DOWN = neutral goals,  hold B = alliance goals
    //   shifter + L1/R1/L2/R2 = pin 1/2/3/4 scoring sequence
    // No shifter: claw idle 25%, R1 claw full + arm downforce (coast on release),
    //   L1 = quick reset (lift down + arm back, one button),
    //   RIGHT claw reverse, R2 arm up (empty), L2 arm up (holding pin)
    // X / A manual arm, UP / LEFT arrows manual lift.
    // X, A, UP or LEFT cancels a running sequence or reset.

    liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    armCoast = false;

    // Carry the lift over from autonomous. The rotation sensor is only zeroed in
    // initialize(), so whatever height autonomous left the lift at is still the
    // reading here. Do NOT cut the motors to 0 V (the lift would let go), brake so
    // it holds that height, drop the auton PID, and sync the target to where it is.
    liftPIDEnabled = false;
    getLiftAngle();
    liftTarget = liftAngleNow;
    liftLeft.brake();
    liftRight.brake();

    seqState = SEQ_IDLE;
    bool r1Prev = false;

    while (true) {

        // ==========================================
        // DRIVE
        // ==========================================

        // During the backup after a release the sequence drives the wheels itself.
        if (seqState != SEQ_BACKUP) {
            chassis.tank(
                controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
            );
        }

        // ==========================================
        // INPUTS (new_press polled every loop so no stale presses build up)
        // ==========================================

        const bool l1New = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
        const bool r1New = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
        const bool l2New = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
        const bool r2New = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);

        const bool neutralShift  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        const bool allianceShift = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        const bool shifter = neutralShift || allianceShift;

        const bool xHeld    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        const bool aHeld    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        const bool upHeld   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        const bool leftHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        const bool rightHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        const bool r1Held   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool l1Held   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        // ==========================================
        // SCORING SEQUENCE START / UPDATE / CANCEL
        // ==========================================

        if (seqActive()) {
            if (xHeld || aHeld || upHeld || leftHeld) cancelSequence();
            else updateSequence();
        }
        else if (shifter) {
            int pin = -1;
            if (r1New) pin = 0;
            else if (r2New) pin = 1;
            else if (l1New) pin = 2;
            else if (l2New) pin = 3;
            if (pin >= 0) startSequence(neutralShift, pin);
        }
        else if (l1New) {
            // L1 on its own: quick reset, lift down and arm back, one button.
            startQuickReset();
        }

        // ==========================================
        // CLAW
        // ==========================================

        if (seqState == SEQ_RELEASE || seqState == SEQ_BACKUP) {
            claw.move_voltage(-12000);
        }
        else if (seqState == SEQ_RESET && !seqResetNow) {
            claw.brake();                       // resumes only once everything is down
        }
        else if (seqActive() || shifter) {
            claw.move_voltage(3000);
        }
        else if (r1Held) {
            claw.move_voltage(12000);
        }
        else if (rightHeld) {
            claw.move_voltage(-12000);
        }
        else {
            claw.move_voltage(3000);
        }

        // ==========================================
        // ARM AND LIFT (only when the sequence is not driving them)
        // ==========================================

        if (!seqActive()) {

            // --- R2 / L2 arm position moves (not while a shifter is held) ---
            if (!shifter && (r2New || l2New)) {
                setArmBrake(false);
                armMoveTarget = arm.get_position() + (r2New ? ARM_UP_ROT_EMPTY : ARM_UP_ROT_HELD);
                armMoveStart = pros::millis();
                lastArmDirection = 0;
                armMoveRunning = (arm.move_absolute(armMoveTarget, 200) == 1);
            }

            if (armMoveRunning) {
                if (xHeld || aHeld) {
                    armMoveRunning = false;      // manual override
                }
                else if (arm.get_position() >= armMoveTarget - 5.0) {
                    // Reached (or passed) the target. Stop and brake-hold right here.
                    // No voltage push, that is what kept it drifting past the target.
                    armMoveRunning = false;
                    lastArmDirection = 2;      // 2 = up, held by brake
                    setArmBrake(false);        // make sure brake mode is HOLD
                    arm.brake();
                }
                else if (pros::millis() - armMoveStart >= 4000) {
                    armMoveRunning = false;
                    lastArmDirection = 0;
                    arm.brake();
                }
            }

            // --- manual arm ---
            if (!armMoveRunning) {
                if (xHeld) {
                    setArmBrake(false);
                    arm.move_voltage(12000);
                    lastArmDirection = 1;
                }
                else if (aHeld) {
                    setArmBrake(false);
                    arm.move_voltage(-12000);
                    lastArmDirection = -1;
                }
                else if (!shifter && r1Held) {
                    // Claw is intaking. Arm up -> keep holding it up.
                    // Anything else (down or unknown) -> press it down.
                    setArmBrake(false);
                    if (lastArmDirection == 1) arm.move_voltage(2500);
                    else if (lastArmDirection == 2) arm.brake();   // R2/L2 moved it up, keep it held
                    else arm.move_voltage(-5000);   // raise toward -4000/-5000 if too weak
                }
                else if (r1Prev && !r1Held) {
                    // R1 let go: coast so the pin does not drag
                    setArmBrake(true);
                    arm.move_voltage(0);
                    lastArmDirection = 0;
                }
                else {
                    if (lastArmDirection == 1) arm.move_voltage(2500);
                    else if (lastArmDirection == -1) arm.move_voltage(-3000);
                    else arm.brake();       // coasts if R1 put the arm in coast
                }
            }

            // --- manual lift: UP / LEFT arrows ---
            liftPIDEnabled = false;
            liftLeft.set_current_limit(2500);
            liftRight.set_current_limit(2500);
            if (upHeld) {
                liftLeft.move_voltage(12000);
                liftRight.move_voltage(12000);
            }
            else if (leftHeld) {
                liftLeft.move_voltage(-4000);
                liftRight.move_voltage(-4000);
            }
            else {
                liftLeft.brake();
                liftRight.brake();
            }
        }

        r1Prev = r1Held && !shifter;

        pros::delay(10);
    }

#endif
}