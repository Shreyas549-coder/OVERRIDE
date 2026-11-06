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
#define AUTON_MODE 2


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
const int    AIV_PIN_CLASS_ID    = 3;      // class ID of the pin in the game model (found with AUTON_MODE 1)
                                           // used when AIV_USE_GAME_MODEL is true

// ---- geometry ----
const double AIV_CAMERA_OFFSET_LEFT_IN = -6.75;  // camera is this far LEFT of drivetrain center, seen from behind
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
// AI VISION: TRACK A PIN IN PLACE (PURE ROTATION)
// =====================================================================
//
// One update step, non-blocking. Call it repeatedly from a loop. It never drives
// forward or backward: both sides always get equal and opposite commands, so the
// only motion possible is rotation about the robot's own centre.
//
// Same targeting and P control as driveBackToPin() minus the drive term, so
// anything tuned there carries straight over. Tracking state lives in the file
// statics below between calls, the way driveBackToPin() keeps it in locals
// across its own loop.
//
// Returns true while a pin is being tracked, false when none is in view.

// telemetry, so a caller can report what the tracker is doing
double aivTrackError   = 0;    // pixels, pin centre minus target column
double aivTrackTargetX = 0;    // the image column the pin is being driven to
double aivTrackTurn    = 0;    // move() units currently applied
bool   aivTrackHasPin  = false;

static double   aivTrackLastX    = AIV_IMAGE_CENTER_PX;
static double   aivTrackWindow   = AIV_FIRST_WINDOW_PX;
static uint32_t aivTrackLastSeen = 0;
static bool     aivTrackPrimed   = false;

// Call once before a tracking loop so it does not start on stale state.
void trackPinInPlaceReset() {
    aivTrackLastX    = AIV_IMAGE_CENTER_PX;
    aivTrackWindow   = AIV_FIRST_WINDOW_PX;
    aivTrackLastSeen = pros::millis();
    aivTrackTurn     = 0;
    aivTrackError    = 0;
    aivTrackHasPin   = false;
    aivTrackPrimed   = true;
}

bool trackPinInPlace() {
    if (!aivTrackPrimed) trackPinInPlaceReset();

    const uint32_t now = pros::millis();

    double cx = 0, wpx = 0;
    if (aivFindPin(aivTrackLastX, aivTrackWindow, cx, wpx)) {
        aivTrackLastSeen = now;
        aivTrackLastX    = cx;
        aivTrackWindow   = AIV_TRACK_WINDOW_PX;
        aivTrackHasPin   = true;

        const double pxPerIn = wpx / AIV_PIN_WIDTH_IN;
        const double targetX = AIV_IMAGE_CENTER_PX + AIV_CAMERA_OFFSET_LEFT_IN * pxPerIn;
        const double error   = cx - targetX;

        aivTrackTargetX = targetX;
        aivTrackError   = error;

        if (std::fabs(error) < AIV_DEADBAND_PX) aivTrackTurn = 0;
        else aivTrackTurn = aivClamp(AIV_STEER_SIGN * aivSteerKp * error, aivSteerMax);
    }
    else if (now - aivTrackLastSeen >= AIV_LOST_CONFIRM_MS) {
        // Really gone. Stop rotating rather than spinning forever on a stale
        // error, and reopen the search window so it can be reacquired anywhere
        // in frame.
        aivTrackHasPin = false;
        aivTrackTurn   = 0;
        aivTrackWindow = AIV_FIRST_WINDOW_PX;
        aivTrackLastX  = AIV_IMAGE_CENTER_PX;
    }
    // else: a frame or two missing, hold the last turn

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

   // PIN TRACKING DEMO. Pure rotation, the robot never translates. Put a pin
   // behind the robot, move it around, and the robot should keep turning so the
   // pin stays on the drivetrain centreline.
   //
   // Controller line 0 shows "Trk e<error> t<targetX>". Error should settle
   // inside AIV_DEADBAND_PX. If targetX ever reads outside 0..320 the pin is too
   // close for the offset maths to have a solution and the robot will hunt
   // without ever converging, because the column it is steering toward is off
   // the edge of the image. With AIV_CAMERA_OFFSET_LEFT_IN -6.75 and
   // AIV_PIN_WIDTH_IN 3.0 that happens once the blob is wider than about 71 px:
   //     targetX = 160 - 2.25 * widthPx
   // driveBackToPin() never hits this because it reaches the pin and ends first.
   // A standing tracker sits in it. Fix it by measuring AIV_PIN_WIDTH_IN
   // properly, or clamp targetX to the image.
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
               controller.print(0, 0, "Trk: no pin    ");
           }
       }

       pros::delay(10);
   }

#else

   liftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   liftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

   liftAutonomousMode = true;
   enableLiftPID();


   // Arm down so the claw is at pickup height, claw spinning to take the pin in.
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
  


   chassis.moveToPoint(5, 18, 1000, {.maxSpeed = 60});
   chassis.waitUntil(4);
    moveArmAuton(-90);
   
   chassis.turnToHeading(225, 1000);
   chassis.waitUntilDone();
   aivSetDriveSpeed(50);
AivEnd pinEnd = driveBackToPin(1000, 10000, 50, 8, 1000);
pros::delay(500);
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
    chassis.turnToHeading(315, 1000);
    chassis.waitUntilDone();
   aivSetDriveSpeed(50);
    AivEnd pinEnd2 = driveBackToPin(1000, 10000, 50, 6, 1000);
    pros::delay(500);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(50, 12, 1000, {.maxSpeed = 60});
    chassis.turnToHeading(-90, 500);
    moveArmAuton(90);
    setLiftTarget(100);
    chassis.moveToPoint(24, 12, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    
    setLiftTarget(60);
    
    chassis.setPose(24,12,-90);
    pros::delay(500);
    claw.move_voltage(-12000);
    chassis.moveToPoint(27, 12, 1000, {.forwards=false, .maxSpeed = 50});
    pros::delay(1200);

   //standalone skills

   //return to loader
   
chassis.setPose(-47.223, -34.389, 0);
chassis.moveToPoint(-47.223, -34.389-22, 1500, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(4);
setLiftTarget(0);
claw.move_voltage(12000);
moveArmAuton(-90);
chassis.turnToHeading(90, 1000);
chassis.moveToPoint(-46.223-13,-34.389-22, 1500, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(6);
    claw.move_voltage(12000);
pros::delay(1000);
//third pin
chassis.setPose(-62.253, -58.81, 90);

chassis.moveToPoint(-62.253+12, -58.81, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(0, 1000);
moveArmAuton(90);
setLiftTarget(140);
chassis.moveToPoint(-62.253+13, -58.81+27, 2000, {.maxSpeed=50, .minSpeed=30});
chassis.waitUntilDone();
setLiftTarget(100);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);

//return to loader
chassis.setPose(-47.223, -34.389, 0);
chassis.moveToPoint(-47.223, -34.389-22, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.waitUntil(4);
pros::Task r2Task([] {
    runR2Sequence();
});

pros::delay(2000);
moveArmAuton(-90);
chassis.turnToHeading(90, 1000);
chassis.moveToPoint(-46.223-13,-34.389-22.5, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
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
chassis.moveToPoint(-47.223, -34.389-22, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
chassis.turnToHeading(90, 1000);
chassis.moveToPoint(-46.223-14,-34.389-22.5, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=30});
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
setLiftTarget(90);
chassis.waitUntilDone();
setLiftTarget(60);
 pros::delay(1000);
   claw.move_voltage(-12000);
   pros::delay(1000);
   claw.move_voltage(0);
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
const double kAlliancePinIn[4] = {1.00, 10.32, 17.32, 23.32};

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
const uint32_t SEQ_LIFT_DELAY_MS = 350; // lift waits this long after the arm starts moving
const uint32_t SEQ_FLIP_DELAY_MS = 150; // after the lift reaches height, wait this long before the arm flips (was hardcoded)
const double   SEQ_FLIP_LIFT_TOL = 20.0;    // arm starts flipping once the lift is within this many degrees of its height (LIFT_SETTLE_TOL is too strict here)
const uint32_t SEQ_RAISE_TIMEOUT_MS = 1500; // flip anyway if the lift/arm have not got there after this long (was 4000)
const int32_t  SEQ_FLIP_VOLTAGE      = 12000; // arm voltage during the flip (full power, same as the other arm moves)
const int32_t  SEQ_FLIP_HOLD_VOLTAGE = 4000;  // arm keeps pressing down with this once it is flipped (same as auton hold)
const uint32_t SEQ_ARM_SETTLE_MS = 200; // after the arm reaches the flip position, wait this long before the lift lowers
const uint32_t SEQ_PRE_RELEASE_MS = 50; // after the lift lowers, wait at least this long before the claw lets go

// Reset. The arm flips back to straight up (the position it was raised to) while
// the lift starts descending. After SEQ_RESET_ARM_BACK_DELAY_MS the arm continues
// back to its resting position. The reset ends when the lift is down and the arm
// has stopped; the max is only a safety net.
const uint32_t SEQ_RESET_ARM_BACK_DELAY_MS = 500;   // arm stops at upright this long, then continues back
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
    liftLeft.set_current_limit(2500);
    liftRight.set_current_limit(2500);
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
                liftLeft.move_voltage(-8000);
                liftRight.move_voltage(-8000);
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