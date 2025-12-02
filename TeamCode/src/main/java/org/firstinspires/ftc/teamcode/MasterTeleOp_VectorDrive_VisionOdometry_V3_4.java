package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@TeleOp(name = "MasterTeleOp_VectorDrive_VisionOdometry_V3_4", group = "TeleOp")
public class MasterTeleOp_VectorDrive_VisionOdometry_V3_4 extends OpMode {
    // mm - millimeters
    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Odometry Sensor ---
    private GoBildaPinpointDriver pinpoint = null;

    // --- Encoder Constant ---
    // GoBilda 5203 (19.1:1) output shaft encoder ticks per revolution (534.8 PPR)
    private final double FLWHEEL_TICKS_PER_REV = 534.8;

    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private final double FEED_TIME_SECONDS = 0.20; // Time the feeder runs for
    private final double FULL_FEED_POWER = 1.0;
    private final int RPM_TOLERANCE = 100;

    // PIDF Coefficients (P=50 for stable PID control, F=12 for power baseline)
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(50, 0, 0, 12);

    // NEW: Realistic RPM targets (Max theoretical is 312 RPM)

    // --- Enhancements ---
    private VoltageSensor batteryVoltageSensor;
    private final double NOMINAL_VOLTAGE = 12.0;

    // --- Timed Launch Variables ---
    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean launchInProgress = false;

    // --- Vision (AprilTag) ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    // [CUSTOMIZE] fx, fy, cx, cy must be calibrated values from your webcam
    private final double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;

    // --- Target Lock / Vision Assist ---
    // Vision Mode Constants
    private final double ROTATION_KP = 0.1;
    private final double STRAFE_KP = 0.7; // Proportional gain for X-axis alignment
    private final double ALIGNMENT_TOLERANCE_MM = 50.0; // Tolerance for strafing lock (50 mm)

    // X-Axis Offset
    private final double ALIGNMENT_X_OFFSET_M = 0.0; // [CUSTOMIZE]

    // State Variables for Vision Modes
    private boolean yawAlignActive = false; // Y button: Yaw Alignment only (Hold)
    private boolean fullAlignActive = false;  // B button: Full Alignment (Yaw + Strafe) (Sticky/One-time)
    private boolean targetLocked = false;
    private boolean bButtonWasPressed = false; // Debounce variable for B button


    // --- State Variables ---
    private boolean flywheelAtSpeed = false;
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;


    @Override
    public void init() {
        // --- Hardware Mapping ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- Motor & Servo Configuration ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // --- Pinpoint Initialization (RESTORED) ---
        try {
            // [CUSTOMIZE] Match this name to your hardware configuration
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();
            telemetry.addData("Pinpoint Status", "Configuration Successful");
        } catch (Exception e) {
            telemetry.addData("Pinpoint Status", "ERROR: Check hardware map.");
            pinpoint = null;
        }

        // --- Vision Initialization ---
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Master TeleOp (Vision+Odom) Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Read Gamepad Inputs ---
        double stickForward = gamepad1.left_stick_y;
        double stickRotate = gamepad1.right_stick_x;

        // Drive/Launcher Triggers
        double slowTrigger = gamepad1.right_trigger;
        double manualFlywheelPower = gamepad1.left_trigger;

        // Buttons
        boolean pressX = gamepad1.x;
        boolean pressB = gamepad1.b; // Full Alignment Mode (One-time press)
        boolean pressA = gamepad1.a;
        boolean pressY = gamepad1.y; // Yaw Alignment Mode (Hold)

        // Dpad inputs for RPM control
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;

        // --- Calculate Voltage Compensation Factor ---
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        // --- Flywheel Control (PASSING VOLTAGE COMP) ---
        handleFlywheel(dpadUp, dpadRight, dpadDown, dpadLeft, manualFlywheelPower, voltageComp);

        // --- Odometry Update and Reset (RESTORED) ---
        Pose2D currentPose = null;
        double odomHeadingDeg = 0.0;
        if (pinpoint != null) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            odomHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

            // Odometry Reset on Gamepad A
            if (pressA) {
                // Sets position to (0, 0, 0) in inches and degrees
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
        }

        // --- Vision Processing ---
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        boolean tagVisible = !detections.isEmpty();
        double tagYawDeg = 0.0, tagX = 0.0, tagY = 0.0;
        if (tagVisible) {
            AprilTagDetection tag = detections.get(0);
            tagYawDeg = tag.ftcPose.yaw;
            tagX = tag.ftcPose.x; // Sideways distance error (meters)
            tagY = tag.ftcPose.y; // Forward distance (meters)
        }

        // --- Vision Mode State Logic ---
        yawAlignActive = pressY;

        if (pressB && !bButtonWasPressed && tagVisible) {
            fullAlignActive = true;
        }
        bButtonWasPressed = pressB;

        // --- Drive Command Variables ---
        double forwardCmd = stickForward;
        double strafeCmd = stickStrafe;
        double rotateCmd = stickRotate;
        targetLocked = false;


        if (yawAlignActive) {
            if(fullAlignActive) { /* Do nothing, fullAlignActive takes precedence */ }

            else if (tagVisible) {
                // 1. ANGLE ALIGNMENT (Rotation)
                double rotPower = -ROTATION_KP * tagYawDeg;
                rotateCmd = clamp(rotPower, -ROTATION_MAX, ROTATION_MAX);

                // 2. OTHER AXES (Stays manual)
                strafeCmd = stickStrafe;
                forwardCmd = stickForward;

                // 3. LOCK DETECTION (Only for Yaw)
                boolean isAngleAligned = Math.abs(tagYawDeg) <= ANGLE_TOLERANCE_DEG;

                if (isAngleAligned) {
                    targetLocked = true;
                    rotateCmd = 0;
                }
            } else { // No tag visible: Use Odometry to maintain current heading
                if (pinpoint != null && Math.abs(stickRotate) < 0.1) {
                    rotateCmd = 0; // Hold heading
                }
                strafeCmd = stickStrafe;
                forwardCmd = stickForward;
            }

        } else if (fullAlignActive) { // --- B BUTTON: FULL ALIGNMENT MODE (Yaw + Strafe - One-time) ---

            if (tagVisible) {
                // 1. ANGLE ALIGNMENT (Rotation)
                double rotPower = -ROTATION_KP * tagYawDeg;
                rotateCmd = clamp(rotPower, -ROTATION_MAX, ROTATION_MAX);

                // 2. X-AXIS ALIGNMENT (Strafe)
                double strafeError_M = tagX - ALIGNMENT_X_OFFSET_M;
                double strafePower = -STRAFE_KP * strafeError_M;
                strafeCmd = clamp(strafePower, -STRAFE_MAX, STRAFE_MAX);

                // 3. FORWARD MOVEMENT (Stays manual)
                forwardCmd = stickForward;

                // 4. LOCK DETECTION
                boolean isAngleAligned = Math.abs(tagYawDeg) <= ANGLE_TOLERANCE_DEG;
                boolean isStrafeAligned = Math.abs(strafeError_M * 1000.0) <= ALIGNMENT_TOLERANCE_MM;

                if (isAngleAligned && isStrafeAligned) {
                    targetLocked = true;
                    // Alignment complete: Clear the sticky state
                    fullAlignActive = false;
                    try { gamepad1.rumble(250); } catch (Exception ignored) {} // RUMBLE on successful lock
                }

                // If aligned, stop the active corrections
                if (isAngleAligned) rotateCmd = 0;
                if (isStrafeAligned) strafeCmd = 0;

            } else { // Tag lost during alignment: Stop autonomous motion and clear the sticky state
                fullAlignActive = false;
                strafeCmd = stickStrafe;
                rotateCmd = stickRotate;
            }

            // Allow manual stick input to immediately cancel the sticky state
            if (Math.abs(stickForward) > 0.1 || Math.abs(stickStrafe) > 0.1 || Math.abs(stickRotate) > 0.1) {
                fullAlignActive = false;
            }
        }

        // --- Apply Drive Enhancements ---
        double slowFactor = 1.0 - (0.7 * slowTrigger);

        forwardCmd *= slowFactor;
        strafeCmd  *= slowFactor;
        rotateCmd  *= slowFactor;

        // --- Final Drive Command: Uses CORRECTED KINEMATICS ---
        driveMecanumVector(strafeCmd, forwardCmd, rotateCmd, voltageComp);

        // --- TIMED FEEDER CONTROL ---
        handleTimedFeeder(pressX);

        // --- Telemetry ---
        updateTelemetry(yawAlignActive, fullAlignActive, tagVisible, tagX, tagY, tagYawDeg, currentPose, manualFlywheelPower);
    }

    // Renamed and Corrected Kinematic Method
    private void driveMecanumVector(double Vx, double Vy, double omega, double voltageComp) {
        // CORRECTED KINEMATICS (standard Mecanum drive)
        double flPower = Vy + Vx + omega;
        double frPower = Vy - Vx - omega;
        double blPower = Vy - Vx + omega;
        double brPower = Vy + Vx - omega;

        // Normalization (find max and scale down to keep proportions)
        double max = Math.abs(flPower);
        max = Math.max(max, Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        // Apply voltage compensation *after* normalization
        flPower *= voltageComp;
        flPower = clamp(flPower, -1.0, 1.0);
        frPower *= voltageComp;
        frPower = clamp(frPower, -1.0, 1.0);
        blPower *= voltageComp;
        blPower = clamp(blPower, -1.0, 1.0);
        brPower *= voltageComp;
        brPower = clamp(brPower, -1.0, 1.0);


        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private void handleTimedFeeder(boolean xPressed) {
        if (launchInProgress) {
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launchInProgress = false;
            }
        } else if (xPressed) {
            feederTimer.reset();
            leftFeeder.setPower(FULL_FEED_POWER);
            rightFeeder.setPower(FULL_FEED_POWER);
            launchInProgress = true;
        }
    }

    // UPDATED: Now uses the correct encoder constant (FLWHEEL_TICKS_PER_REV)
    private void handleFlywheel(boolean dpadUp, boolean dpadRight, boolean dpadDown, boolean dpadLeft, double manualPower, double voltageComp) {

        if (manualPower > 0.1) {
            // MANUAL OVERRIDE (Left Trigger)
            // This mode bypasses PID control to allow max power output (compensated).
            // **FIX:** Scale the manualPower input so max power is reached easily (e.g., at 80% pull).
            double scaledInput = Range.clip(manualPower / 0.8, 0.0, 1.0);

            currentTargetRPM = 0;
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Apply voltage compensation and use the scaled input
            double compensatedPower = Range.clip(scaledInput * voltageComp, 0.0, 1.0);
            launcherMotor.setPower(compensatedPower);
            currentFlywheelRPM = 0.0;
            flywheelAtSpeed = false;
            return;
        }

        // D-PAD RPM CONTROL (PID)
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (dpadUp) currentTargetRPM = RPM_TARGET_UP;
        else if (dpadRight) currentTargetRPM = RPM_TARGET_RIGHT;
        else if (dpadDown) currentTargetRPM = RPM_TARGET_DOWN;
        else if (dpadLeft) currentTargetRPM = RPM_TARGET_LEFT;
        else currentTargetRPM = 0;

        // SET VELOCITY using corrected conversion
        launcherMotor.setVelocity(rpmToTicksPerSecond(currentTargetRPM));

        // --- PID TUNING INSTRUCTIONS ---
        // P-Gain is the proportional response. F-Gain is the baseline power.
        // F-Gain (12) should remain fixed as it matches the motor's theoretical constant.
        // P-Gain (50) controls stability and response time.
        // 1. If motor STILL OSCILLATES (jerks): REDUCE the P-gain in LAUNCHER_PIDF (e.g., from 50 to 10).
        //    The P-gain must be very small for stable velocity control.
        // 2. If motor is SLOW to reach speed: Slightly INCREASE the P-gain (e.g., from 50 to 60) until stable/fastest response is achieved.

        double currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());
        currentFlywheelRPM = currentRPM;

        boolean atSpeedNow = (currentTargetRPM > 0) && (Math.abs(currentRPM - currentTargetRPM) <= RPM_TOLERANCE);
        flywheelAtSpeed = atSpeedNow;
    }

    // configurePinpoint method RESTORED
    private void configurePinpoint() {
        // [CUSTOMIZE] Measure these values on your robot!
        // X-Offset: Distance sideways from the tracking point (Left +, Right -)
        // Y-Offset: Distance forwards from the tracking point (Forward +, Backward -)
        final double POD_X_OFFSET_MM = 0.0; // [CUSTOMIZE]
        final double POD_Y_OFFSET_MM = 0.0; // [CUSTOMIZE]

        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);

        // Correct for 4-Bar Odometry Pods (32mm)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // [CUSTOMIZE] Verify these directions by checking encoder counts manually.
        final GoBildaPinpointDriver.EncoderDirection X_POD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        final GoBildaPinpointDriver.EncoderDirection Y_POD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        pinpoint.setEncoderDirections(X_POD_DIR, Y_POD_DIR);

        // Resets position to (0, 0, 0) and recalibrates IMU
        pinpoint.resetPosAndIMU();
    }
    // END configurePinpoint

    private void updateTelemetry(boolean yawActive, boolean fullActive, boolean visible, double tagX, double tagY, double tagYawDeg, Pose2D pose, double manualFlywheelPower) {
        String launcherStatus = currentTargetRPM > 0 ? "Target: " + currentTargetRPM + " RPM" :
                manualFlywheelPower > 0.1 ? "Manual Power: " + String.format("%.2f", manualFlywheelPower) : "OFF";

        String driveMode;
        if (fullActive) driveMode = targetLocked ? "AUTOCENTER COMPLETE" : "AUTOCENTER ACTIVE (Cancel: Drive Stick)";
        else if (yawActive) driveMode = targetLocked ? "YAW ALIGNED (HOLD)" : "YAW ALIGN (Y HOLD)";
        else driveMode = "MANUAL";

        telemetry.addData("Mode", driveMode);
        telemetry.addData("Tag Visible", visible);

        // CAMERA/APRILTAG TELEMETRY
        telemetry.addLine("");
        telemetry.addData("CAMERA STATUS", visionPortal.getCameraState());
        if (visible) {
            telemetry.addData("TAG X (Strafe Err)", "%.2f M", tagX - ALIGNMENT_X_OFFSET_M);
            telemetry.addData("TAG Y (Forward Dist)", "%.2f M", tagY);
            telemetry.addData("TAG Angle (Yaw)", "%.1f Deg", tagYawDeg);
        }
        telemetry.addData("X-Axis Offset", "%.2f M", ALIGNMENT_X_OFFSET_M);

        telemetry.addLine("-----");
        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addData("Flywheel Target", launcherStatus);
        telemetry.addData("D-Pad Map", "U:%d R:%d D:%d L:%d", RPM_TARGET_UP, RPM_TARGET_RIGHT, RPM_TARGET_DOWN, RPM_TARGET_LEFT);
        telemetry.addData("Launcher Current", "%.0f RPM", currentFlywheelRPM);
        telemetry.addData("Flywheel Ready (PID)", flywheelAtSpeed);
        telemetry.addData("Vision Yaw Align", "HOLD Y (Yaw Only)");
        telemetry.addData("Vision Auto Center", "PRESS B (Strafe+Yaw)");
        telemetry.addData("Feeder Control", "Press X for Timed Feed (%.2fs)", FEED_TIME_SECONDS);
        telemetry.addData("Manual Speed", "Left Trigger (Scaled)");

        // ODOMETRY TELEMETRY
        if (pose != null) {
            telemetry.addLine("");
            telemetry.addData("ODOM X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("ODOM Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("ODOM Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("ODOM Reset", "Press A");
        }

        telemetry.update();
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * FLWHEEL_TICKS_PER_REV;
    }

    private double ticksPerSecondToRPM(double ticksPerSec) {
        return (ticksPerSec / FLWHEEL_TICKS_PER_REV) * 60.0;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        launcherMotor.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Safely set drive motors to zero power on stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}