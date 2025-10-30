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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver; // RESTORED
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D; // RESTORED
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "MasterTeleOp_VectorDrive_VisionOdometry_V2", group = "TeleOp")
public class MasterTeleOp_VectorDrive_VisionOdometry_V2 extends OpMode { // NAME RESTORED

    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Odometry Sensor ---
    private GoBildaPinpointDriver pinpoint = null; // RESTORED

    // --- Launcher & Feeder ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private final double FEED_TIME_SECONDS = 0.20; // Time the feeder runs for
    private final double FULL_FEED_POWER = 1.0;
    private final int LOW_RPM = 2500;
    private final int HIGH_RPM = 3500;
    private final int RPM_TOLERANCE = 100;
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10);

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
    private boolean visionAssistActive = false; // Unified flag for Vision Assist
    private boolean targetLocked = false;
    private final double ROTATION_KP = 0.1;
    private final double ROTATION_MAX = 0.45;
    private final double STRAFE_KP = 0.7; // Proportional gain for X-axis alignment
    private final double STRAFE_MAX = 0.5; // Maximum strafing power
    private final double ALIGNMENT_TOLERANCE_MM = 50.0; // Tolerance for strafing lock (50 mm)
    private final double ANGLE_TOLERANCE_DEG = 5.0; // Tolerance for angle lock

    // X-Axis Offset
    private final double ALIGNMENT_X_OFFSET_M = 0.0; // [CUSTOMIZE]

    // --- State Variables ---
    private boolean flywheelAtSpeed = false;
    private int currentTargetRPM = 0;


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
        double stickForward = -gamepad1.left_stick_y;
        double stickStrafe = gamepad1.left_stick_x;
        double stickRotate = gamepad1.right_stick_x;
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;
        boolean pressX = gamepad1.x;
        boolean pressB = gamepad1.b;
        boolean pressA = gamepad1.a; // RESTORED (Odometry Reset)

        // --- Flywheel Control ---
        handleFlywheel(rightTrigger);

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

        // --- Cancel Overrides ---
        if (pressB) visionAssistActive = false;

        // --- Unified Vision Assist Logic ---
        double forwardCmd = stickForward;
        double strafeCmd = stickStrafe;
        double rotateCmd = stickRotate;

        // Activation: DPAD UP and Tag is visible
        visionAssistActive = gamepad1.dpad_up && tagVisible;

        if (visionAssistActive) {
            // 1. ANGLE ALIGNMENT (Rotation) - Force square alignment
            double rotPower = -ROTATION_KP * tagYawDeg;
            rotateCmd = clamp(rotPower, -ROTATION_MAX, ROTATION_MAX);

            // 2. X-AXIS ALIGNMENT (Strafe) - Force center alignment with offset
            double strafeError_M = tagX - ALIGNMENT_X_OFFSET_M;
            double strafePower = -STRAFE_KP * strafeError_M;
            strafeCmd = clamp(strafePower, -STRAFE_MAX, STRAFE_MAX);

            // 3. FORWARD MOVEMENT - Stays manual, but applies slow factor

            // 4. LOCK DETECTION
            boolean isAngleAligned = Math.abs(tagYawDeg) <= ANGLE_TOLERANCE_DEG;
            // The alignment check uses the modified error, converted to millimeters
            boolean isStrafeAligned = Math.abs(strafeError_M * 1000.0) <= ALIGNMENT_TOLERANCE_MM;

            if (isAngleAligned && isStrafeAligned && !targetLocked) {
                try { gamepad1.rumble(250); } catch (Exception ignored) {}
                targetLocked = true;
            } else if (!isAngleAligned || !isStrafeAligned) {
                targetLocked = false;
            }

            // If aligned, stop the active corrections
            if (isAngleAligned) rotateCmd = 0;
            if (isStrafeAligned) strafeCmd = 0;

        } else {
            targetLocked = false;
        }

        // --- Apply Driver Enhancements ---
        double slowFactor = 1.0 - (0.7 * leftTrigger);
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        forwardCmd *= slowFactor * voltageComp;
        strafeCmd  *= slowFactor * voltageComp;
        rotateCmd  *= slowFactor * voltageComp;

        // --- Final Drive Command: Uses CORRECTED KINEMATICS ---
        driveMecanumVector(strafeCmd, forwardCmd, rotateCmd);

        // --- TIMED FEEDER CONTROL ---
        handleTimedFeeder(pressX);

        // --- Odometry Update and Reset (RESTORED) ---
        Pose2D currentPose = null;
        if (pinpoint != null) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();

            // Odometry Reset on Gamepad A
            if (pressA) {
                // Sets position to (0, 0, 0) in inches and degrees
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
        }

        // --- Telemetry ---
        updateTelemetry(visionAssistActive, tagVisible, tagX, tagY, tagYawDeg, currentPose);
    }

    private void handleTimedFeeder(boolean xPressed) {
        if (launchInProgress) {
            // STEP 2: If a launch is in progress, check the timer
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launchInProgress = false; // Stop the launch
            }
        } else if (xPressed) { // Unconditional launch on X press
            // STEP 1: If X is pressed, START the launch regardless of flywheel speed
            feederTimer.reset();
            leftFeeder.setPower(FULL_FEED_POWER);
            rightFeeder.setPower(FULL_FEED_POWER);
            launchInProgress = true;
        }
    }

    // Renamed and Corrected Kinematic Method
    private void driveMecanumVector(double Vx, double Vy, double omega) {
        // CORRECTED KINEMATICS (standard Mecanum drive)
        double flPower = Vy + Vx + omega; // Front Left: Vy + Vx + Omega
        double frPower = Vy - Vx - omega; // Front Right: Vy - Vx - Omega
        double blPower = Vy - Vx + omega; // Back Left: Vy - Vx + Omega
        double brPower = Vy + Vx - omega; // Back Right: Vy + Vx - Omega

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

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private void handleFlywheel(double trigger) {
        if (trigger > 0.8) currentTargetRPM = HIGH_RPM;
        else if (trigger > 0.3) currentTargetRPM = LOW_RPM;
        else currentTargetRPM = 0;
        launcherMotor.setVelocity(rpmToTicksPerSecond(currentTargetRPM));

        double currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());
        boolean atSpeedNow = (currentTargetRPM > 0) && (Math.abs(currentRPM - currentTargetRPM) <= RPM_TOLERANCE);
        if (atSpeedNow && !flywheelAtSpeed) {
            try { gamepad1.rumble(200); } catch (Exception ignored) {}
        }
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

    // MODIFIED: Accepts Pose2D object again
    private void updateTelemetry(boolean assistActive, boolean visible, double tagX, double tagY, double tagYawDeg, Pose2D pose) {
        telemetry.addData("Mode", assistActive ? (targetLocked ? "VISION ALIGNED" : "VISION ASSIST") : "MANUAL");
        telemetry.addData("Tag Visible", visible);

        // CAMERA/APRILTAG TELEMETRY
        telemetry.addLine("");
        telemetry.addData("CAMERA STATUS", visionPortal.getCameraState());
        if (visible) {
            telemetry.addData("TAG X (Strafe)", "%.2f M", tagX);
            telemetry.addData("TAG Y (Forward)", "%.2f M", tagY);
            telemetry.addData("TAG Angle (Yaw)", "%.1f Deg", tagYawDeg);
        }
        telemetry.addData("X-Axis Offset", "%.2f M", ALIGNMENT_X_OFFSET_M);

        telemetry.addLine("-----");
        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addData("Launcher Target", "%d RPM", currentTargetRPM);
        telemetry.addData("Flywheel Ready", flywheelAtSpeed);
        telemetry.addData("Vision Assist", "DPAD UP (Auto Strafe+Angle)");
        telemetry.addData("Feeder Control", "Press X for Timed Feed (%.2fs)", FEED_TIME_SECONDS);

        // ODOMETRY TELEMETRY (RESTORED)
        if (pose != null) {
            telemetry.addLine("");
            telemetry.addData("ODOM X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("ODOM Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("ODOM Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("ODOM Reset", "Press A");
        }
        // END ODOMETRY TELEMETRY

        telemetry.update();
    }

    private double rpmToTicksPerSecond(double rpm) { return (rpm / 60.0) * 28.0; }
    private double ticksPerSecondToRPM(double ticksPerSec) { return (ticksPerSec / 28.0) * 60.0; }
    private double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

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