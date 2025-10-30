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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver; // ADDED
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D; // ADDED
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // ADDED
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // ADDED

import java.util.List;

@TeleOp(name = "MasterTeleOp_VectorDrive_VisionOdometry", group = "TeleOp")
public class MasterTeleOp_VectorDrive_VisionOdometry extends OpMode {

    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Odometry Sensor ---
    private GoBildaPinpointDriver pinpoint = null; // ADDED

    // --- Mecanum Wheel Geometry (Removed, now using standard kinematics) ---
    // private final double flX = 0.15, flY = 0.15;
    // private final double frX = 0.15, frY = -0.075;
    // private final double blX = -0.15, blY = 0.15;
    // private final double brX = -0.15, brY = -0.15;

    // --- Launcher & Feeder ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private final double FEED_TIME_SECONDS = 0.20;
    private final double FULL_FEED_POWER = 1.0;
    private final double LOW_POWER = -.3;
    private final int HIGH_POWER = -1;
    private final int RPM_TOLERANCE = 100;
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10);

    // --- Enhancements ---
    private VoltageSensor batteryVoltageSensor;
    private final double NOMINAL_VOLTAGE = 12.0;

    // --- State Machines & Timers ---
    private final ElapsedTime feederTimer = new ElapsedTime();
    private enum LaunchState { IDLE, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;

    // --- Vision (AprilTag) ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    // [CUSTOMIZE] fx, fy, cx, cy must be calibrated values from your webcam
    private final double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;

    // --- Target Lock ---
    private boolean targetLockActive = false;
    private boolean targetLocked = false;
    private final double TARGET_LOCK_TOLERANCE_DEG = 2.0;
    private final double ROTATION_KP = 0.02;
    private final double ROTATION_MAX = 0.45;

    // --- State Variables ---
    private boolean flywheelAtSpeed = false;
    private int currentPower = 0;


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

        // --- Pinpoint Initialization ---
        try {
            // [CUSTOMIZE] Match this name to your hardware configuration
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint(); // ADDED
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
        boolean pressY = gamepad1.y;
        boolean pressX = gamepad1.x;
        boolean pressB = gamepad1.b;
        boolean pressA = gamepad1.a; // ADDED for Odometry Reset

        // --- Flywheel Control ---
        handleFlywheel(rightTrigger);

        // --- Vision Processing ---
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        boolean tagVisible = !detections.isEmpty();
        double tagYawDeg = 0.0, tagX = 0.0, tagY = 0.0;
        if (tagVisible) {
            AprilTagDetection tag = detections.get(0);
            tagYawDeg = tag.ftcPose.yaw;
            tagX = tag.ftcPose.x;
            tagY = tag.ftcPose.y;
            telemetry.addData("tagX",tagX);
            telemetry.addData("tagY",tagY);
            telemetry.addData("tagYawDeg",tagYawDeg);
        }

        // --- Cancel Overrides ---
        if (pressB) targetLockActive = false;

        // --- Auto-Drive and Target Lock Logic ---
        double forwardCmd = stickForward;
        double strafeCmd = stickStrafe;
        double rotateCmd = stickRotate;

        boolean isAutoDrive = leftTrigger > 0.05;
        targetLockActive = pressY && tagVisible;

        if (isAutoDrive && !targetLockActive) {
            double autoScale = 1.0 - leftTrigger;
            if (tagVisible) {
                double targetForwardDist = 1.2;
                double fError = tagY - targetForwardDist;
                double sError = tagX;
                forwardCmd = -fError * autoScale;
                strafeCmd = -sError * autoScale;
                rotateCmd = -Math.toRadians(tagYawDeg) * autoScale;
            }
            forwardCmd += stickForward * 0.5;
            strafeCmd += stickStrafe * 0.5;
            rotateCmd += stickRotate * 0.5;
        }

        if (targetLockActive) {
            double rotPower = -ROTATION_KP * tagYawDeg;
            rotPower = clamp(rotPower, -ROTATION_MAX, ROTATION_MAX);
            if (Math.abs(tagYawDeg) <= TARGET_LOCK_TOLERANCE_DEG) {
                rotateCmd = 0;
                if (!targetLocked) {
                    try { gamepad1.rumble(250); } catch (Exception ignored) {}
                    targetLocked = true;
                }
            } else {
                rotateCmd = rotPower;
                targetLocked = false;
            }
        } else {
            targetLocked = false;
        }

        // --- Apply Driver Enhancements ---
        double slowFactor = 1.0;
        // - (0.7 * leftTrigger)^
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        forwardCmd *= slowFactor * voltageComp;
        strafeCmd  *= slowFactor * voltageComp;
        rotateCmd  *= slowFactor * voltageComp;

        // --- Final Drive Command: Uses CORRECTED KINEMATICS ---
        driveMecanumVector(strafeCmd, forwardCmd, rotateCmd);

        // --- Launcher State Machine ---
        handleLauncherState(pressX, flywheelAtSpeed);

        // --- Odometry Update and Reset ---
        Pose2D currentPose = null; // ADDED
        if (pinpoint != null) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();

            // Odometry Reset on Gamepad A
            if (pressA) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
        }

        // --- Telemetry ---
        updateTelemetry(isAutoDrive, tagVisible, currentPose);
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
        max = Math.max(max, Math.abs(flPower));

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
        if (trigger > .9) currentPower = HIGH_POWER;
        else if (trigger > 0.1) currentPower = (int) LOW_POWER;
        else currentPower = 0;
        launcherMotor.setPower(currentPower);

        double currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());
        boolean atSpeedNow = (currentPower > 0) && (Math.abs(currentRPM - currentPower) <= RPM_TOLERANCE);
        if (atSpeedNow && !flywheelAtSpeed) {
            try { gamepad1.rumble(200); } catch (Exception ignored) {}
        }
        flywheelAtSpeed = atSpeedNow;
    }

    private void handleLauncherState(boolean xPressed, boolean isFlywheelReady) {
        if (launchState == LaunchState.IDLE && xPressed && isFlywheelReady ) {
            //launchState == LaunchState.IDLE && xPressed && isFlywheelReady ^
            feederTimer.reset();
            leftFeeder.setPower(FULL_FEED_POWER);
            rightFeeder.setPower(FULL_FEED_POWER);
            launchState = LaunchState.LAUNCHING;
        } else if (launchState == LaunchState.LAUNCHING && feederTimer.seconds() > FEED_TIME_SECONDS) {
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            launchState = LaunchState.IDLE;
        }
    }

    // ADDED Pinpoint Configuration
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
    // END ADDED Pinpoint Configuration

    private void updateTelemetry(boolean auto, boolean visible, Pose2D pose) {
        telemetry.addData("Mode", auto ? "AUTO" : (targetLockActive ? "TARGET-LOCK" : "MANUAL"));
        telemetry.addData("Tag Visible", visible);
        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addData("Launcher Target", "%d RPM", currentPower);
        telemetry.addData("Flywheel Ready", flywheelAtSpeed);


        // ADDED Odometry Telemetry
        if (pose != null) {
            telemetry.addLine("");
            telemetry.addData("ODOM X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("ODOM Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("ODOM Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("ODOM Reset", "Press A");
        }
        // END ADDED Odometry Telemetry

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