// File: MasterTeleOp_Vision.txt
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

import java.util.List;

@TeleOp(name = "MasterTeleOp_Vision", group = "TeleOp")
public class MasterTeleOp_VectorDrive_Vision extends OpMode {

    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Mecanum Wheel Geometry (in meters from robot center) ---
    private final double flX = 0.15, flY = 0.15;
    private final double frX = 0.15, frY = -0.075; // Front-right wheel is offset halfway back
    private final double blX = -0.15, blY = 0.15;
    private final double brX = -0.15, brY = -0.15;

    // --- Launcher & Feeder ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private final double FEED_TIME_SECONDS = 0.20;
    private final double FULL_FEED_POWER = 1.0;
    private final int LOW_RPM = 2500;
    private final int HIGH_RPM = 3500;
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
    private final double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;

    // --- Target Lock ---
    private boolean targetLockActive = false;
    private boolean targetLocked = false;
    private final double TARGET_LOCK_TOLERANCE_DEG = 2.0;
    private final double ROTATION_KP = 0.02;
    private final double ROTATION_MAX = 0.45;

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

        // --- Vision Initialization ---
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Master TeleOp (Vision) Initialized.");
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
            forwardCmd += stickForward * 0.2;
            strafeCmd += stickStrafe * 0.2;
            rotateCmd += stickRotate * 0.2;
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
        double slowFactor = 1.0 - (0.7 * leftTrigger);
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        forwardCmd *= slowFactor * voltageComp;
        strafeCmd  *= slowFactor * voltageComp;
        rotateCmd  *= slowFactor * voltageComp;

        // --- Final Drive Command ---
        driveMecanumVectorOffset(strafeCmd, forwardCmd, rotateCmd);

        // --- Launcher State Machine ---
        handleLauncherState(pressX, flywheelAtSpeed);

        // --- Telemetry ---
        updateTelemetry(isAutoDrive, tagVisible);
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

    private void handleLauncherState(boolean xPressed, boolean isFlywheelReady) {
        if (launchState == LaunchState.IDLE && xPressed && isFlywheelReady) {
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

    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) {
        double flPower = Vy + Vx - omega * flY;
        double frPower = Vy - Vx - omega * frY;
        double blPower = Vy - Vx - omega * blY;
        double brPower = Vy + Vx - omega * brY;

        double max = Math.max(1.0, Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));
        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backLeft.setPower(blPower / max);
        backRight.setPower(brPower / max);
    }

    private void updateTelemetry(boolean auto, boolean visible) {
        telemetry.addData("Mode", auto ? "AUTO" : (targetLockActive ? "TARGET-LOCK" : "MANUAL"));
        telemetry.addData("Tag Visible", visible);
        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addData("Launcher Target", "%d RPM", currentTargetRPM);
        telemetry.addData("Flywheel Ready", flywheelAtSpeed);
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
    }
}