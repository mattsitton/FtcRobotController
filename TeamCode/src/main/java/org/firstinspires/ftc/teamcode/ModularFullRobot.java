
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

// Vision Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * ====================================================================================================
 * This TeleOp code has been refactored to separate the vision processing from the
 * auto-drive control logic. Each major subsystem is now handled in its own method.
 * ====================================================================================================
 */
@TeleOp(name="MODULAR: Full Robot", group="TeleOp")
public class ModularFullRobot extends OpMode {

    // ---------------- Hardware & Control State Variables ----------------
    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor flywheel, feeder;
    /* private CRServo intakeLeft, intakeRight; */

    // Control State
    private enum LauncherState { IDLE, SPIN_UP, FIRE, COOLDOWN }
    private LauncherState launcherState = LauncherState.IDLE;
    private long stateStartTime = 0;
    /*
    private boolean intakeOn = false;
    private boolean lastB = false;
    */

    // ---------------- Vision & Auto-Drive Variables ----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // These variables are updated by handleVision() and used by handleDriving()
    private double forwardDistance = -1, strafeOffset = -1, yaw = 0;

    // Constants
    private double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
    private double targetForward = 1.2, targetStrafe = 0.0;
    private double tolerance = 0.05, rotationTolerance = 0.05;
    private double minForward = 1.0, maxForward = 1.5, minStrafe = -0.2, maxStrafe = 0.2;
    private double flX = 0.15, flY = 0.15, frX = 0.15, frY = -0.075;
    private double blX = -0.15, blY = 0.15, brX = -0.15, brY = -0.15;


    @Override
    public void init() {
        // Initialize all hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        feeder.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Vision System
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Initialization Complete.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // The main loop is now a clean, high-level overview of the robot's tasks.
        handleVision();
        handleDriving();
        // handleLauncher(); // Add this back if you implement the launcher
        // handleIntake();   // Add this back if you enable the intake
        updateTelemetry();
    }

    /**
     * Handles all vision processing. It gets AprilTag detections and updates
     * the robot's estimated position (forwardDistance, strafeOffset, yaw).
     */
    private void handleVision() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        // Default to -1 to indicate no tag is seen
        forwardDistance = -1;
        strafeOffset = -1;
        yaw = 0;

        if (!currentDetections.isEmpty()) {
            AprilTagDetection tag = currentDetections.get(0);
            forwardDistance = tag.ftcPose.y;
            strafeOffset = tag.ftcPose.x;
            yaw = tag.ftcPose.yaw;
        }
    }

    /**
     * Handles all drive logic. It checks for driver input and the robot's position
     * to decide whether to use manual control or engage the auto-drive feature.
     */
    private void handleDriving() {
        double rTrig = gamepad1.right_trigger;
        boolean inShootingZone = (forwardDistance > 0) &&
                (forwardDistance >= minForward && forwardDistance <= maxForward) &&
                (strafeOffset >= minStrafe && strafeOffset <= maxStrafe);

        if (rTrig > 0.1 && inShootingZone) {
            // --- AUTO-DRIVE MODE ---
            double nudgeScale = 0.05;
            double nudgeForward = -gamepad1.left_stick_y * nudgeScale;
            double nudgeStrafe = gamepad1.left_stick_x * nudgeScale;

            double fError = forwardDistance - targetForward + nudgeForward;
            double sError = strafeOffset - targetStrafe + nudgeStrafe;
            double rError = yaw;

            double kP = 1.0;
            double kPR = 1.5;

            double forwardPower = -fError * kP;
            double strafePower = -sError * kP;
            double rotatePower = -rError * kPR;

            double speedScale = 1.1 - rTrig;

            forwardPower = Math.max(-speedScale, Math.min(speedScale, forwardPower));
            strafePower = Math.max(-speedScale, Math.min(speedScale, strafePower));
            rotatePower = Math.max(-0.4, Math.min(0.4, rotatePower));

            if (Math.abs(fError) < tolerance && Math.abs(sError) < tolerance && Math.abs(rError) < rotationTolerance) {
                stopDrive();
            } else {
                driveMecanumVector(strafePower, forwardPower, rotatePower);
            }
        } else {
            // --- MANUAL DRIVE MODE ---
            double Vy = -gamepad1.left_stick_y;
            double Vx = gamepad1.left_stick_x * 1.1;
            double omega = gamepad1.right_stick_x;
            driveMecanumVectorOffset(Vx, Vy, omega);
        }
    }

    /**
     * Handles updating and sending all telemetry data to the Driver Station.
     */
    private void updateTelemetry() {
        boolean inShootingZone = (forwardDistance > 0) &&
                (forwardDistance >= minForward && forwardDistance <= maxForward) &&
                (strafeOffset >= minStrafe && strafeOffset <= maxStrafe);

        telemetry.addData("--- Drive Mode ---", (gamepad1.right_trigger > 0.1 && inShootingZone) ? "AUTO-DRIVE (INVERTED)" : "MANUAL");
        if (gamepad1.right_trigger > 0.1 && inShootingZone) {
            telemetry.addData("Auto-Drive Speed", "%.0f %%", ((1.1 - gamepad1.right_trigger) * 100));
        }
        telemetry.addData("In Shooting Zone", inShootingZone);
        telemetry.addData("Tag Visible", forwardDistance > 0);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.close();
        }
    }

    // =================================================================
    //                    MECANUM DRIVE METHODS
    // =================================================================
    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) { /* ...unchanged... */ }
    private void driveMecanumVector(double Vx, double Vy, double omega) { /* ...unchanged... */ }
    private void stopDrive() { /* ...unchanged... */ }
}