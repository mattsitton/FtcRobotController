
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Vision Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="FIXED: Vector FR Offset + Vision + Launcher", group="TeleOp")
public class VectorFROffsetVisionLauncherTeleOp_v2 extends OpMode {

    // ---------------- Drive Motors ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double flX = 0.15, flY = 0.15;
    private double frX = 0.15, frY = -0.075;
    private double blX = -0.15, blY = 0.15;
    private double brX = -0.15, brY = -0.15;

    // ---------------- Camera / Vision (MODERNIZED) ----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    // Your camera calibration values
    private double fx = 600.0, fy = 600.0;
    private double cx = 320.0, cy = 240.0;

    // ---------------- Launcher ----------------
    private DcMotor flywheel, feeder;
    private enum LauncherState { IDLE, SPIN_UP, FIRE, COOLDOWN }
    private LauncherState launcherState = LauncherState.IDLE;
    private long stateStartTime = 0;

    private final double FLYWHEEL_LOW = 0.67;
    private final double FLYWHEEL_HIGH = 1.0;
    private final long FEED_DURATION = 200; // ms

    @Override
    public void init() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Launcher
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        feeder.setDirection(DcMotor.Direction.FORWARD);

        // --- Vision Initialization (NEW CODE) ---
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS) // Match your original units
                .setLensIntrinsics(fx, fy, cx, cy) // Use your existing calibration
                // You can also add .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) or other families
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Vector FR Offset + Vision + Launcher Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---------------- Joystick Input ----------------
        double Vy = -gamepad1.left_stick_y;
        double Vx = gamepad1.left_stick_x * 1.1;
        double omega = gamepad1.right_stick_x;

        // ---------------- Drive Robot ----------------
        driveMecanumVectorOffset(Vx, Vy, omega);

        // ---------------- Vision Processing (NEW CODE) ----------------
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        double forwardDistance = -1, strafeOffset = -1, yaw = 0;

        if (!currentDetections.isEmpty()) {
            // Get the first detected tag
            AprilTagDetection tag = currentDetections.get(0);

            // Extract pose data from the tag.
            // ftcPose.y is the forward distance (range)
            // ftcPose.x is the strafe distance
            // ftcPose.yaw is the rotation of the tag relative to the camera
            forwardDistance = tag.ftcPose.y;
            strafeOffset = tag.ftcPose.x;
            yaw = tag.ftcPose.yaw; // This is yaw relative to the camera
        }

        // ---------------- Launcher State Machine (UNCHANGED) ----------------
        boolean spinTrigger = gamepad1.left_trigger > 0.8;
        boolean spinMedium = gamepad1.left_trigger > 0.4;
        boolean fireButton = gamepad1.x;
        long currentTime = System.currentTimeMillis();

        switch (launcherState) {
            case IDLE:
                flywheel.setPower(spinTrigger ? FLYWHEEL_HIGH : spinMedium ? FLYWHEEL_LOW : 0.0);
                feeder.setPower(0.0);
                if (fireButton) {
                    launcherState = LauncherState.SPIN_UP;
                    stateStartTime = currentTime;
                }
                break;
            case SPIN_UP:
                flywheel.setPower(FLYWHEEL_HIGH);
                feeder.setPower(0.0);
                if (currentTime - stateStartTime >= 300) {
                    launcherState = LauncherState.FIRE;
                    stateStartTime = currentTime;
                }
                break;
            case FIRE:
                feeder.setPower(1.0);
                if (currentTime - stateStartTime >= FEED_DURATION) {
                    launcherState = LauncherState.COOLDOWN;
                    stateStartTime = currentTime;
                    feeder.setPower(0.0);
                }
                break;
            case COOLDOWN:
                flywheel.setPower(FLYWHEEL_HIGH);
                if (currentTime - stateStartTime >= 200) {
                    launcherState = LauncherState.IDLE;
                }
                break;
        }

        // ---------------- Telemetry (UNCHANGED) ----------------
        telemetry.addLine("=== Joystick Input ===");
        telemetry.addData("Vx (Strafe)", "%.2f", Vx);
        telemetry.addData("Vy (Forward)", "%.2f", Vy);
        telemetry.addData("Omega (Rotation)", "%.2f", omega);

        telemetry.addLine("=== Motor Powers ===");
        telemetry.addData("FL", "%.2f", frontLeft.getPower());
        telemetry.addData("FR", "%.2f", frontRight.getPower());
        telemetry.addData("BL", "%.2f", backLeft.getPower());
        telemetry.addData("BR", "%.2f", backRight.getPower());

        telemetry.addLine("=== Vision ===");
        if(currentDetections.isEmpty()){
            telemetry.addLine("No tags detected.");
        } else {
            telemetry.addData("Tags Detected", currentDetections.size());
            telemetry.addData("Forward Distance (m)", "%.2f", forwardDistance);
            telemetry.addData("Strafe Offset (m)", "%.2f", strafeOffset);
            telemetry.addData("Yaw (rad)", "%.2f", yaw);
        }

        telemetry.addLine("=== Launcher ===");
        telemetry.addData("State", launcherState);
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Feeder Power", feeder.getPower());

        telemetry.update();
    }

    // --- Good practice to close the camera when the OpMode is stopped ---
    @Override
    public void stop() {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.close();
        }
    }

    // ---------------- Vector-Based Mecanum Drive (UNCHANGED) ----------------
    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) {
        double flRot = omega * (flX * Vy - flY * Vx);
        double frRot = omega * (frX * Vy - frY * Vx);
        double blRot = omega * (blX * Vy - blY * Vx);
        double brRot = omega * (brX * Vy - brY * Vx);

        double fl = Vy + Vx + flRot;
        double fr = Vy - Vx + frRot;
        double bl = Vy - Vx + blRot;
        double br = Vy + Vx + brRot;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}