
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetectionPipeline;
import java.util.ArrayList;

@TeleOp(name="Vector FR Offset + Vision + Launcher", group="TeleOp")
public class VectorFROffsetVisionLauncherTeleOp extends OpMode {

    // ---------------- Hardware ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Wheel coordinates relative to robot center (meters)
    private double flX = 0.15, flY = 0.15;
    private double frX = 0.15, frY = -0.075;
    private double blX = -0.15, blY = 0.15;
    private double brX = -0.15, brY = -0.15;

    // ---------------- Camera / Vision ----------------
    private OpenCvWebcam webcam;
    private AprilTagDetectionPipeline aprilTagPipeline;
    private double fx = 600.0, fy = 600.0;  // Logitech C270 approx intrinsics
    private double cx = 320.0, cy = 240.0;
    private double tagsize = 0.166; // meters

    // ---------------- Launcher ----------------
    private DcMotor flywheel;
    private DcMotor feeder;

    private enum LauncherState { IDLE, SPIN_UP, FIRE, COOLDOWN }
    private LauncherState launcherState = LauncherState.IDLE;
    private long stateStartTime = 0;

    private final double FLYWHEEL_LOW = 0.67;
    private final double FLYWHEEL_HIGH = 1.0;
    private final long FEED_DURATION = 200; // ms

    @Override
    public void init() {
        // ---------------- Hardware Mapping ----------------
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        feeder.setDirection(DcMotor.Direction.FORWARD);

        // ---------------- Camera Initialization ----------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId);

        aprilTagPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(aprilTagPipeline);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT));

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

        // ---------------- Vision Processing ----------------
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        double forwardDistance = -1, strafeOffset = -1, yaw = 0;

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            forwardDistance = tag.pose.z;
            strafeOffset = tag.pose.x;
            yaw = tag.pose.yaw;
        }

        // ---------------- Launcher State Machine ----------------
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

        // ---------------- Telemetry ----------------
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
        telemetry.addData("Forward Distance (m)", "%.2f", forwardDistance);
        telemetry.addData("Strafe Offset (m)", "%.2f", strafeOffset);
        telemetry.addData("Yaw (rad)", "%.2f", yaw);

        telemetry.addLine("=== Launcher ===");
        telemetry.addData("State", launcherState);
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Feeder Power", feeder.getPower());

        telemetry.update();
    }

    // ---------------- Vector-Based Mecanum Drive with FR Offset ----------------
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
