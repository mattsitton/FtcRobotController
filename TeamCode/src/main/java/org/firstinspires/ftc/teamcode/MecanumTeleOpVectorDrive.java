package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetectionPipeline;

import java.util.ArrayList;
@Disabled
@TeleOp(name="Mecanum TeleOp Full + Vector Drive + AutoFire", group="TeleOp")
public class MecanumTeleOpVectorDrive extends OpMode {

    // ---------------- Hardware ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private CRServo intakeLeft, intakeRight;
    private DcMotor flywheel;
    private Servo gateServo;

    private boolean intakeOn = false;
    private boolean lastB = false;

    // ---------------- Vision ----------------
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272, fy = 578.272;
    double cx = 402.145, cy = 221.506;
    double tagsize = 0.166; // meters

    // ---------------- Auto-shoot targets ----------------
    double targetForward = 1.2;
    double targetStrafe = 0.0;
    double tolerance = 0.05;
    double rotationTolerance = 0.05;

    double minForward = 1.0;
    double maxForward = 1.5;
    double minStrafe = -0.2;
    double maxStrafe = 0.2;

    double maxAutoSpeed = 0.5;

    // ---------------- Auto-fire ----------------
    private boolean firing = false;
    private boolean lastA = false;
    private ElapsedTime fireTimer = new ElapsedTime();
    private double fireDuration = 0.5; // seconds gate stays open

    // ---------------- Wheel geometry ----------------
    private double flX = 0.15, flY = 0.15;      // front-left
    private double frX = 0.15, frY = -0.075;    // front-right offset halfway back
    private double blX = -0.15, blY = 0.15;     // back-left
    private double brX = -0.15, brY = -0.15;    // back-right

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRight.setDirection(CRServo.Direction.REVERSE);

        gateServo.setPosition(0.0); // start closed

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT));

        telemetry.addLine("Initialized — Ready!");
    }

    @Override
    public void loop() {
        // ---------------- Intake ----------------
        if (gamepad1.b && !lastB) intakeOn = !intakeOn;
        lastB = gamepad1.b;
        intakeLeft.setPower(intakeOn ? 1.0 : 0.0);
        intakeRight.setPower(intakeOn ? 1.0 : 0.0);

        // ---------------- Flywheel ----------------
        double trigger = gamepad1.left_trigger;
        if (trigger > 0.8) flywheel.setPower(1.0);
        else if (trigger > 0.4) flywheel.setPower(0.67);
        else flywheel.setPower(0.0);

        // ---------------- Gate manual control ----------------
        if (gamepad1.x) gateServo.setPosition(0.0);
        if (gamepad1.y) gateServo.setPosition(1.0);

        // ---------------- Vision ----------------
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
        double forwardDist = -1;
        double strafeOffset = -1;
        double currentYaw = 0.0;

        if (detections != null && detections.size() > 0) {
            AprilTagDetection tag = detections.get(0);
            forwardDist = tag.pose.z;
            strafeOffset = tag.pose.x;
            currentYaw = tag.pose.yaw;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Forward Dist (m)", "%.2f", forwardDist);
            telemetry.addData("Strafe Offset (m)", "%.2f", strafeOffset);
            telemetry.addData("Yaw (rad)", "%.2f", currentYaw);
        } else telemetry.addLine("No tags detected");

        // ---------------- Adjust max auto speed ----------------
        if (gamepad1.right_bumper) maxAutoSpeed += 0.01;
        if (gamepad1.left_bumper) maxAutoSpeed -= 0.01;
        maxAutoSpeed = Math.max(0.1, Math.min(1.0, maxAutoSpeed));

        // ---------------- Auto-drive + fine adjust ----------------
        double rTrig = gamepad1.right_trigger;
        boolean inShootingZone = forwardDist >= minForward && forwardDist <= maxForward
                && strafeOffset >= minStrafe && strafeOffset <= maxStrafe;

        if (rTrig > 0.1 && inShootingZone && forwardDist > 0) {
            double nudgeScale = 0.05;
            double nudgeForward = -gamepad1.left_stick_y * nudgeScale;
            double nudgeStrafe = gamepad1.left_stick_x * nudgeScale;

            double fError = forwardDist - targetForward + nudgeForward;
            double sError = strafeOffset - targetStrafe + nudgeStrafe;
            double rError = currentYaw - 0.0;

            double kP = 1.0;
            double kPR = 1.5;

            double forwardPower = -fError * kP;
            double strafePower = -sError * kP;
            double rotatePower = -rError * kPR;

            forwardPower = Math.max(-maxAutoSpeed, Math.min(maxAutoSpeed, forwardPower));
            strafePower = Math.max(-maxAutoSpeed, Math.min(maxAutoSpeed, strafePower));
            rotatePower = Math.max(-0.4, Math.min(0.4, rotatePower));

            if (Math.abs(forwardPower) < 0.05) forwardPower = 0;
            if (Math.abs(strafePower) < 0.05) strafePower = 0;
            if (Math.abs(rotatePower) < 0.05) rotatePower = 0;

            if (Math.abs(fError) < tolerance && Math.abs(sError) < tolerance
                    && Math.abs(rError) < rotationTolerance) stopDrive();
            else driveMecanumVector(strafePower, forwardPower, rotatePower);
        } else {
            double Vy = -gamepad1.left_stick_y;
            double Vx = gamepad1.left_stick_x * 1.1;
            double omega = gamepad1.right_stick_x;
            driveMecanumVector(Vx, Vy, omega);
        }

        // ---------------- Auto-fire ----------------
        if (gamepad1.a && !lastA && inShootingZone && !firing) {
            firing = true;
            fireTimer.reset();
            gateServo.setPosition(1.0); // open
        }
        lastA = gamepad1.a;

        if (firing && fireTimer.seconds() >= fireDuration) {
            gateServo.setPosition(0.0); // close
            firing = false;
        }

        // ---------------- Telemetry ----------------
        telemetry.addData("Auto Drive", (rTrig>0.1 && inShootingZone) ? "ACTIVE" : "OFF");
        telemetry.addData("Max Auto Speed", "%.2f", maxAutoSpeed);
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.addData("Gate Servo Pos", gateServo.getPosition());
        telemetry.addData("Firing", firing ? "YES" : "NO");
        telemetry.addData("FL", frontLeft.getPower());
        telemetry.addData("FR", frontRight.getPower());
        telemetry.addData("BL", backLeft.getPower());
        telemetry.addData("BR", backRight.getPower());
        telemetry.update();
    }

    // ---------------- Vector-based mecanum drive ----------------
    private void driveMecanumVector(double Vx, double Vy, double omega) {
        // Compute wheel powers with rotation effect based on wheel X/Y offsets
        double fl = Vy + Vx + omega * (-flY); // cross product approx
        double fr = Vy - Vx + omega * (-frY);
        double bl = Vy - Vx + omega * (-blY);
        double br = Vy + Vx + omega * (-brY);

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
