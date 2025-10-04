
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="FTCDECODEV2_AutoDriveVision", group="TeleOp")
public class FTCDECODEV2_AutoDriveVision extends OpMode {

    // ---------------- Drive ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ---------------- Launcher & Feeder ----------------
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private final double FEED_TIME_SECONDS = 0.2;
    private final double FULL_FEED_POWER = 1.0;
    private final int LOW_RPM = 2500;
    private final int HIGH_RPM = 3500;
    private final int MIN_RPM_THRESHOLD = 2450;
    private ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState { IDLE, LAUNCH, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;

    // ---------------- Intake (Disabled) ----------------
    /*
    private CRServo intakeLeft, intakeRight;
    private boolean intakeOn = false;
    private boolean lastB = false;
    */

    // ---------------- Vision ----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private double fx = 600.0, fy = 600.0;
    private double cx = 320.0, cy = 240.0;

    // ---------------- Mecanum Drive Geometry ----------------
    private double flX = 0.15, flY = 0.15;
    private double frX = 0.15, frY = -0.075;
    private double blX = -0.15, blY = 0.15;
    private double brX = -0.15, brY = -0.15;

    @Override
    public void init() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Launcher + feeders
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300,0,0,10));

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Disabled intake (future)
        /*
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeRight.setDirection(CRServo.Direction.REVERSE);
        */

        // Vision
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---------------- Auto-drive & manual nudge ----------------
        double lt = gamepad1.left_trigger; // Auto-drive engagement
        double rTrig = gamepad1.right_trigger; // Flywheel speed

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Auto-drive scaling
        boolean autoDrive = lt > 0.05;
        double autoScale = 1.0 - lt; // more pressed = slower

        if(autoDrive){
            // Example: modify forward/strafe/rotate based on vision
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if(!detections.isEmpty()){
                AprilTagDetection tag = detections.get(0);
                // Simple auto-drive: move forward/back to align Y axis
                double fError = tag.ftcPose.y - 1.2; // target forward 1.2m
                forward = -fError * autoScale;
                // Strafe can also be adjusted similarly
                double sError = tag.ftcPose.x;
                strafe = -sError * autoScale;
                rotate = -tag.ftcPose.yaw * autoScale;
            }
            // Allow driver nudge
            forward += -gamepad1.left_stick_y * 0.2;
            strafe += gamepad1.left_stick_x * 0.2;
            rotate += gamepad1.right_stick_x * 0.2;
        }

        driveMecanumVectorOffset(strafe, forward, rotate);

        // ---------------- Flywheel control ----------------
        if(rTrig > 0.8) launcherMotor.setVelocity(rpmToTicksPerSecond(HIGH_RPM));
        else if(rTrig > 0.3) launcherMotor.setVelocity(rpmToTicksPerSecond(LOW_RPM));
        else launcherMotor.setVelocity(0);

        // ---------------- Launcher X-shot ----------------
        handleLauncherState(gamepad1.x);

        // ---------------- Telemetry ----------------
        telemetry.addData("Auto-drive Active", autoDrive);
        telemetry.addData("Launcher RPM", ticksPerSecondToRPM(launcherMotor.getVelocity()));
        telemetry.addData("Launcher State", launchState);
        telemetry.update();
    }

    private void handleLauncherState(boolean xPressed){
        double currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());

        switch(launchState){
            case IDLE:
                if(xPressed && (currentRPM >= LOW_RPM)){
                    launchState = LaunchState.LAUNCH;
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_FEED_POWER);
                rightFeeder.setPower(FULL_FEED_POWER);
                launchState = LaunchState.LAUNCHING;
                feederTimer.reset();
                break;
            case LAUNCHING:
                if(feederTimer.seconds() > FEED_TIME_SECONDS){
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    private double rpmToTicksPerSecond(double rpm){
        return (rpm/60.0)*28.0; // assuming 28 ticks per rev
    }

    private double ticksPerSecondToRPM(double ticksPerSec){
        return (ticksPerSec/28.0)*60.0;
    }

    private void driveMecanumVectorOffset(double Vx, double Vy, double omega){
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

    @Override
    public void stop(){
        if(visionPortal != null) visionPortal.close();
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        launcherMotor.setVelocity(0);
    }
}
