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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MasterTeleOp_V3_6_Cleaned", group = "TeleOp")
public class MasterTeleOp_V3_6_Cleaned extends OpMode {

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private VoltageSensor batteryVoltageSensor;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0; // Camera intrinsics
    private final int BLUE_GOAL_ID = 20, RED_GOAL_ID = 24;

    // --- Constants ---
    private final double FLWHEEL_TICKS_PER_REV = 534.8;
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(50, 0, 0, 12);
    private final double FEED_TIME = 0.20;

    // Tuning
    private final double ROTATION_KP = 0.1, ROTATION_MAX = 0.45;
    private final double STRAFE_KP = 0.7, STRAFE_MAX = 0.5;
    private final double ANGLE_TOL = 5.0, STRAFE_TOL_MM = 50.0;

    // --- State Variables (Global) ---
    private boolean fieldCentric = true;
    private boolean rightBumperPressed = false;
    private boolean yButtonWasPressed = false;
    private boolean optionA_Active = false; // Hold (Square Up)
    private boolean optionB_Active = false; // Toggle (Turret)

    // Flywheel State
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private boolean flywheelAtSpeed = false;

    // Vision State (Global for Telemetry)
    private boolean tagVisible = false;
    private double tagYaw = 0, tagBearing = 0, tagX = 0;
    private int tagID = -1;

    // Feeder State
    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean launchInProgress = false;

    @Override
    public void init() {
        // Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Launcher
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        // Servos
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sensors
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) { pinpoint = null; }

        // Vision
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy).build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor).build();

        telemetry.addLine("Initialized V3.6 (Cleaned)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Update Sensors & Inputs
        if (pinpoint != null) pinpoint.update();
        if (gamepad1.a && pinpoint != null) pinpoint.resetPosAndIMU();

        // Field Centric Toggle
        if (gamepad1.right_bumper && !rightBumperPressed) fieldCentric = !fieldCentric;
        rightBumperPressed = gamepad1.right_bumper;

        // 2. Vision Update (Using Helper)
        AprilTagDetection target = findTargetTag();
        tagVisible = (target != null);
        if (tagVisible) {
            tagYaw = target.ftcPose.yaw;
            tagBearing = target.ftcPose.bearing;
            tagX = target.ftcPose.x;
            tagID = target.id;
        }

        // 3. Logic: Option A (Hold Square) vs Option B (Tap Turret)
        optionA_Active = gamepad1.b;

        if (gamepad1.y && !yButtonWasPressed && tagVisible) optionB_Active = true;
        yButtonWasPressed = gamepad1.y;

        // 4. Calculate Drive Powers
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Alignment Overrides
        if (optionA_Active && tagVisible) {
            // Option A: Square Up (Hold)
            optionB_Active = false; // Override B
            rot = clamp(-ROTATION_KP * tagYaw, -ROTATION_MAX, ROTATION_MAX);
            str = clamp(-STRAFE_KP * tagX, -STRAFE_MAX, STRAFE_MAX);

            // Stop if aligned
            if (Math.abs(tagYaw) <= ANGLE_TOL) rot = 0;
            if (Math.abs(tagX * 1000) <= STRAFE_TOL_MM) str = 0;

        } else if (optionB_Active) {
            // Option B: Turret Aim (Latched)
            if (tagVisible) {
                rot = clamp(-ROTATION_KP * tagBearing, -ROTATION_MAX, ROTATION_MAX);
                if (Math.abs(tagBearing) <= ANGLE_TOL) rot = 0;
            } else {
                optionB_Active = false; // Lost tag
            }
            // Cancel on manual stick input
            if (Math.abs(fwd) > 0.1 || Math.abs(str) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) optionB_Active = false;
        }

        // 5. Field Centric Transform
        if (fieldCentric && !optionA_Active && !optionB_Active && pinpoint != null) {
            double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            double tempStr = str * Math.cos(-heading) - fwd * Math.sin(-heading);
            fwd = str * Math.sin(-heading) + fwd * Math.cos(-heading);
            str = tempStr;
        }

        // 6. Apply Power & Flywheel
        double voltComp = 12.0 / batteryVoltageSensor.getVoltage();
        driveMecanum(str, fwd, rot, voltComp);
        handleFlywheel(voltComp);
        handleFeeder();

        // 7. Telemetry (No arguments needed now!)
        updateTelemetry();
    }

    // --- Helper Methods ---

    // Simplified Vision Search
    private AprilTagDetection findTargetTag() {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null && (detection.id == BLUE_GOAL_ID || detection.id == RED_GOAL_ID)) {
                return detection;
            }
        }
        return null;
    }

    // Simplified Drive Method
    private void driveMecanum(double x, double y, double rot, double voltComp) {
        double slow = 1.0 - (0.7 * gamepad1.right_trigger);
        x *= slow; y *= slow; rot *= slow;

        double fl = (y + x + rot);
        double fr = (y - x - rot);
        double bl = (y - x + rot);
        double br = (y + x - rot);

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl/=max; fr/=max; bl/=max; br/=max; }

        frontLeft.setPower(Range.clip(fl * voltComp, -1, 1));
        frontRight.setPower(Range.clip(fr * voltComp, -1, 1));
        backLeft.setPower(Range.clip(bl * voltComp, -1, 1));
        backRight.setPower(Range.clip(br * voltComp, -1, 1));
    }

    // Consolidated Flywheel Logic
    private void handleFlywheel(double voltComp) {
        double manual = gamepad1.left_trigger;
        if (manual > 0.1) {
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(Range.clip((manual/0.8)*voltComp, 0, 1));
            currentTargetRPM = 0;
            flywheelAtSpeed = false;
            return;
        }

        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gamepad1.dpad_up)    currentTargetRPM = 300;
        if (gamepad1.dpad_right) currentTargetRPM = 280;
        if (gamepad1.dpad_down)  currentTargetRPM = 260;
        if (gamepad1.dpad_left)  currentTargetRPM = 240;

        launcherMotor.setVelocity((currentTargetRPM / 60.0) * FLWHEEL_TICKS_PER_REV);
        currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
        flywheelAtSpeed = (currentTargetRPM > 0 && Math.abs(currentFlywheelRPM - currentTargetRPM) <= 100);
    }

    private void handleFeeder() {
        if (launchInProgress) {
            if (feederTimer.seconds() > FEED_TIME) {
                leftFeeder.setPower(0); rightFeeder.setPower(0); launchInProgress = false;
            }
        } else if (gamepad1.x) {
            feederTimer.reset(); leftFeeder.setPower(1.0); rightFeeder.setPower(1.0); launchInProgress = true;
        }
    }

    // Simplified Telemetry (Reads Global Vars)
    private void updateTelemetry() {
        telemetry.addData("Drive", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Vision Mode", optionA_Active ? "SQUARE (HOLD)" : (optionB_Active ? "TURRET (LATCH)" : "MANUAL"));
        telemetry.addData("Target", tagVisible ? "ID " + tagID : "Searching...");

        if (tagVisible) {
            telemetry.addData("Align", "Yaw:%.1f | Bear:%.1f | X:%.2f", tagYaw, tagBearing, tagX);
        }

        telemetry.addLine("-----");
        telemetry.addData("RPM", "%.0f / %d", currentFlywheelRPM, currentTargetRPM);
        telemetry.addData("Ready", flywheelAtSpeed);
        telemetry.update();
    }

    private double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}