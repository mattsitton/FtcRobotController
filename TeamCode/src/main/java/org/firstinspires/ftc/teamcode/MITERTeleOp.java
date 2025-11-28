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

@TeleOp(name = "MITERTeleOp", group = "TeleOp")
public class MITERTeleOp extends OpMode {

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private VoltageSensor batteryVoltageSensor;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
    private final int BLUE_GOAL_ID = 20, RED_GOAL_ID = 24;

    // --- CONSTANTS (1:1 BARE MOTOR) ---
    // GoBilda Bare Motor (1:1) has 28 ticks per revolution
    private final double FLWHEEL_TICKS_PER_REV = 28.0;

    // PIDF: F ~= 12 for 1:1 motor
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(50, 0, 0, 12);
    private final double FEED_TIME = 0.20;

    // --- HIGH SPEED CALIBRATION (Feet vs RPM) ---
    // ESTIMATED RANGE: 2000 to 4500 RPM for a 1:1 Flywheel
    private final double[][] RPM_CALIBRATION = {
            {2.0, 2500},  // 2 ft -> 2500 RPM
            {4.0, 3000},  // 4 ft -> 3000 RPM
            {6.0, 3500},  // 6 ft -> 3500 RPM
            {8.0, 4000},  // 8 ft -> 4000 RPM
            {10.0, 4500}  // 10 ft -> 4500 RPM
    };

    // Tuning
    private final double ROTATION_KP = 0.1, ROTATION_MAX = 0.45;
    private final double STRAFE_KP = 0.7, STRAFE_MAX = 0.5;
    private final double ANGLE_TOL = 5.0, STRAFE_TOL_MM = 50.0;

    // Unit Conversion
    private final double METER_TO_FEET = 3.28084;

    // --- Global State ---
    private boolean fieldCentric = true;
    private boolean rightBumperPressed = false;
    private boolean yButtonWasPressed = false;
    private boolean optionA_Active = false; // Hold (Square Up)
    private boolean optionB_Active = false; // Toggle (Turret)

    // Flywheel State
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private boolean flywheelAtSpeed = false;

    // Auto-RPM State
    private boolean autoRPMActive = false;
    private double calculatedAutoRPM = 0.0;
    private double currentDistFeet = 0.0;

    // Vision State
    private boolean tagVisible = false;
    private double tagYaw = 0, tagBearing = 0, tagX = 0, tagY_Meters = 0;
    private int tagID = -1;

    // Feeder State
    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean launchInProgress = false;

    @Override
    public void init() {
        // Motors & Sensors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT is better for flywheels
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Pinpoint
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

        telemetry.addLine("Initialized MITERTeleOp (High Speed)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Update Sensors
        if (pinpoint != null) pinpoint.update();
        if (gamepad1.a && pinpoint != null) pinpoint.resetPosAndIMU();

        if (gamepad1.right_bumper && !rightBumperPressed) fieldCentric = !fieldCentric;
        rightBumperPressed = gamepad1.right_bumper;

        // 2. Vision Update
        AprilTagDetection target = findTargetTag();
        tagVisible = (target != null);
        if (tagVisible) {
            tagYaw = target.ftcPose.yaw;
            tagBearing = target.ftcPose.bearing;
            tagX = target.ftcPose.x;
            tagY_Meters = target.ftcPose.y;
            tagID = target.id;
            currentDistFeet = tagY_Meters * METER_TO_FEET;
        }

        // 3. Auto-RPM Calculation
        if (tagVisible) {
            calculatedAutoRPM = interpolateRPM(currentDistFeet);
        }

        // 4. Logic: Option A (Hold Square) vs Option B (Tap Turret)
        optionA_Active = gamepad1.b;
        if (gamepad1.y && !yButtonWasPressed && tagVisible) optionB_Active = true;
        yButtonWasPressed = gamepad1.y;

        // 5. Drive Calculations
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        if (optionA_Active && tagVisible) { // Square Up
            optionB_Active = false;
            rot = clamp(-ROTATION_KP * tagYaw, -ROTATION_MAX, ROTATION_MAX);
            str = clamp(-STRAFE_KP * tagX, -STRAFE_MAX, STRAFE_MAX);
            if (Math.abs(tagYaw) <= ANGLE_TOL) rot = 0;
            if (Math.abs(tagX * 1000) <= STRAFE_TOL_MM) str = 0;
        } else if (optionB_Active) { // Turret
            if (tagVisible) {
                rot = clamp(-ROTATION_KP * tagBearing, -ROTATION_MAX, ROTATION_MAX);
                if (Math.abs(tagBearing) <= ANGLE_TOL) rot = 0;
            } else { optionB_Active = false; }
            if (Math.abs(fwd) > 0.1 || Math.abs(str) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) optionB_Active = false;
        }

        // 6. Field Centric Transform
        if (fieldCentric && !optionA_Active && !optionB_Active && pinpoint != null) {
            double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            double tempStr = str * Math.cos(-heading) - fwd * Math.sin(-heading);
            fwd = str * Math.sin(-heading) + fwd * Math.cos(-heading);
            str = tempStr;
        }

        // 7. Outputs
        double voltComp = 12.0 / batteryVoltageSensor.getVoltage();
        driveMecanum(str, fwd, rot, voltComp);
        handleFlywheel(voltComp);
        handleFeeder();
        updateTelemetry();
    }

    // --- HELPER METHODS ---

    private double interpolateRPM(double distanceFeet) {
        if (distanceFeet <= RPM_CALIBRATION[0][0]) return RPM_CALIBRATION[0][1];
        if (distanceFeet >= RPM_CALIBRATION[RPM_CALIBRATION.length - 1][0]) return RPM_CALIBRATION[RPM_CALIBRATION.length - 1][1];

        for (int i = 0; i < RPM_CALIBRATION.length - 1; i++) {
            double dist1 = RPM_CALIBRATION[i][0];
            double dist2 = RPM_CALIBRATION[i+1][0];

            if (distanceFeet >= dist1 && distanceFeet <= dist2) {
                double rpm1 = RPM_CALIBRATION[i][1];
                double rpm2 = RPM_CALIBRATION[i+1][1];
                double slope = (rpm2 - rpm1) / (dist2 - dist1);
                return rpm1 + (slope * (distanceFeet - dist1));
            }
        }
        return 0;
    }

    private void handleFlywheel(double voltComp) {
        autoRPMActive = gamepad1.left_bumper; // Hold LB for Auto

        double manual = gamepad1.left_trigger;
        if (manual > 0.1) {
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(Range.clip((manual/0.8)*voltComp, 0, 1));
            currentTargetRPM = 0; flywheelAtSpeed = false; return;
        }

        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (autoRPMActive && tagVisible) {
            currentTargetRPM = (int) calculatedAutoRPM;
        } else {
            // High Speed Presets
            if (gamepad1.dpad_up)    currentTargetRPM = 4000;
            if (gamepad1.dpad_right) currentTargetRPM = 3500;
            if (gamepad1.dpad_down)  currentTargetRPM = 3000;
            if (gamepad1.dpad_left)  currentTargetRPM = 2500;
        }

        launcherMotor.setVelocity((currentTargetRPM / 60.0) * FLWHEEL_TICKS_PER_REV);
        currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
        flywheelAtSpeed = (currentTargetRPM > 0 && Math.abs(currentFlywheelRPM - currentTargetRPM) <= 150);
    }

    private AprilTagDetection findTargetTag() {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null && (detection.id == BLUE_GOAL_ID || detection.id == RED_GOAL_ID)) {
                return detection;
            }
        }
        return null;
    }

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

    private void handleFeeder() {
        if (launchInProgress) {
            if (feederTimer.seconds() > FEED_TIME) {
                leftFeeder.setPower(0); rightFeeder.setPower(0); launchInProgress = false;
            }
        } else if (gamepad1.x) {
            feederTimer.reset(); leftFeeder.setPower(1.0); rightFeeder.setPower(1.0); launchInProgress = true;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");

        if (tagVisible) {
            telemetry.addData("Vision", "ID " + tagID + " (" + String.format("%.1f", currentDistFeet) + " ft)");
        } else {
            telemetry.addData("Vision", "Searching...");
        }

        if (autoRPMActive) {
            telemetry.addData("RPM Mode", "AUTO (Target: " + (int)calculatedAutoRPM + ")");
        } else {
            telemetry.addData("RPM Mode", "MANUAL / PRESET");
        }

        telemetry.addData("Actual RPM", "%.0f / %d", currentFlywheelRPM, currentTargetRPM);
        telemetry.update();
    }

    private double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}