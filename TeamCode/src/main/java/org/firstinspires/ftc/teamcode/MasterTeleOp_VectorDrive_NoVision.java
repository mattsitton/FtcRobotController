// File: MasterTeleOp_NoVision.txt
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

@TeleOp(name = "MasterTeleOp_NoVision", group = "TeleOp")
public class MasterTeleOp_VectorDrive_NoVision extends OpMode {

    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Mecanum Wheel Geometry (in meters from robot center) ---
    private final double flX = 0.15, flY = 0.15;
    private final double frX = 0.15, frY = -0.075;
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

        telemetry.addLine("Master TeleOp (No Vision) Initialized.");
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

        // --- Flywheel Control ---
        handleFlywheel(rightTrigger);

        // --- Apply Driver Enhancements to manual inputs ---
        double slowFactor = 1.0 - (0.7 * leftTrigger);
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        double forwardCmd = stickForward * slowFactor * voltageComp;
        double strafeCmd  = stickStrafe  * slowFactor * voltageComp;
        double rotateCmd  = stickRotate  * slowFactor * voltageComp;

        // --- Final Drive Command ---
        driveMecanumVectorOffset(strafeCmd, forwardCmd, rotateCmd);

        // --- Launcher State Machine ---
        handleLauncherState(pressX, flywheelAtSpeed);

        // --- Telemetry ---
        updateTelemetry();
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

        // --- CORRECTED SECTION ---
        // Find the maximum absolute power among all wheels
        double max = Math.abs(flPower);
        if (Math.abs(frPower) > max) {
            max = Math.abs(frPower);
        }
        if (Math.abs(blPower) > max) {
            max = Math.abs(blPower);
        }
        if (Math.abs(brPower) > max) {
            max = Math.abs(brPower);
        }

        // If the maximum is greater than 1.0, scale all powers down
        // to preserve the drive proportions.
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

    private void updateTelemetry() {
        telemetry.addData("Mode", "MANUAL");
        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addData("Launcher Target", "%d RPM", currentTargetRPM);
        telemetry.addData("Flywheel Ready", flywheelAtSpeed);
        telemetry.update();
    }

    private double rpmToTicksPerSecond(double rpm) { return (rpm / 60.0) * 28.0; }
    private double ticksPerSecondToRPM(double ticksPerSec) { return (ticksPerSec / 28.0) * 60.0; }

    @Override
    public void stop() {
        launcherMotor.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}